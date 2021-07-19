/*
 * Copyright (c) 2018-2020 InvenSense, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *
 * You may obtain a copy of the License at
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Modifications Copyright (c) 2021 ModalAI Inc.
 *
 * For terms, see LICENSE file.
 */

#include<stdio.h>
#include<stdlib.h>
#include<fcntl.h>
#include<dirent.h>
#include<math.h>
#include<unistd.h>
#include<string.h>
#include<errno.h>
#include<getopt.h>
#include<signal.h>
#include<time.h>
#include<linux/types.h>

#define VER_MAJOR (0)
#define VER_MINOR (3)
#define SUCCESS (0)
#define FAILURE (-1)
#define TEMP_SENSOR_B   (3988)
#define TEMP_R_INF	  (0.0155223)
#define MAX_SYSFS_NAME_LEN      (100)
#define NS_IN_SEC (1000000000.00)
#define ARRAY_SIZE(x) sizeof(x)

//#define DEBUG

#ifdef DEBUG
#define DPRINT printf
#else
#define DPRINT
#endif

#define IIO_DEVICE "/dev/iio:device%lu"
#define TRIGGER_NAME "tdk_thermistor-hrtimer%lu"
#define THERMISTOR_NAME "tdk_thermistor"

/* sysfs to control chip */
#define SYSFS_PATH              "/sys/bus/iio/devices/iio:device%lu"
#define SYSFS_CHIP_NAME         "name"
#define SYSFS_CHIP_ENABLE       "buffer/enable"
#define SYSFS_BUFFER_LENGTH     "buffer/length"
#define SYSFS_TEMPERATURE_EN    "scan_elements/in_temp_en"
#define SYSFS_TIMESTAMP_EN      "scan_elements/in_timestamp_en"
#define SYSFS_CURRENT_TRIGGER   "trigger/current_trigger"
#define IIO_DIR                 "/sys/bus/iio/devices/"

#define THERMISTOR_LOG_FILE     "temperature_sensor.csv"


static const char tdk_therm_name[] = "tdk_thermistor";

/* commandline options */
static const struct option options[] = {
    {"help", no_argument, NULL, 'h'},
    {"enable", required_argument, NULL, 'e'},
    {"time", required_argument, NULL, 't'},
    {"path", required_argument, NULL, 'p'},
    {0, 0, 0, 0},
};

static const char *options_descriptions[] = {
    "Show this help and quit.",
    "Turn thermistor buffer on/off.",
};


static char iio_device[1024] = "";
static char trigger_name[1024] = "";
static char sysfs_path[1024] = "";
static int  iio_fd = -1;
static FILE *thermistor_log_fp;

/**
 * find_type_by_name() - function to match top level types by name
 * @name: top level type instance name
 * @type: the type of top level instance being sort
 *
 * Typical types this is used for are device and trigger.
 **/
static int find_type_by_name(const char *name, const char *type)
{
        const struct dirent *ent;
        int number, numstrlen;

        FILE *nameFile;
        DIR *dp;
        char thisname[MAX_SYSFS_NAME_LEN];
        char filename[MAX_SYSFS_NAME_LEN];
        size_t filename_sz = 0;
        int ret = 0;
        int status = -1;

        dp = opendir(IIO_DIR);
        if (dp == NULL) {
                printf("No industrialio devices available\n");
                return -1;
        }

        while (ent = readdir(dp), ent != NULL) {
                if (strcmp(ent->d_name, ".") != 0 &&
                    strcmp(ent->d_name, "..") != 0 &&
                    strlen(ent->d_name) > strlen(type) &&
                    strncmp(ent->d_name, type, strlen(type)) == 0) {

                        //printf("name: %s\n", ent->d_name);

                        numstrlen = sscanf(ent->d_name + strlen(type),
                                           "%d", &number);
                        filename_sz = strlen(IIO_DIR)
                                        + strlen(type)
                                        + numstrlen
                                        + 6;
                        /* verify the next character is not a colon */
                        if (strncmp(ent->d_name + strlen(type) + numstrlen,
                                        ":", 1) != 0) {

                                snprintf(filename, filename_sz, "%s%s%d/name",
                                        IIO_DIR, type, number);

                                nameFile = fopen(filename, "r");
                                if (!nameFile)
                                        continue;
                                ret = fscanf(nameFile, "%s", thisname);
                                fclose(nameFile);

                                if (ret == 1 && strcmp(name, thisname) == 0) {
                                        status = number;
                                        goto exit_closedir;
                                }
                        }
                }
        }

exit_closedir:
        closedir(dp);
        return status;
}

static int process_sysfs_request()
{
        int dev_num = find_type_by_name(THERMISTOR_NAME, "iio:device");
        if (dev_num < 0)
                return -1;

        //sprintf(data, IIO_DIR "iio:device%d", dev_num);
        return dev_num;
}

/*Process the thermistor reading, convert to degree celcius*/
static float process_thermistor_sensor_val(u_int16_t temp)
{
	float temp_voltage;
	float temp_resistance;
	float temp_c;

	DPRINT("Temp adc value: %u\n", temp);
	temp_voltage = (temp / 16368.0f) * 3.3f;
	DPRINT("Temperature voltage: %f\n", temp_voltage);

	temp_resistance = (33000/temp_voltage) - 10000;
	if (temp_resistance < 0.0f) {
		temp_resistance = 0.0f;
	}
	DPRINT("Temperature resistance: %f\n", temp_resistance);

	temp_c = (TEMP_SENSOR_B / log(temp_resistance/TEMP_R_INF)) - 273.15;
	DPRINT("Temperature degC: %f\n", temp_c);

	return temp_c;
}

/*Get the time stamp*/
static u_int64_t process_timestamp(u_int8_t *buff)
{
	u_int8_t i;
	u_int64_t ts = 0;
	u_int64_t t1 = 0;

	for (i = 0; i < 8; i++) {
		t1 = buff[i];
		ts |= (t1 << (i*8));
	}

	DPRINT("TS = %lx\n", ts);
	return ts;
}

/*Helper function to write "int" value to sysfs file*/
static int write_sysfs_int(char *attr_name, int data)
{
	int ret;
	FILE *fp;
	char path[1024];

	ret = snprintf(path, sizeof(path), "%s/%s", sysfs_path, attr_name);
	if (ret < 0 || ret >= (int)sizeof(path)) {
		return ret;
	}

	ret = 0;
	//printf("sysfs: %d -> %s\n", data, path);
	fp = fopen(path, "w");
	if (fp == NULL) {
		ret = -errno;
		printf("Failed to open %s\n", path);
	} else {
		if (fprintf(fp, "%d\n", data) < 0) {
			printf("Failed to write to %s\n", path);
			ret = -errno;
		}
		fclose(fp);
	}
	fflush(stdout);
	return ret;
}

/*Helper function to write "str" value to sysfs file*/
static int write_sysfs_str(char *attr_name, char *val)
{
	int ret;
	FILE *fp;
	char path[1024];

	ret = snprintf(path, sizeof(path), "%s/%s", sysfs_path, attr_name);
	if (ret < 0 || ret >= (int)sizeof(path)) {
		return ret;
	}

	ret = 0;
	//printf("sysfs: %s -> %s\n", val, path);
	fp = fopen(path, "w");
	if (fp == NULL) {
		ret = -errno;
		printf("Failed to open %s\n", path);
	} else {
	if (fprintf(fp, "%s\n", val) < 0) {
		printf("Failed to write to %s\n", path);
		ret = -errno;
	}
	fclose(fp);
	}
	fflush(stdout);
	return ret;
}

/* show usage */
static void usage(void)
{
	unsigned int i;

	printf("Usage:\n\t test-thermistor [-e <enable/disable>]"
					"\n\nOptions:\n");
	for (i = 0; options[i].name; i++)
		printf("\t-%c, --%s\n\t\t\t%s\n",
	options[i].val, options[i].name,
	options_descriptions[i]);
	fflush(stdout);
}

/*Verify if the iio_device is tdk_thermistor*/
static int verify_iio_device_name(void)
{
	FILE *fp;
	char name[256];
	char path[1024];
	int ret;

	ret = snprintf(path, sizeof(path), "%s/%s", sysfs_path, SYSFS_CHIP_NAME);
	if (ret < 0 || ret >= (int)sizeof(path)) {
		return ret;
	}

	ret = 0;
	fp = fopen(path, "r");
	if (fp == NULL) {
		ret = errno;
		printf("Failed to open %s\n", path);
	} else {
		if (fscanf(fp, "%s", name) != 1) {
			ret = errno;
			printf("Failed to read chip name: %s\n", strerror(errno));
		} else {
			//printf("Chip name = %s\n", name);
			if (strncmp(tdk_therm_name, name, sizeof(tdk_therm_name)) == 0)
				ret = 0;
			else
				ret = -1;
		}
		fclose(fp);
	}
	fflush(stdout);
	return ret;

}

/*Enable iio buffering*/
static int enable_iio_trigger_buffer(void)
{
	int ret;

	/*echo 1 > $dev/scan_elements/in_temp_en*/
	ret = write_sysfs_int(SYSFS_TEMPERATURE_EN, 1);

	/*echo 1 > $dev/scan_elements/in_timestamp_en*/
	ret = write_sysfs_int(SYSFS_TIMESTAMP_EN, 1);

	/*echo $trigger > $dev/trigger/current_trigger*/
	ret = write_sysfs_str(SYSFS_CURRENT_TRIGGER, trigger_name);

	/*echo 1 > $dev/buffer/enable*/
	ret = write_sysfs_int(SYSFS_CHIP_ENABLE, 1);

	return ret;
}

/*Disable iio buffering*/
static int disable_iio_trigger_buffer(void)
{
	int ret;

	/*echo 1 > $dev/buffer/enable*/
	ret = write_sysfs_int(SYSFS_CHIP_ENABLE, 0);

	return ret;
}

/*Open log file and fill the header*/
static int init_log_file(void)
{
	int ret = 0;

	if (thermistor_log_fp == NULL) {
		ret = errno;
		printf("Error opening file for logging:%s\n", strerror(errno));
		return ret;
	}

	if (fprintf(thermistor_log_fp, "# Chirp Microsystems redswallow Data Log\n") < 0) {
		printf("Failed to write to %s\n", THERMISTOR_LOG_FILE);
		ret = -errno;
	}

	if (fprintf(thermistor_log_fp, "# Non-Chirp Sensors\n") < 0) {
		printf("Failed to write to %s\n", THERMISTOR_LOG_FILE);
		ret = -errno;
	}

	if (fprintf(thermistor_log_fp, "# SENSOR_TEMP (SensorID=5)\n") < 0) {
		printf("Failed to write to %s\n", THERMISTOR_LOG_FILE);
		ret = -errno;
	}

	if (fprintf(thermistor_log_fp, "# time[s], SensorID, temp_c\n") < 0) {
		printf("Failed to write to %s\n", THERMISTOR_LOG_FILE);
		ret = -errno;
	}

	return ret;
}

/*Close the log file*/
static void close_log_file(void)
{
	if (thermistor_log_fp != NULL)
		fclose(thermistor_log_fp);
}

/* signal handler to disable sensors when ctr-C */
static void sig_handler(int s)
{
	int ret = 0;

	(void)s;

	printf("Disable buffer\n");
	ret = disable_iio_trigger_buffer();
	if (ret) {
	    printf("failed to disable buffer\n");
	    fflush(stdout);
	    return;
	}

	/* close */
	if (iio_fd != -1) {
	    close(iio_fd);
	    iio_fd = -1;
	}

	/*close logfile */
	printf("Close log file");
	close_log_file();

	fflush(stdout);
	exit(1);
}

/*Application entry point, command line takes iio buffer enable/disable option*/
int main(int argc, char *argv[])
{
	int ret = 0;
	u_int8_t rd_buf[16];
	u_int16_t val = 0;
	u_int64_t ts = 0;
	int8_t device_no = -1;
	u_int64_t therm_enable = 0;
	int i;
    int time = 0;
	float temperature_c;
	int opt, option_index;
	struct sigaction sig_action;
    FILE* logging_path; // actuall logging path used
    int logging = 0;

	if (argc < 2) {
		usage();
		printf("Insufficient arguments\n");
		ret = EINVAL;
		return ret;
	}

        //printf("TDK-Robotics-RB5-therm-app-%d.%d\n",VER_MAJOR,VER_MINOR);
        // get absolute IIO path & build MPU's sysfs paths
        device_no = process_sysfs_request();
        if (device_no < 0) {
                 printf("Cannot find %s sysfs path\n", THERMISTOR_NAME);
		 ret = EAGAIN;
                 return ret;
        }

	while ((opt = getopt_long(argc, argv, "hd:e:t:p:", options, &option_index)) != -1) {
		switch (opt) {
		case 'e':
			therm_enable = strtoul(optarg, NULL, 10);
		    break;
		case 't':
			time = atoi(optarg);
		    break;
		case 'p':
			thermistor_log_fp = fopen(optarg, "w+");
            logging = 1;
		    break;
		case 'h':
			usage();
		return 0;
	     }
	 }

	/* signal handling */
	sig_action.sa_handler = sig_handler;
	sigemptyset(&sig_action.sa_mask);
	sig_action.sa_flags = 0;
	sigaction(SIGINT, &sig_action, NULL);

	/*Set-up device name*/
	ret = snprintf(iio_device, sizeof(iio_device), IIO_DEVICE, device_no);
	if (ret < 0 || ret >= (int)sizeof(iio_device)) {
		printf("Error %d, cannot set iio_device\n", ret);
		fflush(stdout);
		return -errno;
	}

	/*Set-up device name*/
	ret = snprintf(trigger_name, sizeof(trigger_name), TRIGGER_NAME, device_no);
	if (ret < 0 || ret >= (int)sizeof(trigger_name)) {
		printf("Error %d, cannot set iio buffer trigger\n", ret);
		fflush(stdout);
		return -errno;
	}

	/*Set-up sysfs path*/
	ret = snprintf(sysfs_path, sizeof(sysfs_path), SYSFS_PATH, device_no);
	if (ret < 0 || ret >= (int)sizeof(sysfs_path)) {
		printf("Error %d, cannot set sysfs_path\n", ret);
		fflush(stdout);
		return -errno;
	}

	/*Verify IIO device name*/
	ret = verify_iio_device_name();
	if (ret < 0) {
		printf("Wrong IIO device number\n");
		return ret;
	}

    // begin setting up duration counter
    struct timespec curr, start;
    int time_elapsed = 0;
    clockid_t clk_id = CLOCK_REALTIME;

	if (therm_enable) {
		ret = enable_iio_trigger_buffer();
		if (ret == SUCCESS) {
			iio_fd = open(iio_device, O_RDWR);
			if (iio_fd < 0) {
				printf("IIO: device open failed\n");
				ret = -1;
			}
			/*ToDo: Flust the buffer by reading; before logging to file. Check better way*/
			if (read(iio_fd, rd_buf, ARRAY_SIZE(rd_buf)) != ARRAY_SIZE(rd_buf)) {
				printf("IIO device Read Error:%s\n", strerror(errno));
				ret = -1;
			}
            
            // init log file
            init_log_file();

            // start time
            clock_gettime(clk_id, &start);
			while (logging) {
				if (read(iio_fd, rd_buf, ARRAY_SIZE(rd_buf)) != ARRAY_SIZE(rd_buf)) {
					printf("IIO device Read Error:%s\n", strerror(errno));
                    return -1;
                } else {
				    val = ((rd_buf[0] & 0x7F) << 7) | (rd_buf[1] >> 1);
				    temperature_c = process_thermistor_sensor_val(val);
				    ts = process_timestamp(&rd_buf[8]);
				    ret = 0;
				    
				    fprintf(thermistor_log_fp, "%f,5,%f\n", (double)(ts/NS_IN_SEC), temperature_c);
				    
                }
                clock_gettime(clk_id, &curr);
                time_elapsed = curr.tv_sec - start.tv_sec;

                if (time != 0 && time_elapsed >= time){
                    close_log_file();
                    logging = 0;
                    break;
                }
			}
		} else {
			printf("Failed to enable IIO buffer mechanism\n");
		}
	} else {
		ret = disable_iio_trigger_buffer();
	}
	return ret;
}
