#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include "see_sensor.h"
#include <time.h>

// Used to capture ctrl-c signal to allow graceful exit
void intHandler(int dummy) {
    printf("Got SIGINT, exiting\n");
    exit(0);
}

void help() {
    printf("Usage: test_see_sensor_icm4x6xx <options>\n");
    printf("Options:\n");
    printf("-p   Set path for logging data");
    printf("-t   Set duration in seconds\n");
    printf("-d   Show extra debug messages.\n");
    printf("-c   Continuous test. Enter ctrl-c to end.\n");
    printf("-h   Show help.\n");
}

int main(int argc, char *argv[]) {
    int opt = 0;
    int continuous_test = 0;
    see_sensor_icm4x6xx_imu_data_t data;

    FILE* logging_path;
    int logging = 0;
    int duration = 10; //default logging duration
    int rate = 500; //sample collection rate

    printf("Test SEE sensor ICM4x6xx starting\n");

    // Parse all command line options
    while ((opt = getopt(argc, argv, "p:t:r:")) != -1) {
        switch (opt) {
        case 'p':
            logging_path = fopen(optarg, "w");
            fprintf(logging_path,"Timestamp,DSP Timestamp,Temperature,Accel1,Gyro1,Accel2,Gyro2,Accel3,Gyro3");
            logging = 1;
            printf("Path is recorded");
        case 't':
            duration = atoi(optarg);
            printf("time is set");
            break;
        case 'r':
            rate = atoi(optarg);
            break;
        case ':':
            fprintf(stderr, "Error - option %c needs a value\n\n", optopt);
            help();
            return -1;
        case '?':
            fprintf(stderr, "Error - unknown option: %c\n\n", optopt);
            help();
            return -1;
        }
    }

    // optind is for the extra arguments which are not parsed
    for (; optind < argc; optind++) {
        fprintf(stderr, "extra arguments: %s\n", argv[optind]);
        help();
        return -1;
    }

    // Setup our signal handler to catch ctrl-c
    // signal(SIGINT, intHandler);

    if (see_sensor_icm4x6xx_detect()) {
        fprintf(stderr, "Failed to detect SEE ICM4X6XX\n");
        return -1;
    } else {
        printf("Detected SEE ICM4X6XX\n");
    }

    if (see_sensor_icm4x6xx_init()) {
        fprintf(stderr, "Failed to initialize SEE ICM4X6XX\n");
        return -1;
    } else {
        printf("Initialized SEE ICM4X6XX\n");
    }

    sleep(1);

    // Try a few reads
    unsigned int i = 0;
    clock_t start, curr;
    double running;
    double timestamp_decimal;
    start = clock();
    while(logging){
        // The read is blocking and will only return when there is data available.
        if (see_sensor_icm4x6xx_read(&data)) {
            fprintf(stderr, "Failed basic read test %d\n", i);
            return -1;
        } else {
            
            fprintf(logging_path, "\n%lu,%lu,%f,",data.timestamp_apps_real,data.timestamp_slpi_ticks, data.temperature);
            for (unsigned int j = 0; j < 3; j++) {
                if (j != 2)
                    fprintf(logging_path, "%f,%f,",data.accl_ms2[j],data.gyro_rad[j]);
                else
                    fprintf(logging_path, "%f,%f",data.accl_ms2[j],data.gyro_rad[j]);
            }
        }
        // grab time and exit if past duration
        curr = clock() - start;
        running = (double)(curr)/CLOCKS_PER_SEC;
        if (duration != 0 && running*10 >= duration){
            fclose(logging_path);
            break;      
        }
    }

    if (see_sensor_icm4x6xx_close()) {
        fprintf(stderr, "Failed to close SEE ICM4X6XX interface\n");
        return -1;
    } else {
        printf("Closed SEE ICM4X6XX interface\n");
    }

    sleep(1);

    printf("Test PASSED!\n");

    return 0;
}
