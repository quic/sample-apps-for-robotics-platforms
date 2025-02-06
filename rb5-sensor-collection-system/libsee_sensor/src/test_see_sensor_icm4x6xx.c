
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include "see_sensor.h"

// Used to capture ctrl-c signal to allow graceful exit
void intHandler(int dummy) {
    printf("Got SIGINT, exiting\n");
    exit(0);
}

void help() {
    printf("Usage: test_see_sensor_icm4x6xx <options>\n");
    printf("Options:\n");
    printf("-d   Show extra debug messages.\n");
    printf("-c   Continuous test. Enter ctrl-c to end.\n");
    printf("-h   Show help.\n");
}

int main(int argc, char *argv[]) {
    int opt = 0;
    int continuous_test = 0;
    see_sensor_icm4x6xx_imu_data_t data;

    printf("Test SEE sensor ICM4x6xx starting\n");

    // Parse all command line options
    while ((opt = getopt(argc, argv, "dch")) != -1) {
        switch (opt) {
        case 'd':
            printf("Enabling debug messages\n");
            see_sensor_icm4x6xx_enable_debug_messages();
            break;
        case 'c':
            printf("Enabling continuous test. Hit ctrl-c to exit\n");
            continuous_test = 1;
            break;
        case 'h':
            help();
            return -1;
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
    for (; i < 3; i++) {
        // The read is blocking and will only return when there is data available.
        if (see_sensor_icm4x6xx_read(&data)) {
            fprintf(stderr, "Failed basic read test %d\n", i);
            return -1;
        } else {
            printf("Read %d succeeded\n", i);
            printf("\tSLPI timestamp: %lu 19.2MHz ticks\n", data.timestamp_slpi_ticks);
            printf("\tLocal timestamp: %lu ns\n", data.timestamp_apps_real);
            printf("\tTemperature: %f\n", data.temperature);
            for (unsigned int j = 0; j < 3; j++) {
                printf("\tAccel[%u] = %f, Gyro[%u] = %f\n", j, data.accl_ms2[j], j, data.gyro_rad[j]);
            }
        }
        usleep(250000);
    }

    if (continuous_test) {
        printf("Entering continuous test mode. Hit ctrl-c to exit.\n");
        i = 0;
        while(1) {
            if (see_sensor_icm4x6xx_read(&data)) {
                fprintf(stderr, "Failed continuous mode read test %d\n", i);
                return -1;
            }
            i++;
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
