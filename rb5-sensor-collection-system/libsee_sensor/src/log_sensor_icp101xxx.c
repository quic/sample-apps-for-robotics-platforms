#include <time.h>
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
    printf("-p   Set path for logging data");
    printf("-t   Set duration in seconds\n");
    printf("-d   Show extra debug messages.\n");
    printf("-c   Continuous test. Enter ctrl-c to end.\n");
    printf("-h   Show help.\n");
}
int main(int argc, char *argv[]) {
    int opt = 0;
    int continuous_test = 0;
    see_sensor_icp101xx_barometer_data_t data;

    int rate = 30;
    int duration = 5;
    int logging = 0;
    FILE* logging_path;

    printf("Logging ICP101XX starting\n");

    // Parse all command line options
    while ((opt = getopt(argc, argv, "t:r:p:")) != -1) {
        switch (opt) {
        case 'r':
            rate = atoi(optarg);
            break;
        case 't':
            duration = atoi(optarg);
            break;
        case 'p':
            logging_path = fopen(optarg, "w");
	        fprintf(logging_path, "Timestamp,DSP Timestamp,Pressure,Temperature\n");
            logging = 1;
            break;
        case ':':
            return -1;
        case '?':
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

    if (see_sensor_icp101xx_detect()) {
        fprintf(stderr, "Failed to detect SEE ICP101XX\n");
        return -1;
    } else {
        printf("Detected SEE ICP101XX\n");
    }

    if (see_sensor_icp101xx_init()) {
        fprintf(stderr, "Failed to initialize SEE ICP101XX\n");
        return -1;
    } else {
        printf("Initialized SEE ICP101XX\n");
    }

    sleep(1);

    // Try a few reads
    clock_t start, curr;
    double running;
    start = clock();
    while (logging) {
        if (see_sensor_icp101xx_read(&data)) {
            return -1;
        } else {
            fprintf(logging_path, "%lu,%lu,%f,%f\n", data.timestamp_apps_real, data.timestamp_slpi_ticks, data.pressure, data.temperature);
        }
        curr = clock() - start;
        running = (double)(curr)/CLOCKS_PER_SEC;
        if (duration != 0 && running*10 >= duration){
            fclose(logging_path);
            break;
        }
    }

    if (see_sensor_icp101xx_close()) {
        fprintf(stderr, "Failed to close SEE ICP101XX interface\n");
        return -1;
    } else {
        printf("Closed SEE ICP101XX interface\n");
    }

    sleep(1);

    printf("Test PASSED!\n");

    return 0;
}
