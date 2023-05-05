/*
* Copyright (c) 2020, 2022 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <stdlib.h>
#include <errno.h>
#include <poll.h>
#include <linux/input.h>

#define GPIO_CHIP           399
#define FILE_LED            "/sys/class/leds/%s/brightness"
#define FILE_KEY_EVENT      "/dev/input/event%s"
#define FILE_KEY_EVENT0     "/dev/input/event0"
#define FILE_KEY_EVENT1     "/dev/input/event1"
#define FILE_KEY_EVENT2     "/dev/input/event2"
#define FILE_KEY_EVENT3     "/dev/input/event3"
#define FILE_GPIO_EXPORT    "/sys/class/gpio/export"
#define FILE_GPIO_UNEXPORT  "/sys/class/gpio/unexport"
#define FILE_GPIO_DIRECTION "/sys/class/gpio/gpio%d/direction"
#define FILE_GPIO_VALUE     "/sys/class/gpio/gpio%d/value"
#define FILE_GPIO_EDGE      "/sys/class/gpio/gpio%d/edge"
#define FILE_PWM_EXPORT     "/sys/class/pwm/pwmchip5/export"
#define FILE_PWM_UNEXPORT   "/sys/class/pwm/pwmchip5/unexport"
#define FILE_PWM_PERIOD     "/sys/class/pwm/pwmchip5/pwm%d/period"
#define FILE_PWM_DUTY_CYCLE "/sys/class/pwm/pwmchip5/pwm%d/duty_cycle"
#define FILE_PWM_ENABLE     "/sys/class/pwm/pwmchip5/pwm%d/enable"
#define FILE_TEMPERATURE    "/sys/class/thermal/thermal_zone%d/temp"
#define FILE_REGULATOR      "/sys/kernel/debug/regulator/18200000.rsc:rpmh-regulator-%s/enable"

static int led_control(const char* led_name, unsigned int brightness)
{
    char led_file_name[128] = {0};
    char write_buf[5] = {0};
    int fd;

    snprintf(led_file_name, sizeof(led_file_name), FILE_LED, led_name);
    snprintf(write_buf, sizeof(write_buf), "%d", brightness);

    fd = open(led_file_name, O_RDWR);
    if (fd < 0) {
        printf("Open File: %s error!\n", led_file_name);
        return -ENOENT;
    }

    write(fd, write_buf, sizeof(write_buf));

    return 0;
}

void* key_event(void* arg)
{
    int fd;
    int i;
    char event_file_name[24] = {0};
    struct input_event event;

    snprintf(event_file_name, sizeof(event_file_name), FILE_KEY_EVENT, (char *)arg);

    if ((fd = open(event_file_name, O_RDWR, 0)) >= 0) {
        for (i = 0; i < LED_MAX; i++) {
            event.time.tv_sec  = time(0);
            event.time.tv_usec = 0;
            event.type         = EV_LED;
            event.code         = i;
            event.value        = 0;
            write(fd, &event, sizeof(event));
        }
    } else {
        printf("open file: %s error!", event_file_name);

        return NULL;
    }

    while ((read(fd, &event, sizeof(event))) > 0)
    {
        printf("%-24.24s.%06lu type 0x%04x; code 0x%04x;"
                " value 0x%08x; ",
                ctime(&event.time.tv_sec),
                event.time.tv_usec,
                event.type, event.code, event.value);
        if (event.code > BTN_MISC) {
            printf("Button %d %s\n",
                    event.code & 0xff,
                    event.value ? "press" : "release");
        } else {
            printf("Key %d (0x%02x) %s\n",
                    event.code & 0xff,
                    event.code & 0xff,
                    event.value ? "press" : "release");
        }
    }

    return NULL;
}

int gpio_control(unsigned int gpio_number, int direction, unsigned int* gpio_value)
{
    char gpio_file_name[128] = {0};
    char write_buf[5] = {0};
    char read_buf[2] = {0};
    int fd;

    snprintf(write_buf, sizeof(write_buf), "%d", gpio_number + GPIO_CHIP);

    fd = open(FILE_GPIO_EXPORT, O_WRONLY);
    if (fd < 0)
    {
        printf("Open File: %s error!\n", FILE_GPIO_EXPORT);
        return -ENOENT;
    }

    write(fd, write_buf, sizeof(write_buf));
    close(fd);

    snprintf(gpio_file_name, sizeof(gpio_file_name), FILE_GPIO_DIRECTION, gpio_number + GPIO_CHIP);
    fd = open(gpio_file_name, O_RDWR);
    if (fd < 0) {
        printf("Open File: %s error!\n", gpio_file_name);
        return -ENOENT;
    }

    memset(write_buf, 0, sizeof(write_buf));

    if (direction == 0) {
        snprintf(write_buf, sizeof(write_buf), "%s", "in");
        write(fd, write_buf, sizeof(write_buf));
        close(fd);

        snprintf(gpio_file_name, sizeof(gpio_file_name), FILE_GPIO_VALUE, gpio_number + GPIO_CHIP);
        fd = open(gpio_file_name, O_RDONLY);
        if (fd < 0) {
            printf("Open File: %s error!\n", gpio_file_name);
            return -ENOENT;
        }

        read(fd, read_buf, sizeof(read_buf));
        close(fd);

        sscanf(read_buf, "%d", gpio_value);
        return 0;
    } else if (*gpio_value == 0) {
        snprintf(write_buf, sizeof(write_buf), "%s", "low");
    } else {
        snprintf(write_buf, sizeof(write_buf), "%s", "high");
    }

    write(fd, write_buf, sizeof(write_buf));
    close(fd);

    return 0;
}

int get_gpio_irq_event(unsigned int gpio_number)
{
    char gpio_file_name[128] = {0};
    char write_buf[5] = {0};
    int fd;
    struct pollfd pfd;

    snprintf(write_buf, sizeof(write_buf), "%d", gpio_number + GPIO_CHIP);

    fd = open(FILE_GPIO_EXPORT, O_WRONLY);
    if (fd < 0) {
        printf("Open File: %s error!\n", FILE_GPIO_EXPORT);
        return -ENOENT;
    }

    write(fd, write_buf, sizeof(write_buf));
    close(fd);

    snprintf(gpio_file_name, sizeof(gpio_file_name), FILE_GPIO_EDGE, gpio_number + GPIO_CHIP);
    fd = open(gpio_file_name, O_WRONLY);
    if (fd < 0) {
        printf("Open File: %s error!\n", gpio_file_name);
        return -ENOENT;
    }

    memset(write_buf, 0, sizeof(write_buf));
    snprintf(write_buf, sizeof(write_buf), "%s", "both");
    write(fd, write_buf, sizeof(write_buf));
    close(fd);

    snprintf(gpio_file_name, sizeof(gpio_file_name), FILE_GPIO_VALUE, gpio_number + GPIO_CHIP);
    fd = open(gpio_file_name, O_RDONLY);
    if (fd < 0) {
        printf("Open File: %s error!\n", gpio_file_name);
        return -ENOENT;
    }

    pfd.fd = fd;
    pfd.events = POLLPRI;

    while (1)
    {
        if (poll(&pfd, 1, 0) == -1) {
            perror("poll failed!\n");
            return -ENOENT;
        }

        if (pfd.revents & POLLPRI) {
            char buffer[16];
            int len;

            if (lseek(fd, 0, SEEK_SET) == -1) {
                perror("lseek failed!\n");
                return -ENOENT;
            }

            if ((len=read(fd,buffer,sizeof(buffer))) == -1) {
                perror("read failed!\n");
                return -ENOENT;
            }

            buffer[len] = 0;
            printf("%s",buffer);
        }
    }

    return 0;
}

int set_pwm(unsigned int pwm_num, unsigned int period, unsigned int duty_cycle)
{
    char pwm_file_name[128] = {0};
    char write_buf[12] = {0};
    int fd;

    fd = open(FILE_PWM_EXPORT, O_WRONLY);
    if (fd < 0) {
        printf("Open File: %s error!\n", FILE_PWM_EXPORT);
        return -ENOENT;
    }

    snprintf(write_buf, sizeof(write_buf), "%d", pwm_num);
    write(fd, write_buf, sizeof(write_buf));
    close(fd);

    snprintf(pwm_file_name, sizeof(pwm_file_name), FILE_PWM_PERIOD, pwm_num);
    fd = open(pwm_file_name, O_WRONLY);
    if (fd < 0) {
        printf("Open File: %s error!\n", pwm_file_name);
        return -ENOENT;
    }

    memset(write_buf, 0, sizeof(write_buf));
    snprintf(write_buf, sizeof(write_buf), "%u", period);
    write(fd, write_buf, sizeof(write_buf));
    close(fd);

    snprintf(pwm_file_name, sizeof(pwm_file_name), FILE_PWM_DUTY_CYCLE, pwm_num);
    fd = open(pwm_file_name, O_WRONLY);
    if (fd < 0) {
        printf("Open File: %s error!\n", pwm_file_name);
        return -ENOENT;
    }

    memset(write_buf, 0, sizeof(write_buf));
    snprintf(write_buf, sizeof(write_buf), "%u", duty_cycle);
    write(fd, write_buf, sizeof(write_buf));
    close(fd);

    snprintf(pwm_file_name, sizeof(pwm_file_name), FILE_PWM_ENABLE, pwm_num);
    fd = open(pwm_file_name, O_WRONLY);
    if (fd < 0) {
        printf("Open File: %s error!\n", pwm_file_name);
        return -ENOENT;
    }

    memset(write_buf, 0, sizeof(write_buf));
    snprintf(write_buf, sizeof(write_buf), "%d", 1);
    write(fd, write_buf, sizeof(write_buf));
    close(fd);

    /* leftover: is unexport can useful */
    return 0;
}

int get_temperature(int thremal_zone, float* temp)
{
    char temp_file_name[128] = {0};
    char read_buf[12] = {0};
    int i_temp;
    int fd;

    snprintf(temp_file_name, sizeof(temp_file_name), FILE_TEMPERATURE, thremal_zone);
    fd = open(temp_file_name, O_RDONLY);
    if (fd < 0) {
        printf("Open File: %s error!\n", temp_file_name);
        return -ENOENT;
    }

    read(fd, read_buf, sizeof(read_buf));
    close(fd);

    sscanf(read_buf, "%d", &i_temp);
    *temp = (float)i_temp / 1000;

    return 0;
}

int set_regulator(const char* reg_name, const char* reg_enable)
{
    char reg_file_name[128] = {0};
    int regulator_en;
    int fd;

    snprintf(reg_file_name, sizeof(reg_file_name), FILE_REGULATOR, reg_name);
    fd = open(reg_file_name, O_WRONLY);
    if (fd < 0) {
        printf("Open File: %s error!\n", reg_file_name);
        return -ENOENT;
    }

    write(fd, reg_enable, sizeof(reg_enable));
    close(fd);

    return 0;
}

int main(int argc, char* argv[])
{
    if (argc < 2 || strcmp(argv[1], "-help") == 0 || strcmp(argv[1], "-h") == 0) {
        printf("qrbx210_platform help:\n");
        printf("You can use like this:\n");
        printf("./qrbx210_platform -led user-led0_green 255\n");
        printf("./qrbx210_platform -button\n");

        return 0;
    }

    if (!(strcmp(argv[1], "-led"))) {
        unsigned int led_brightness;

        if (argc != 4) {
            printf("-led: parameter error!\n");
            printf("You can use like this:\n");
            printf("./qrbx210_platform -led user-led0_green 255\n");
            printf("./qrbx210_platform -temp\n");

            return -1;
        }

        if (!(sscanf(argv[3], "%u", &led_brightness))) {
            printf("-led: parameter3 error!\n");
            printf("You can use like this:\n");
            printf("./qrbx210_platform -led user-led0_green 255\n");
            printf("./qrbx210_platform -pwm 1 1000 500\n");

            return -1;
        }

        if (led_control(argv[2], led_brightness)) {
            printf("-led: parameter2 error!\n");
            printf("You can use like this:\n");
            printf("./qrbx210_platform -led user-led0_green 255\n");
            printf("./qrbx210_platform -gpio out 114 0\n");
            printf("./qrbx210_platform -irq 114\n");

            return -1;
        }
    } else if (!(strcmp(argv[1], "-button"))) {
        pthread_t t0, t1, t2, t3;
        char* par0 = "0";
        char* par1 = "1";
        char* par2 = "2";
        char* par3 = "3";
        printf("Button event catching:\n");
        pthread_create(&t0, NULL, key_event, par0);
        pthread_create(&t1, NULL, key_event, par1);
        pthread_create(&t3, NULL, key_event, par3);

        getchar();
    } else if (!(strcmp(argv[1], "-gpio"))) {
        unsigned int gpio_num;
        unsigned int gpio_val;

        if (argc < 4) {
            printf("-gpio parameters error!\n");
            return 0;
        }

        if (!(strcmp(argv[2], "out"))) {
            if (argc < 5) {
                printf("-gpio parameters error!\n");
                return 0;
            }

            if (!(sscanf(argv[3], "%u", &gpio_num))) {
                printf("-gpio: parameter3 error!\n");
                printf("You can use like this:\n");
                printf("./qrbx210_platform -gpio out 114 0\n");

                return -1;
            }

            if (!(sscanf(argv[4], "%u", &gpio_val))) {
                printf("-gpio: parameter3 error!\n");
                printf("You can use like this:\n");
                printf("./qrbx210_platform -gpio out 114 0\n");

                return -1;
            }

            if (gpio_control(gpio_num, 1, &gpio_val)) {
                printf("-gpio: parameter error!\n");
                printf("You can use like this:\n");
                printf("./qrbx210_platform -gpio out 114 1\n");

                return -1;
            }
        } else if (!(strcmp(argv[2], "in"))) {
            if (!(sscanf(argv[3], "%u", &gpio_num))) {
                printf("-gpio: parameter3 error!\n");
                printf("You can use like this:\n");
                printf("./qrbx210_platform -gpio out 114 0\n");

                return -1;
            }

            if (gpio_control(gpio_num, 0, &gpio_val)) {
                printf("-gpio: parameter error!\n");
                printf("You can use like this:\n");
                printf("./qrbx210_platform -gpio out 114 1\n");

                return -1;
            }

            printf("gpio%d input value: %d\n", gpio_num, gpio_val);
        }
    } else if (!(strcmp(argv[1], "-irq"))) {
        unsigned int gpio_num;

        if (argc < 3) {
            printf("-irq parameters error!\n");
            return -1;
        }

        if (!(sscanf(argv[2], "%u", &gpio_num))) {
            printf("-irq: parameter3 error!\n");
            printf("You can use like this:\n");
            printf("./qrbx210_platform -irq 114\n");

            return -1;
        }

        if (get_gpio_irq_event(gpio_num)) {
            printf("-gpio: parameter error!\n");
            printf("You can use like this:\n");
            printf("./qrbx210_platform -irq 114\n");

            return -1;
        }
    } else if (!(strcmp(argv[1], "-pwm"))) {
        unsigned int pwm_number;
        unsigned int pwm_period;
        unsigned int pwm_duty;

        if (argc < 5) {
            printf("-pwm parameters error!\n");
            return -1;
        }

        if (!(sscanf(argv[2], "%u", &pwm_number))) {
            printf("-pwm: parameter2 error!\n");
            printf("You can use like this:\n");
            printf("./qrbx210_platform -pwm 1 1000 500\n");

            return -1;
        }

        if (!(sscanf(argv[3], "%u", &pwm_period))) {
            printf("-pwm: parameter3 error!\n");
            printf("You can use like this:\n");
            printf("./qrbx210_platform -pwm 1 1000 500\n");

            return -1;
        }

        if (!(sscanf(argv[4], "%u", &pwm_duty))) {
            printf("-pwm: parameter4 error!\n");
            printf("You can use like this:\n");
            printf("./qrbx210_platform -pwm 1 1000 500\n");

            return -1;
        }

        if (pwm_period < pwm_duty) {
            printf("-pwm: parameter34 error!\n");
            printf("You can use like this:\n");
            printf("./qrbx210_platform -pwm 1 1000 500\n");

            return -1;
        }

        if (set_pwm(pwm_number, pwm_period, pwm_duty)) {
            printf("-pwm: parameter error!\n");
            printf("You can use like this:\n");
            printf("./qrbx210_platform -pwm 1 1000 500\n");

            return -1;
        }
    } else if (!(strcmp(argv[1], "-temp"))) {
        float soc_temp;
        unsigned int thermal_number;

        if (argc < 3) {
            printf("-temp: parameters error\n");
            return -1;
        }

        if (!(sscanf(argv[2], "%d", &thermal_number))) {
            printf("-temp: parameter2 error\n");
            printf("You can use like this:\n");
            printf("./qrbx210_platform -temp 0\n");
            return -1;
        }

        if (get_temperature(thermal_number, &soc_temp)) {
            printf("-temp: get SoC temperature error!\n");
        } else {
            printf("Get SoC thermal temperature: %.2f\n", soc_temp);
        }
    } else if (!(strcmp(argv[1], "-regulator"))) {
        if (argc < 4) {
            printf("-regulator parameters error!\n");
            return -1;
        }

        if (set_regulator(argv[2], argv[3]))
        {
            printf("-regulator: regulator set error!\n");
        }
    }

    return 0;
}
