/* Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-only
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#define LED_FILE "/sys/class/leds/green/breath"

int LED_Ctrol(char *buff)
{
    struct file *fd = NULL;
    mm_segment_t fs;
    loff_t pos;

    fd = filp_open(LED_FILE, O_RDWR, 0);
    if (IS_ERR(fd)) {
	printk(KERN_ERR "open failed!\n");
	filp_close(fd, NULL);
        return -1;
    }

    fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;
    vfs_write(fd, buff, sizeof(buff), &pos);
    pos = 0;

    filp_close(fd, NULL);
    set_fs(fs);

    return 0;
}

static int __init led_init(void)
{
    char buff_on[] = {"128"};
    LED_Ctrol(buff_on);
    printk(KERN_ALERT "LED ON\n");
    return 0;
}

static void __exit led_exit(void)
{
    char buff_off[] = {"0"};
    LED_Ctrol(buff_off);
    printk(KERN_ALERT "LED OFF\n");
}

module_init(led_init);
module_exit(led_exit);
MODULE_LICENSE("GPL v2");
