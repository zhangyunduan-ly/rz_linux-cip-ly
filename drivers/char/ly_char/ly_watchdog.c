/*
 * ly_watchdog.c -- support gpio watchdog
 *
 *  Author			zhangyunduan
 *  Email   		zhangyunduan@linyang.com.cn
 *  Create time 	2024-10-15
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/unistd.h>

#include <linux/version.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/printk.h>
#include <linux/device.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h>
#include <asm/io.h>
#include <linux/timer.h>
#include <linux/syscalls.h>

#define DEVICE_NAME "ly_watchdog"
// #define WATCHDOG_DEBUG

#define STATE_HIGH 1
#define STATE_LOW  0

#define DEFAULT_TIMER_EXPIRES       500
#define DEFAULT_WATCHDOG_TIMEOUT    20000

struct ly_watchdog_dev {
    char state;      // state: current feeddog gpio state
    struct gpio_desc *feed_gpios;
    int is_open;
    int is_enable;
    struct timer_list timer;
    unsigned long timer_expires;
    unsigned long timeout;
    struct tasklet_struct tasklet;
};

static struct ly_watchdog_dev *ly_watchdog;
static unsigned long dog_keepalive = 0;

static void tasklet_hander_function(unsigned long data)
{
    pr_info("watchdog ksys_sync\n");
    udelay(data);
}

static inline void poll_timer_func(struct timer_list *ptimer)
{
    if (dog_keepalive < ly_watchdog->timeout) {
        dog_keepalive += ly_watchdog->timer_expires;
    }

    if (dog_keepalive >= ly_watchdog->timeout) {
        pr_err("ERROR: wathdog timeout!\n");
        tasklet_init(&ly_watchdog->tasklet, tasklet_hander_function, ly_watchdog->tasklet.data);
        ly_watchdog->tasklet.data = 10;
        return;
    }

    if (ly_watchdog->is_enable == 0) {
        return;
    }

    mod_timer(ptimer, jiffies + msecs_to_jiffies(ly_watchdog->timer_expires));

    /* toogle the feed dog pin */
    ly_watchdog->state = (ly_watchdog->state == STATE_HIGH) ? STATE_LOW : STATE_HIGH;
    gpiod_set_value(ly_watchdog->feed_gpios, ly_watchdog->state);
}

static inline int watchdog_keepalive(void)
{
    // pr_debug("watchdog: keep alive\n");

    dog_keepalive = 0;

    return 0;
}

static inline int watchdog_settimout(int new_settimout)
{
    pr_info("watchdog: set timeout: %d\n", new_settimout);

    ly_watchdog->timeout = new_settimout;
    mod_timer(&ly_watchdog->timer, jiffies + msecs_to_jiffies(ly_watchdog->timer_expires));

    return 0;
}

static inline int watchdog_enable(void)
{
    pr_info("watchdog: enable\n");

    ly_watchdog->is_enable = 1;
    watchdog_keepalive();
    mod_timer(&ly_watchdog->timer, jiffies + msecs_to_jiffies(ly_watchdog->timer_expires));

    return 0;
}

static inline int watchdog_disable(void)
{
    pr_info("watchdog: disable\n");

    ly_watchdog->is_enable = 0;

    return 0;
}

static int watchdog_open(struct inode *inode, struct file *file)
{
    if (1 == ly_watchdog->is_open) {
        return 0;
    }

    ly_watchdog->is_open = 1;

    /* timer wathdog */
    timer_setup(&ly_watchdog->timer, poll_timer_func, 0);

    return 0;
}

static int watchdog_close(struct inode *inode, struct file *file)
{
    return 0;
}

static long watchdog_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int val;
    int ret = -1;

    switch (cmd) {
    case WDIOC_KEEPALIVE:
        ret = watchdog_keepalive();
        break;
    case WDIOC_SETTIMEOUT:
        if (copy_from_user(&val, (void __user *)arg, sizeof(int)))
            return -EFAULT;
        ret = watchdog_settimout(val);
        break;
    case WDIOC_SETOPTIONS:
        if (get_user(val, (int __user *)arg)) {
            ret = -EFAULT;
            break;
        }

        if (val & WDIOS_ENABLECARD)
            ret = watchdog_enable();
        else if (val & WDIOS_DISABLECARD)
            ret = watchdog_disable();
        else
            ret = -EINVAL;
        break;
    }

    return ret;
}

static struct file_operations watchdog_fops = {
    .owner = THIS_MODULE,
    .open = watchdog_open,
    .release = watchdog_close,
    .unlocked_ioctl = watchdog_ioctl,
};

static struct miscdevice miscwatchdog = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = DEVICE_NAME,
    .fops = &watchdog_fops,
};

static int watchdog_probe(struct platform_device *pdev)
{
    int ret = -1;

    ly_watchdog = devm_kzalloc(&pdev->dev, sizeof(struct ly_watchdog_dev), GFP_KERNEL);
    if (NULL == ly_watchdog) {
        pr_err("watchdog: no memory to zalloc\n");
        ret = -ENOMEM;
    }

    /* feed gpio */
    ly_watchdog->feed_gpios = devm_gpiod_get(&pdev->dev, "feed", GPIOD_OUT_LOW);
    if (IS_ERR(ly_watchdog->feed_gpios)) {
        pr_err("watchdog: cannot get feed gpio\n");
        return PTR_ERR(ly_watchdog->feed_gpios);
    }

    ly_watchdog->is_open = 0;
    ly_watchdog->is_enable = 1;
    ly_watchdog->timer_expires = DEFAULT_TIMER_EXPIRES;
    ly_watchdog->timeout = DEFAULT_WATCHDOG_TIMEOUT;

    ret = misc_register(&miscwatchdog);
    if (ret < 0) {
        pr_err("watchdog: misc register error\n");
    }

    pr_info("watchdog: misc register successed: \n");

    return ret;
}

static int watchdog_remove(struct platform_device *pdev)
{
    del_timer(&ly_watchdog->timer);
    misc_deregister(&miscwatchdog);

    return 0;
}

static const struct of_device_id ly_wdt_dt_ids[] = {
    {.compatible = "ly-watchdog"},
    {/* sentinel */},
};

MODULE_DEVICE_TABLE(of, ly_wdt_dt_ids);

static struct platform_driver ly_wdt_driver = {
    .probe = watchdog_probe,
    .remove = watchdog_remove,
    .driver = {
        .name = "ly_wdt",
        .of_match_table = of_match_ptr(ly_wdt_dt_ids),
    },
};

static int __init watchdog_init(void)
{
    return platform_driver_register(&ly_wdt_driver);
}

static void __exit watchdog_exit(void)
{
    platform_driver_unregister(&ly_wdt_driver);
}

module_init(watchdog_init);

module_exit(watchdog_exit);

MODULE_DESCRIPTION("Driver for Watchdog");
MODULE_AUTHOR("zhangyunduan@linyang.com.cn");
MODULE_LICENSE("GPL");
MODULE_ALIAS("gpio:watchdog");
