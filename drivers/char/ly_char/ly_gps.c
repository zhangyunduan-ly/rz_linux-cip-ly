/*
 * ly_gps.c -- support gps
 *
 *  Author			zhangyunduan
 *  Email   		zhangyunduan@linyang.com.cn
 *  Create time 	2025-08-04
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
#include <linux/io.h>
#include <linux/err.h>
#include <linux/device.h>

#define DEVICE_NAME "ly_gps"

struct ly_gps_dev {
    struct gpio_desc *pps_gpios;                    // 掉电检测
    unsigned int irq;                               // 掉电检测中断
    wait_queue_head_t wait_q;                       // 定义等待队列头部
    int pps_flag;                                   // 等待条件
};

static struct ly_gps_dev *ly_gps;

static int gps_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int gps_close(struct inode *inode, struct file *file)
{
    return 0;
}

static ssize_t gps_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    unsigned char uc[1];

    ly_gps->pps_flag = 0;
    if (wait_event_interruptible(ly_gps->wait_q, ly_gps->pps_flag != 0)) {
        return -ERESTARTSYS;
    }
    uc[1] = 1;
    ly_gps->pps_flag = 0;

    if (copy_to_user(buf, uc, 1)) {
        return -EFAULT;
    }

    return 1;
}

static struct file_operations gps_fops = {
    .owner = THIS_MODULE,
    .open = gps_open,
    .release = gps_close,
    .read = gps_read,
};

static struct miscdevice miscgps = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = DEVICE_NAME,
    .fops = &gps_fops,
};

static irqreturn_t pps_interrupt(int irq, void *dev_id)
{
    ly_gps->pps_flag = 1;
    wake_up_interruptible(&ly_gps->wait_q); // 中断唤醒

    return IRQ_RETVAL(IRQ_HANDLED);
}

static int gps_probe(struct platform_device *pdev)
{
    int ret = -1;

    ly_gps = devm_kzalloc(&pdev->dev, sizeof(struct ly_gps_dev), GFP_KERNEL);
    if (NULL == ly_gps) {
        pr_err("gps: no memory to zalloc\n");
        ret = -ENOMEM;
    }

    /* 1pps gpio */
	ly_gps->pps_gpios = devm_gpiod_get(&pdev->dev, "pps", GPIOD_IN);
	if (IS_ERR(ly_gps->pps_gpios)) {
		pr_err("gps: cannot get 1pps gpio\n");
		return PTR_ERR(ly_gps->pps_gpios);
	}

    ly_gps->irq = gpiod_to_irq(ly_gps->pps_gpios);
    if (ly_gps->irq < 0) {
        pr_err("gps: cannot get IRQ number\n");
        return ly_gps->irq;
    }
    
    init_waitqueue_head(&ly_gps->wait_q);

    ret = devm_request_irq(&pdev->dev,
                           ly_gps->irq,
                           pps_interrupt,
                           IRQF_TRIGGER_RISING,
                           "1pps",
                           ly_gps);
    if (ret) {
        pr_err("gps: cannot request IRQ\n");
    }

    ret = misc_register(&miscgps);
    if (ret < 0) {
        pr_err("gps: misc register error\n");
        return ret;
    }
    pr_info("gps: misc register successed: \n");

    return ret;
}

static void gps_remove(struct platform_device *pdev)
{
    misc_deregister(&miscgps);
}

static const struct of_device_id ly_gps_dt_ids[] = {
    {.compatible = "ly-gps"},
    {/* sentinel */},
};

MODULE_DEVICE_TABLE(of, ly_gps_dt_ids);

static struct platform_driver ly_gps_driver = {
    .probe = gps_probe,
    .remove = gps_remove,
    .driver = {
        .name = "ly_gps_gps",
        .of_match_table = of_match_ptr(ly_gps_dt_ids),
    },
};

static int __init gps_init(void)
{
    return platform_driver_register(&ly_gps_driver);
}

static void __exit gps_exit(void)
{
    platform_driver_unregister(&ly_gps_driver);
}

module_init(gps_init);

module_exit(gps_exit);

MODULE_DESCRIPTION("Driver for gps");
MODULE_AUTHOR("zhangyunduan@linyang.com.cn");
MODULE_LICENSE("GPL");
MODULE_ALIAS("gpio:gps");
