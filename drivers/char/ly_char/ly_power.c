/*
 * ly_power.c -- support power
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
#include <linux/io.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h>
#include <asm/io.h>
#include <linux/time.h>

#include <linux/syscalls.h>
#include <linux/reboot.h>

#define DEVICE_NAME "ly_power"

#define STATE_HIGH 1
#define STATE_LOW  0

#define IO_MUX_NUM 64

#define DELAY_CNT  6000 // 消抖时间
#define DELAY_CNT1 1000 // 短消抖时间

/* 定义魔数 */
#define POWER_MAGIC 'P'

/* 定义命令 */
#define POWER_CHARGE_BATTERY   _IOW(POWER_MAGIC, 1, int) // 电池充电控制
#define POWER_CHARGE_CAPACITOR _IOW(POWER_MAGIC, 2, int) // 超级电容充电控制
#define POWER_SYSTEM_REBOOT    _IOW(POWER_MAGIC, 3, int) // 系统重启
#define POWER_SYSTEM_HALT      _IOW(POWER_MAGIC, 4, int) // 系统停止

struct ly_power_dev {
    struct gpio_desc *battery_charge_gpios;         // 备用电池充电控制
    struct gpio_desc *battery_discharge_gpios;      // 备用电池放电控制
    struct gpio_desc *capacitor_charge_gpios;       // 超级电容充电控制
    struct gpio_desc *capacitor_discharge_gpios;    // 超级电容放电控制
    struct gpio_desc *pfi_gpios;                    // 掉电检测
    unsigned int irq;                               // 掉电检测中断
    wait_queue_head_t wait_q;                       // 定义等待队列头部
    int poweroff_flag;                              // 等待条件
    int rtnflag;                                    // 用于判断是否read是否直接返回
    int irqflag;                                    // 等待队列唤醒后重新等待初始化等待队列标识
    int wait_i;                                     // 已进入等待标识
};

static struct ly_power_dev *ly_power;

static int power_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int power_close(struct inode *inode, struct file *file)
{
    return 0;
}

static ssize_t power_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    int i, cnt;
    unsigned char uc[1];

    // 防止已经在等待时重新read
    if (ly_power->wait_i) {
        ly_power->irqflag = 1;
        ly_power->rtnflag = 0;
        ly_power->wait_i = 0;
        ly_power->poweroff_flag = 1;
        wake_up_interruptible(&ly_power->wait_q);
        disable_irq_nosync(ly_power->irq);
    }

    cnt = 0;
    for (i = 0; i < DELAY_CNT; i++) {
        if (gpiod_get_value(ly_power->pfi_gpios) == 1) {
            cnt++;
        }
        udelay(1);
    }

    if (cnt > (DELAY_CNT * 9 / 10)) {
        gpiod_set_value(ly_power->capacitor_discharge_gpios, 0);

        uc[0] = 0x01;
        if (ly_power->rtnflag) {
            if (ly_power->irqflag) {
                enable_irq(ly_power->irq);
                init_waitqueue_head(&ly_power->wait_q);
            }
            ly_power->wait_i = 1;
            ly_power->poweroff_flag = 0;
            if (wait_event_interruptible(ly_power->wait_q, ly_power->poweroff_flag != 0)) {
                return -ERESTARTSYS;
            }

            uc[0] = 0x00;
            ly_power->poweroff_flag = 0;
            ly_power->rtnflag = 0;
            ly_power->wait_i = 0;
        } else {
            ly_power->rtnflag = 1;
        }
    } else {
        uc[0] = 0x00;
        ly_power->rtnflag = 0;
        ly_power->wait_i = 0;
    }

    if (copy_to_user(buf, uc, 1)) {
        return -EFAULT;
    }

    return 1;
}

static long power_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    unsigned int data;

    switch (cmd) {

    case POWER_CHARGE_BATTERY:
        break;

    case POWER_CHARGE_CAPACITOR:
        break;

    case POWER_SYSTEM_REBOOT:
        kernel_restart(NULL);
        break;

    case POWER_SYSTEM_HALT:
        kernel_halt();
        break;
    }

    return ret;
}

static struct file_operations power_fops = {
    .owner = THIS_MODULE,
    .open = power_open,
    .release = power_close,
    .read = power_read,
    .unlocked_ioctl = power_ioctl,
};

static struct miscdevice miscpower = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = DEVICE_NAME,
    .fops = &power_fops,
};

static irqreturn_t poweroff_interrupt(int irq, void *dev_id)
{
    int i = 0, cnt = 0;

    pr_info("poweroff interrupt\n");

    // 短消抖去毛刺
    for (i = 0; i < DELAY_CNT1; i++) {
        udelay(1);
        if (gpiod_get_value(ly_power->pfi_gpios) == 0) {
            cnt++;
        }
    }

    if (cnt > (DELAY_CNT1 * 9 / 10)) {
        // 先打开超级电容和电池
        gpiod_set_value(ly_power->capacitor_discharge_gpios, 1);

        // 长消抖确认是否真的发生掉电
        for (i = DELAY_CNT1; i < DELAY_CNT; i++) {
            udelay(1);
            if (gpiod_get_value(ly_power->pfi_gpios) == 0) {
                cnt++;
            }
        }

        if (cnt > (DELAY_CNT * 9 / 10)) {
            pr_info("poweroff interrupt actual\n");
            ly_power->poweroff_flag = 1;
            wake_up_interruptible(&ly_power->wait_q); // 中断唤醒
            disable_irq_nosync(irq);
            ly_power->irqflag = 1;
        } else {
            gpiod_set_value(ly_power->capacitor_discharge_gpios, 0);
        }
    }

    return IRQ_RETVAL(IRQ_HANDLED);
}

static int power_probe(struct platform_device *pdev)
{
    int ret = -1;

    ly_power = devm_kzalloc(&pdev->dev, sizeof(struct ly_power_dev), GFP_KERNEL);
    if (NULL == ly_power) {
        pr_err("power: no memory to zalloc\n");
        ret = -ENOMEM;
    }

    /* capacitor discharge gpio */
	ly_power->capacitor_discharge_gpios = devm_gpiod_get(&pdev->dev, "capacitor-discharge", GPIOD_OUT_LOW);
	if (IS_ERR(ly_power->capacitor_discharge_gpios)) {
		pr_err("power: cannot get capacitor charge gpio\n");
		return PTR_ERR(ly_power->capacitor_discharge_gpios);
	}

    /* pfi gpio */
	ly_power->pfi_gpios = devm_gpiod_get(&pdev->dev, "pfi", GPIOD_IN);
	if (IS_ERR(ly_power->pfi_gpios)) {
		pr_err("power: cannot get pfi gpio\n");
		return PTR_ERR(ly_power->pfi_gpios);
	}

    ly_power->irq = gpiod_to_irq(ly_power->pfi_gpios);
    if (ly_power->irq < 0) {
        pr_err("power: cannot get IRQ number\n");
        return ly_power->irq;
    }

    ret = devm_request_irq(&pdev->dev,
                           ly_power->irq,
                           poweroff_interrupt,
                           IRQF_TRIGGER_FALLING,
                           "poweroff",
                           ly_power);
    if (ret) {
        pr_err("power: cannot request IRQ\n");
    }

    init_waitqueue_head(&ly_power->wait_q);

    ret = misc_register(&miscpower);
    if (ret < 0) {
        pr_err("power: misc register error\n");
        return ret;
    }
    pr_info("power: misc register successed: \n");

    return ret;
}

static void power_remove(struct platform_device *pdev)
{
    misc_deregister(&miscpower);
}

static const struct of_device_id ly_power_dt_ids[] = {
    {.compatible = "ly-power"},
    {/* sentinel */},
};

MODULE_DEVICE_TABLE(of, ly_power_dt_ids);

static struct platform_driver ly_power_driver = {
    .probe = power_probe,
    .remove = power_remove,
    .driver = {
        .name = "ly_power_power",
        .of_match_table = of_match_ptr(ly_power_dt_ids),
    },
};

static int __init power_init(void)
{
    return platform_driver_register(&ly_power_driver);
}

static void __exit power_exit(void)
{
    platform_driver_unregister(&ly_power_driver);
}

module_init(power_init);

module_exit(power_exit);

MODULE_DESCRIPTION("Driver for Power");
MODULE_AUTHOR("zhangyunduan@linyang.com.cn");
MODULE_LICENSE("GPL");
MODULE_ALIAS("gpio:power");
