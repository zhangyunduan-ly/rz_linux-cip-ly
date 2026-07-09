#include <linux/init.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/watchdog.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/gpio.h>

#define WDI_PIN 648 // (17*8+0)+512

static struct timer_list watchdog_timer;
static int timer_index = 0;

static unsigned int early_watchdog_status = 1;
static unsigned int early_watchdog_timeout = 360;

static void poll_timer_func(struct timer_list *t)
{
    // pr_info("Feeding the watchdog...\n");

    if (++timer_index >= early_watchdog_timeout) {
        /* stop dog */
        pr_info("early watchdog: kernel stop dog\n");
        if (early_watchdog_status) {
            early_watchdog_status = 0;
        }
        return;
    }

    /* feed dog */
    if (timer_index % 2) {
        gpio_set_value(WDI_PIN, 1); //high
    } else {
        gpio_set_value(WDI_PIN, 0); //low
    }

    /* restart watchdog timer */
    mod_timer(&watchdog_timer, jiffies + msecs_to_jiffies(500));
}

static ssize_t enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", early_watchdog_status);
}

static ssize_t enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret; 
    unsigned int status;

    ret = sscanf(buf, "%u", &status);
    if (ret != 1)
        return -EINVAL;

    if (!status && early_watchdog_status) {
        early_watchdog_status = 0; 
        gpio_set_value(WDI_PIN, 1); //high
        del_timer(&watchdog_timer);
    }    
    else if (status && !early_watchdog_status) {
        early_watchdog_status = status;
        timer_index = 0; 
        gpio_set_value(WDI_PIN, 0); //low
        mod_timer(&watchdog_timer, jiffies + msecs_to_jiffies(500));
    }    

    return count;
}

static ssize_t timeout_show(struct device *dev,
                            struct device_attribute *attr,
                            char *buf)
{
    return sprintf(buf, "%d\n", early_watchdog_timeout/2);
}

static ssize_t timeout_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret;

    ret = sscanf(buf, "%u", &early_watchdog_timeout);
    if (ret != 1)
        return -EINVAL;

    early_watchdog_timeout <<= 1;

    return count;
}

static DEVICE_ATTR_RW(enable);
static DEVICE_ATTR_RW(timeout);

static struct attribute *early_watchdog_attrs[] = {
    &dev_attr_enable.attr,
    &dev_attr_timeout.attr,
    NULL,
};
ATTRIBUTE_GROUPS(early_watchdog);

static struct class early_watchdog_class = {
    .name = "early_watchdog",
    .class_groups = early_watchdog_groups,
};

static int __init init_early_watchdog_timer(void)
{
    int ret;

    pr_info("Initializing early watchdog timer...\n");

    gpio_direction_output(WDI_PIN, 1);

    timer_setup(&watchdog_timer, poll_timer_func, 0);

    mod_timer(&watchdog_timer, jiffies + msecs_to_jiffies(500));

    ret = class_register(&early_watchdog_class);
    if (ret != 0) {
        pr_err("early watchdog: class register error!\n");
    } else {
        pr_info("early watchdog: class register success.\n");
    }

    return 0;
}

early_initcall(init_early_watchdog_timer);
// module_init(init_early_watchdog_timer);

static void __exit cleanup_early_watchdog_timer(void)
{
    pr_info("early watchdog: cleaning up watchdog timer...\n");
    del_timer(&watchdog_timer);
    class_destroy(&early_watchdog_class);
}

module_exit(cleanup_early_watchdog_timer);
