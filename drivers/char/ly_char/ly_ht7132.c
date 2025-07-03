#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#define DEVICE_NAME "ly_ht7132"
#define BUFFER_SIZE 256

struct ly_ht7132_dev {
    struct spi_device *spi;
    struct cdev cdev;
    dev_t devno;
    struct class *class;
    
    u8 rx_buffer[BUFFER_SIZE];
    int rx_length;
    struct gpio_desc *cs_int_gpios;
    int irq;
    
    wait_queue_head_t read_wq;
    struct mutex buffer_lock;
    bool data_ready;
};

static irqreturn_t ly_ht7132_irq(int irq, void *dev_id)
{
    struct ly_ht7132_dev *ly_ht7132 = dev_id;
    struct spi_transfer t = {
        .rx_buf = ly_ht7132->rx_buffer,
        .len = BUFFER_SIZE,
    };
    struct spi_message m;
    int ret;
    
    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    
    ret = spi_sync(ly_ht7132->spi, &m);
    if (ret == 0) {
        mutex_lock(&ly_ht7132->buffer_lock);
        ly_ht7132->rx_length = t.len;
        ly_ht7132->data_ready = true;
        mutex_unlock(&ly_ht7132->buffer_lock);
        
        wake_up_interruptible(&ly_ht7132->read_wq);
        printk(KERN_DEBUG "Received %d bytes\n", t.len);
    } else {
        printk(KERN_ERR "Linyang ht7132 transfer failed: %d\n", ret);
    }
    
    return IRQ_HANDLED;
}

static int ly_ht7132_open(struct inode *inode, struct file *filp)
{
    struct ly_ht7132_dev *ly_ht7132 = container_of(inode->i_cdev,  struct ly_ht7132_dev, cdev);
    filp->private_data = ly_ht7132;
    return 0;
}

static ssize_t ly_ht7132_read(struct file *filp, char __user *buf, 
                            size_t count, loff_t *f_pos)
{
    struct ly_ht7132_dev *ly_ht7132 = filp->private_data;
    int ret;
    
    if (wait_event_interruptible(ly_ht7132->read_wq, 
                               ly_ht7132->data_ready || filp->f_flags & O_NONBLOCK)) {
        return -ERESTARTSYS;
    }
    
    mutex_lock(&ly_ht7132->buffer_lock);
    if (!ly_ht7132->data_ready) {
        mutex_unlock(&ly_ht7132->buffer_lock);
        return -EAGAIN;
    }
    
    count = min(count, (size_t)ly_ht7132->rx_length);
    if (copy_to_user(buf, ly_ht7132->rx_buffer, count)) {
        mutex_unlock(&ly_ht7132->buffer_lock);
        return -EFAULT;
    }
    
    ly_ht7132->data_ready = false;
    mutex_unlock(&ly_ht7132->buffer_lock);
    
    return count;
}

static struct file_operations ly_ht7132_fops = {
    .owner = THIS_MODULE,
    .open = ly_ht7132_open,
    .read = ly_ht7132_read,
};

static int ly_ht7132_probe(struct spi_device *spi)
{
    struct ly_ht7132_dev *ly_ht7132;
    int ret;
    
    // 分配设备结构
    ly_ht7132 = devm_kzalloc(&spi->dev, sizeof(struct ly_ht7132_dev), GFP_KERNEL);
    if (!ly_ht7132)
        return -ENOMEM;
    
    // 初始化设备
    spi->mode = SPI_MODE_1;
    spi->bits_per_word = 8;
    ret = spi_setup(spi);
    if (ret < 0) {
        dev_err(&spi->dev, "SPI setup failed\n");
        return ret;
    }
    
    ly_ht7132->spi = spi;
    spi_set_drvdata(spi, ly_ht7132);
    
    // 初始化等待队列和互斥锁
    init_waitqueue_head(&ly_ht7132->read_wq);
    mutex_init(&ly_ht7132->buffer_lock);
    
    // 注册字符设备
    ret = alloc_chrdev_region(&ly_ht7132->devno, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        dev_err(&spi->dev, "Failed to allocate device number\n");
        return ret;
    }
    
    cdev_init(&ly_ht7132->cdev, &ly_ht7132_fops);
    ly_ht7132->cdev.owner = THIS_MODULE;
    ret = cdev_add(&ly_ht7132->cdev, ly_ht7132->devno, 1);
    if (ret) {
        dev_err(&spi->dev, "Failed to add character device\n");
        goto err_cdev;
    }
    
    // 创建设备节点
    ly_ht7132->class = class_create(THIS_MODULE, DEVICE_NAME);
    if (IS_ERR(ly_ht7132->class)) {
        ret = PTR_ERR(ly_ht7132->class);
        goto err_class;
    }
    
    device_create(ly_ht7132->class, NULL, ly_ht7132->devno, NULL, DEVICE_NAME);
    
    ly_ht7132->cs_int_gpios = devm_gpiod_get(&spi->dev, "cs-int", GPIOD_IN);
	if (IS_ERR(ly_ht7132->cs_int_gpios)) {
		pr_err("ht7132: cannot get cs gpio\n");
		return PTR_ERR(ly_ht7132->cs_int_gpios);
	}

    ly_ht7132->irq = gpiod_to_irq(ly_ht7132->cs_int_gpios);
    if (ly_ht7132->irq < 0) {
        pr_err("ht7132: cannot get IRQ number\n");
        return ly_ht7132->irq;
    }

    ret = devm_request_irq(&spi->dev,
                           ly_ht7132->irq,
                           ly_ht7132_irq,
                           IRQF_TRIGGER_FALLING,
                           "ly-ht7132-cs",
                           ly_ht7132);
    if (ret) {
        pr_err("ht7132: cannot request IRQ\n");
    }

    dev_info(&spi->dev, "Linyang ht7132 device registered\n");
    return 0;
    
err_irq:
    device_destroy(ly_ht7132->class, ly_ht7132->devno);
    class_destroy(ly_ht7132->class);
err_class:
    cdev_del(&ly_ht7132->cdev);
err_cdev:
    unregister_chrdev_region(ly_ht7132->devno, 1);
    return ret;
}

static int ly_ht7132_remove(struct spi_device *spi)
{
    struct ly_ht7132_dev *ly_ht7132 = spi_get_drvdata(spi);
    
    if (spi->irq > 0)
        free_irq(spi->irq, ly_ht7132);
    
    device_destroy(ly_ht7132->class, ly_ht7132->devno);
    class_destroy(ly_ht7132->class);
    cdev_del(&ly_ht7132->cdev);
    unregister_chrdev_region(ly_ht7132->devno, 1);
    
    dev_info(&spi->dev, "Linyang ht7132 device removed\n");
    return 0;
}

static const struct of_device_id ly_ht7132_of_match[] = {
    { .compatible = "ly,ht7132", .data = 0 },
    {},
};
MODULE_DEVICE_TABLE(of, ly_ht7132_of_match);

static struct spi_driver ly_ht7132_driver = {
    .driver = {
        .name = "ly-ht7132",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ly_ht7132_of_match),
    },
    .probe = ly_ht7132_probe,
    .remove = ly_ht7132_remove,
};

module_spi_driver(ly_ht7132_driver);

MODULE_DESCRIPTION("Driver for Linyang Ht7132");
MODULE_AUTHOR("zhangyunduan@linyang.com.cn");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");