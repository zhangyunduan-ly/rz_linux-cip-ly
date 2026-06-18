#include <linux/module.h>
#include <linux/init.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/kfifo.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>

#define DRIVER_NAME "gpio_uart"
#define DEVICE_NAME "ttyGPIO"
#define GPIO_UART_NR 1
#define FIFO_SIZE 256
#define PORT_GPIO_UART 123

/* UART configuration */
struct gpio_uart_config {
    unsigned int baud;
    unsigned int databits;
    unsigned int stopbits;
    unsigned int parity;
};

struct gpio_uart_port {
    struct uart_port port;
    struct gpio_desc *gpiod_tx;
    struct gpio_desc *gpiod_rx;
    struct device *dev;
    struct gpio_uart_config config;
    bool tx_running;
    bool rx_running;
    struct hrtimer tx_timer;
    struct hrtimer rx_timer;
    ktime_t bit_period;
    unsigned char tx_char;
    int tx_bit;
    int rx_bit;
    unsigned char rx_char;
    int tx_stop_bits_remaining;
    int rx_irq;
};

/* Helper function to calculate bit period from baud rate */
static void gpio_uart_set_bit_period(struct gpio_uart_port *up, unsigned int baud)
{
    if (baud > 0) {
        up->bit_period = ktime_set(0, NSEC_PER_SEC / baud);
    } else {
        up->bit_period = ktime_set(0, NSEC_PER_SEC / 9600); /* Default 9600 */
    }
}

/* Calculate parity bit */
static int gpio_uart_parity_bit(unsigned char data, unsigned int parity)
{
    int parity_bit = 0;
    int i;

    if (parity == 0) {
        return 0; /* No parity */
    }

    /* Calculate parity */
    for (i = 0; i < 8; i++) {
        parity_bit ^= ((data >> i) & 1);
    }

    if (parity == 2) { /* Even parity */
        return parity_bit;
    } else { /* Odd parity */
        return !parity_bit;
    }
}

/* UART operations implementation */
static unsigned int gpio_uart_tx_empty(struct uart_port *port)
{
    struct gpio_uart_port *up = container_of(port, struct gpio_uart_port, port);
    unsigned int len;
    unsigned long flags;

    if (up->tx_running)
        return 0;

    uart_port_lock_irqsave(port, &flags);
    len = kfifo_len(&port->state->port.xmit_fifo);
    uart_port_unlock_irqrestore(port, flags);

    return len == 0 ? TIOCSER_TEMT : 0;
}

static void gpio_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
    /* Modem control not supported for GPIO UART */
}

static unsigned int gpio_uart_get_mctrl(struct uart_port *port)
{
    /* No modem control signals */
    return 0;
}

static void gpio_uart_start_tx(struct uart_port *port)
{
    struct gpio_uart_port *up = container_of(port, struct gpio_uart_port, port);

    if (!up->tx_running) {
        up->tx_running = true;
        up->tx_bit = -1;
        hrtimer_start(&up->tx_timer, up->bit_period, HRTIMER_MODE_REL);
    }
}

static void gpio_uart_stop_tx(struct uart_port *port)
{
    struct gpio_uart_port *up = container_of(port, struct gpio_uart_port, port);

    up->tx_running = false;
    hrtimer_cancel(&up->tx_timer);
}

static void gpio_uart_stop_rx(struct uart_port *port)
{
    struct gpio_uart_port *up = container_of(port, struct gpio_uart_port, port);

    up->rx_running = false;
    hrtimer_cancel(&up->rx_timer);
}

static void gpio_uart_enable_ms(struct uart_port *port)
{
    /* Modem status not supported */
}

static void gpio_uart_break_ctl(struct uart_port *port, int break_state)
{
    struct gpio_uart_port *up = container_of(port, struct gpio_uart_port, port);

    if (break_state) {
        gpiod_set_value(up->gpiod_tx, 0); /* Send break (space) */
    } else {
        gpiod_set_value(up->gpiod_tx, 1); /* Release break (mark) */
    }
}

/* TX timer callback - handles bit-banged transmission */
static enum hrtimer_restart gpio_uart_tx_timer_callback(struct hrtimer *timer)
{
    struct gpio_uart_port *up = container_of(timer, struct gpio_uart_port, tx_timer);
    struct uart_port *port = &up->port;
    unsigned long flags;
    unsigned int parity_bit;

    if (!up->tx_running) {
        return HRTIMER_NORESTART;
    }

    if (up->tx_bit == -1) {
        /* Get next character from core xmit_fifo */
        uart_port_lock_irqsave(port, &flags);
        if (kfifo_get(&port->state->port.xmit_fifo, &up->tx_char) == 0) {
            /* FIFO empty, stop transmission */
            up->tx_running = false;
            uart_port_unlock_irqrestore(port, flags);
            uart_write_wakeup(port);
            return HRTIMER_NORESTART;
        }
        uart_port_unlock_irqrestore(port, flags);

        /* Send start bit (0) */
        gpiod_set_value(up->gpiod_tx, 0);
        up->tx_bit = 0;
        up->tx_stop_bits_remaining = 0;
    } else if (up->tx_bit < up->config.databits) {
        /* Send data bit (LSB first) */
        gpiod_set_value(up->gpiod_tx, (up->tx_char >> up->tx_bit) & 1);
        up->tx_bit++;
    } else if ((up->tx_bit == up->config.databits) && (up->config.parity)) {
        /* Send parity bit */
        parity_bit = gpio_uart_parity_bit(up->tx_char, up->config.parity);
        gpiod_set_value(up->gpiod_tx, parity_bit);
        up->tx_bit++;
    } else if (up->tx_stop_bits_remaining < up->config.stopbits) {
        /* Send stop bit(s) */
        gpiod_set_value(up->gpiod_tx, 1); /* Stop bit is always 1 (mark) */
        up->tx_stop_bits_remaining++;
        if (up->tx_stop_bits_remaining >= up->config.stopbits) {
            up->tx_bit = -1;
        }
    }

    /* Schedule next bit transmission */
    if (up->tx_running) {
        hrtimer_forward(timer, hrtimer_get_expires(timer), up->bit_period);
        return HRTIMER_RESTART;
    }

    return HRTIMER_NORESTART;
}

/* GPIO interrupt handler - captures start bit precisely */
static irqreturn_t gpio_uart_rx_irq_handler(int irq, void *dev_id)
{
    struct gpio_uart_port *up = dev_id;
    int bit_value;

    if (!up->rx_running)
        return IRQ_HANDLED;

    bit_value = gpiod_get_value(up->gpiod_rx);
    if (bit_value != 0)
        return IRQ_HANDLED;

    disable_irq_nosync(irq);
    up->rx_bit = 0;
    up->rx_char = 0;

    hrtimer_start(&up->rx_timer, up->bit_period, HRTIMER_MODE_REL);

    return IRQ_HANDLED;
}

/* RX timer callback - handles bit-banged reception */
static enum hrtimer_restart gpio_uart_rx_timer_callback(struct hrtimer *timer)
{
    struct gpio_uart_port *up = container_of(timer, struct gpio_uart_port, rx_timer);
    int bit_value;
    unsigned char received_char;
    int parity_bit;
    unsigned char tty_flag = TTY_NORMAL;

    if (!up->rx_running || !up->gpiod_rx) {
        return HRTIMER_NORESTART;
    }

    bit_value = gpiod_get_value(up->gpiod_rx);

    if (up->rx_bit < up->config.databits) {
        /* Receive data bit (LSB first) */
        if (bit_value) {
            up->rx_char |= (1 << up->rx_bit);
        }
        up->rx_bit++;
    } else if ((up->rx_bit == up->config.databits) && (up->config.parity)) {
        /* Receive parity bit */
        parity_bit = gpio_uart_parity_bit(up->rx_char, up->config.parity);
        if (parity_bit != bit_value) {
            tty_flag = TTY_PARITY;
        }
        up->rx_bit++;
    } else if (up->rx_bit < up->config.databits + (up->config.parity ? 1 : 0) + up->config.stopbits) {
        /* Receive stop bit */
        if (bit_value != 1) {
            tty_flag = TTY_FRAME;
        }
        up->rx_bit++;
        /* If all stop bits received, push character to TTY layer */
        if (up->rx_bit >= up->config.databits + (up->config.parity ? 1 : 0) + up->config.stopbits) {
            received_char = up->rx_char;
            tty_insert_flip_char(&up->port.state->port, received_char, tty_flag);
            tty_flip_buffer_push(&up->port.state->port);
            up->rx_bit = 0;
            if (up->rx_irq) {
                enable_irq(up->rx_irq);
            }
            return HRTIMER_NORESTART;
        }
    }

    /* Schedule next bit reception */
    if (up->rx_running) {
        hrtimer_forward(timer, hrtimer_get_expires(timer), up->bit_period);
        return HRTIMER_RESTART;
    }

    return HRTIMER_NORESTART;
}

static int gpio_uart_startup(struct uart_port *port)
{
    struct gpio_uart_port *up = container_of(port, struct gpio_uart_port, port);
	int ret;

    /* Initialize timers */
    hrtimer_init(&up->tx_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    up->tx_timer.function = gpio_uart_tx_timer_callback;

    if (up->gpiod_rx) {
        hrtimer_init(&up->rx_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        up->rx_timer.function = gpio_uart_rx_timer_callback;
    }

    up->tx_running = false;
    up->rx_running = false;
    up->tx_bit = -1;
    up->rx_bit = -1;

    /* Set TX line to idle state (mark) */
    gpiod_set_value(up->gpiod_tx, 1);

    /* Start RX if available */
    if (up->gpiod_rx) {
        up->rx_running = true;
        ret = request_irq(up->rx_irq, gpio_uart_rx_irq_handler,
                          IRQF_TRIGGER_FALLING, DRIVER_NAME, up);
        if (ret) {
            dev_err(up->dev, "Failed to request IRQ %d (%d), fallback to polling\n",
                    up->rx_irq, ret);
            up->rx_irq = 0;
        }
    }

    return 0;
}

static void gpio_uart_shutdown(struct uart_port *port)
{
    struct gpio_uart_port *up = container_of(port, struct gpio_uart_port, port);

    gpio_uart_stop_tx(port);
    gpio_uart_stop_rx(port);

    if (up->rx_irq) {
        free_irq(up->rx_irq, up);
    }

    /* Set TX line to idle state (mark) */
    gpiod_set_value(up->gpiod_tx, 1);
}

static void gpio_uart_flush_buffer(struct uart_port *port)
{
    struct gpio_uart_port *up = container_of(port, struct gpio_uart_port, port);

    up->tx_running = false;
    hrtimer_cancel(&up->tx_timer);
}

static void gpio_uart_set_termios(struct uart_port *port, struct ktermios *termios, const struct ktermios *old)
{
    struct gpio_uart_port *up = container_of(port, struct gpio_uart_port, port);
    unsigned int baud, databits, stopbits, parity;

    /* Get baud rate */
    baud = tty_termios_baud_rate(termios);
    if (baud == 0) {
        baud = 9600; /* Default */
    }
    tty_termios_encode_baud_rate(termios, baud, baud);

    /* Get data bits */
    switch (termios->c_cflag & CSIZE) {
        case CS5:
            databits = 5;
            break;
        case CS6:
            databits = 6;
            break;
        case CS7:
            databits = 7;
            break;
        case CS8:
        default:
            databits = 8;
            break;
    }

    /* Get stop bits */
    stopbits = (termios->c_cflag & CSTOPB) ? 2 : 1;

    /* Get parity */
    if (termios->c_cflag & PARENB) {
        parity = (termios->c_cflag & PARODD) ? 1 : 2; /* 1=odd, 2=even */
    } else {
        parity = 0; /* no parity */
    }

    /* Update configuration */
    up->config.baud = baud;
    up->config.databits = databits;
    up->config.stopbits = stopbits;
    up->config.parity = parity;

    /* Update bit period */
    gpio_uart_set_bit_period(up, baud);

    /* Apply hardware settings */
    if (termios->c_cflag & CRTSCTS) {
        /* Hardware flow control not supported */
        termios->c_cflag &= ~CRTSCTS;
    }

    if (termios->c_cflag & CLOCAL) {
        /* Ignore modem control lines */
        port->flags &= ~UPF_HARDPPS_CD;
    } else {
        port->flags |= UPF_HARDPPS_CD;
    }
}

static const char *gpio_uart_type(struct uart_port *port)
{
    return "GPIO UART";
}

static void gpio_uart_release_port(struct uart_port *port)
{
    /* No resources to release */
}

static int gpio_uart_request_port(struct uart_port *port)
{
    return 0;
}

static void gpio_uart_config_port(struct uart_port *port, int flags)
{
    if (flags & UART_CONFIG_TYPE) {
        port->type = PORT_GPIO_UART;
    }
}

static int gpio_uart_verify_port(struct uart_port *port, struct serial_struct *ser)
{
    if (ser->type != PORT_UNKNOWN && ser->type != PORT_GPIO_UART)
        return -EINVAL;

    return 0;
}

static const struct uart_ops gpio_uart_ops = {
    .tx_empty	= gpio_uart_tx_empty,
    .set_mctrl	= gpio_uart_set_mctrl,
    .get_mctrl	= gpio_uart_get_mctrl,
    .start_tx	= gpio_uart_start_tx,
    .stop_tx	= gpio_uart_stop_tx,
    .stop_rx	= gpio_uart_stop_rx,
    .enable_ms	= gpio_uart_enable_ms,
    .break_ctl	= gpio_uart_break_ctl,
    .startup	= gpio_uart_startup,
    .shutdown	= gpio_uart_shutdown,
    .flush_buffer	= gpio_uart_flush_buffer,
    .set_termios	= gpio_uart_set_termios,
    .type		= gpio_uart_type,
    .release_port	= gpio_uart_release_port,
    .request_port	= gpio_uart_request_port,
    .config_port	= gpio_uart_config_port,
    .verify_port	= gpio_uart_verify_port,
};

static struct uart_driver gpio_uart_urtdrv = {
    .owner		= THIS_MODULE,
    .driver_name	= DRIVER_NAME,
    .dev_name	= DEVICE_NAME,
    .major		= 0, /* dynamic allocation */
    .minor		= 0,
    .nr		= GPIO_UART_NR,
    .cons		= NULL,
};

static int gpio_uart_probe(struct platform_device *pdev)
{
    struct gpio_uart_port *up;
    int ret;

    up = devm_kzalloc(&pdev->dev, sizeof(*up), GFP_KERNEL);
    if (!up) {
        return -ENOMEM;
    }

    up->dev = &pdev->dev;

    /* get TX GPIO; optional RX GPIO */
    up->gpiod_tx = devm_gpiod_get_optional(&pdev->dev, "tx", GPIOD_OUT_HIGH);
    if (IS_ERR(up->gpiod_tx)) {
        dev_err(&pdev->dev, "Failed to get TX GPIO\n");
        return PTR_ERR(up->gpiod_tx);
    }

    up->gpiod_rx = devm_gpiod_get_optional(&pdev->dev, "rx", GPIOD_IN);
    if (IS_ERR(up->gpiod_rx)) {
        dev_err(&pdev->dev, "Failed to get RX GPIO\n");
        return PTR_ERR(up->gpiod_rx);
    }

    int irq = gpiod_to_irq(up->gpiod_rx);
    if (irq < 0) {
        dev_err(&pdev->dev, "No IRQ for RX GPIO %d\n", irq);
        return -EINVAL;
    } else {
        up->rx_irq = irq;
        dev_info(&pdev->dev, "RX GPIO IRQ: %d\n", irq);
    }
    
    /* Initialize UART port */
    up->port.dev = &pdev->dev;
    up->port.type = PORT_GPIO_UART;
    up->port.line = 0;
    up->port.iotype = UPIO_PORT;
    up->port.iobase = 0;
    up->port.membase = (void __iomem *)~0;
    up->port.fifosize = FIFO_SIZE;
    up->port.ops = &gpio_uart_ops;
    up->port.flags = UPF_FIXED_TYPE | UPF_BOOT_AUTOCONF;

    /* Set default configuration */
    up->config.baud = 9600;
    up->config.databits = 8;
    up->config.stopbits = 1;
    up->config.parity = 0; /* no parity */
    gpio_uart_set_bit_period(up, up->config.baud);

    /* Add UART port */
    ret = uart_add_one_port(&gpio_uart_urtdrv, &up->port);
    if (ret) {
        dev_err(&pdev->dev, "Failed to add UART port\n");
        return ret;
    }

    platform_set_drvdata(pdev, up);
    dev_info(&pdev->dev, "GPIO UART driver probe completed\n");

    return 0;
}

static void gpio_uart_remove(struct platform_device *pdev)
{
    struct gpio_uart_port *up = platform_get_drvdata(pdev);

    if (up) {
        uart_remove_one_port(&gpio_uart_urtdrv, &up->port);
    }
}

static const struct of_device_id gpio_uart_of_match[] = {
    { .compatible = "gpio-uart" },
    { }
};
MODULE_DEVICE_TABLE(of, gpio_uart_of_match);

static struct platform_driver gpio_uart_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = gpio_uart_of_match,
    },
    .probe = gpio_uart_probe,
    .remove = gpio_uart_remove,
};

static int __init gpio_uart_init(void)
{
    int ret;

    ret = uart_register_driver(&gpio_uart_urtdrv);
    if (ret) {
        pr_err("Failed to register GPIO UART driver\n");
        return ret;
    }

    ret = platform_driver_register(&gpio_uart_driver);
    if (ret) {
        pr_err("Failed to register GPIO UART platform driver\n");
        uart_unregister_driver(&gpio_uart_urtdrv);
        return ret;
    }

    pr_info("GPIO UART driver initialized\n");
    return 0;
}

static void __exit gpio_uart_exit(void)
{
    platform_driver_unregister(&gpio_uart_driver);
    uart_unregister_driver(&gpio_uart_urtdrv);
    pr_info("GPIO UART driver exited\n");
}

module_init(gpio_uart_init);
module_exit(gpio_uart_exit);

MODULE_AUTHOR("Zhang Yunduan");
MODULE_DESCRIPTION("GPIO bitbanged UART");
MODULE_LICENSE("GPL");
