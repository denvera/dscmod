#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/interrupt.h> 
#include <linux/hrtimer.h>
#include <linux/ktime.h>

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/kfifo.h>
#include <linux/mutex.h>
#include <asm/uaccess.h>
#include <asm/errno.h>

#define LINUX

#define VERSION "0.1"
#define DEV_NAME "dsc"
#define FIFO_SIZE 1024
#define MSG_FIFO_MAX 128
#define BUF_LEN 128
#define MS_TO_NS(x) (x * 1E6L)


static DEFINE_MUTEX(dsc_mutex);
static DECLARE_KFIFO(dsc_msg_fifo, char, FIFO_SIZE);

static int dsc_open(struct inode *, struct file *);
static int dsc_release(struct inode *, struct file *);
static ssize_t dsc_read(struct file *, char *, size_t, loff_t *);
static ssize_t dsc_write(struct file *, const char *, size_t, loff_t *);


static struct gpio keybus[] = {
        { 7, GPIOF_IN, "DSC CLK" },   // turns LED on
        { 8, GPIOF_IN, "DSC DATA" },   // turns LED off
};

static struct gpio leds[] = {
        {  4, GPIOF_OUT_INIT_LOW, "LED 1" }
};

static char dev_open = 0;
static int major;
static struct class *cl_dsc;
static struct device *dev_dsc;

static struct file_operations fops = {
    .read = dsc_read,
    .write = dsc_write,
    .open = dsc_open,
    .release = dsc_release
};

static int dsc_msg_idx_rd = 0, dsc_msg_idx_wr = 0;
static int msg_len[MSG_FIFO_MAX];
static char cur_msg[BUF_LEN];

static int keybus_irqs[] = { -1, -1 };
static int blink_delay = 100;

static char bit_counter = 0;
static bool start_bit = true;

static irqreturn_t clk_isr(int irq, void *data) {
/*    if (start_bit) {
        start_bit = false;
        return;
    }
*/ // Only triggered on rising edge, don't need above
    //
    // Start clock
    // Read bit
    //cur_msg[bit_counter++] = gpio_get_value();
    // Reset clock

    return IRQ_HANDLED;
}

static int dsc_msg_to_fifo(char *msg, int msg_len) {
    unsigned int copied;
    if (kfifo_avail(&dsc_msg_fifo) < msg_len) {
        printk (KERN_ERR "dsc: No space left in FIFO\n");
        return -ENOSPC;
    }
    copied = kfifo_in(&dsc_msg_fifo, msg, msg_len);
    if (copied != msg_len) {
        printk (KERN_ERR "dsc: Short write to FIFO: %d\/%dn", copied, msg_len);
    }
    dsc_msg_idx_wr = (dsc_msg_idx_wr+1) % MSG_FIFO_MAX;
    return copied;
}

static int dsc_open(struct inode *inode, struct file *file) {
    if (dev_open) return -EBUSY;
    dev_open++;
    return 0;
}
static int dsc_release(struct inode *inode, struct file *file) {
    dev_open--;
    return 0;
}
static ssize_t dsc_read(struct file *filp, char *buffer, size_t length, loff_t *offset) {
    int bytes_read = 0;
    return 0;

}
static ssize_t dsc_write(struct file *filp, const char *buff, size_t len, loff_t *off) {
    printk (KERN_ERR "Sorry, this operation isn't supported.\n");
    return -EINVAL;
}


int gpio_irq(void) {
    int i = 0;
    int ret = 0;
//    for (i = 0; i < 1; i++) {
        ret = gpio_to_irq(keybus[i].gpio);
        if (ret < 0) {
            printk(KERN_ERR "Unable to request IRQ %d: %d\n", i, ret);
            return ret;
        }
        keybus_irqs[i] = ret;
        ret = request_irq(keybus_irqs[i], clk_isr, IRQF_TRIGGER_RISING | IRQF_DISABLED, "dscmod#clk", NULL);
        if (ret) {
            printk(KERN_ERR "Unable to request IRQ: %d\n", ret);
            return ret;
        }
    return 0;
}

int ungpio_irq(void) {
    free_irq(keybus_irqs[0], NULL);
    gpio_free_array(leds, ARRAY_SIZE(leds));
    gpio_free_array(keybus, ARRAY_SIZE(keybus));

    return 0;
}
    
static int __init dsc_init(void)
{
    int ret = 0;
    void *ptr_err;

    struct timeval tv = ktime_to_timeval(ktime_get_real());
    printk(KERN_INFO "DSC GPIO v%s at %d\n", VERSION, (int)tv.tv_sec);

    major = register_chrdev(0, DEV_NAME, &fops);
    if (major < 0)
        goto err_cl_create;

    cl_dsc = class_create(THIS_MODULE, "dsc");
    if (IS_ERR(ptr_err = cl_dsc))
        goto err_cl_create;

    dev_dsc = device_create(cl_dsc, NULL, MKDEV(major, 0), NULL, DEV_NAME);
    if (IS_ERR(ptr_err = dev_dsc))
        goto err_dev_create;
        
    ret = gpio_request_array(leds, ARRAY_SIZE(leds));

    if (ret) {
        printk(KERN_ERR "Unable to request GPIO for LED: %d\n", ret);        
        gpio_free_array(leds, ARRAY_SIZE(leds));
        return ret;
    }

    ret = gpio_request_array(keybus, ARRAY_SIZE(keybus));

    if (ret) {
        printk(KERN_ERR "Unable to request GPIOs for KeyBus: %d\n", ret);
        goto fail2;
    }
    ret = gpio_irq();
    if (ret != 0) {
        printk(KERN_ERR "Unable to request GPIOs for KeyBus: %d\n", ret);
        goto fail2;
    } else {
        printk(KERN_INFO "IRQ setup successful\n");
    }


    return 0;
fail2:
    gpio_free_array(keybus, ARRAY_SIZE(keybus));

fail1:
    gpio_free_array(leds, ARRAY_SIZE(leds));
    return ret; 

err_dev_create:
    class_destroy(cl_dsc);

err_cl_create:
    unregister_chrdev(major, DEV_NAME);
    return PTR_ERR(ptr_err);

}


static void __exit dsc_exit(void)
{
   printk(KERN_INFO "Unloading DSC GPIO\n");
   ungpio_irq();
   device_destroy(cl_dsc, MKDEV(major, 0));
   class_destroy(cl_dsc);
   unregister_chrdev(major, DEV_NAME);
}
   


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Denver Abrey");
MODULE_DESCRIPTION("Kernel module to speak DSC KeyBus via GPIO");
MODULE_VERSION(VERSION);

module_init(dsc_init);
module_exit(dsc_exit);
