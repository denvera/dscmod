#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/interrupt.h> 
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/kfifo.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <asm/errno.h>
#include <linux/delay.h>

#include "dscmod.h"

#define LINUX

#define VERSION "0.1"
#define DEV_NAME "dsc"
#define FIFO_SIZE 1024
#define MSG_FIFO_MAX 1024
#define BUF_LEN 1024
#define MS_TO_NS(x) (x * 1E6L)
#define US_TO_NS(x) (x * 1E3L)
#define MSG_POST_WAIT_MS 5
#define DSC_NUM_DEVS 2

static DEFINE_MUTEX(dsc_mutex);
static DECLARE_KFIFO(dsc_msg_fifo, char, FIFO_SIZE);
static DECLARE_KFIFO(dsc_write_fifo, char, FIFO_SIZE);
static DECLARE_WAIT_QUEUE_HEAD(wq);

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

static int dsc_major, dsc_minor;

static struct class *cl_dsc;
static struct device *dev_dsc;

static struct file_operations fops = {
    .read = dsc_read,
    .write = dsc_write,
    .open = dsc_open,
    .release = dsc_release
};

static struct hrtimer msg_timer;
static struct hrtimer bit_timer;
static ktime_t msg_ktime, bit_ktime;

static int dsc_msg_idx_rd = 0, dsc_msg_idx_wr = 0;
static int msg_len[MSG_FIFO_MAX];
static char cur_msg[BUF_LEN];

static char cur_msg_c[BUF_LEN];
static char s_tmp[BUF_LEN] = "C ";

static char write_c;
static bool writing = false;

static unsigned char cur_msg_bin[BUF_LEN];

static int keybus_irqs[] = { -1, -1 };
static int blink_delay = 100;

static unsigned int bit_counter = 0;
static unsigned int bin_bit_counter = 0;
static unsigned int byte_counter = 0;
//static bool start_bit = true;

struct dsc_dev {    
    int idx_r;
    int idx_w;
    dev_t dev;
    struct class *cl;
    struct device *dv;
    struct kfifo r_fifo;
    struct kfifo w_fifo;
    int msg_len[MSG_FIFO_MAX];
    char cur_msg[BUF_LEN];
    bool binary;
    char *name;
    struct semaphore sem;
    struct cdev cdev;        
};

static struct dsc_dev dsc_txt = { .binary = false, .name = "dsc_txt" }, dsc_bin = { .binary = true, .name = "dsc_bin" };

static struct dsc_dev * dsc_devs[] = { &dsc_txt, &dsc_bin };

static enum hrtimer_restart msg_timer_callback(struct hrtimer *timer) {
    int i, fmt_msg_len, copied = 0;
    if (dev_open || 0) {        
        cur_msg[bit_counter] = '\n';
        cur_msg_c[bit_counter] = '\n';
        bit_counter++;
 //       s_tmp = "C ";
//        strncat(s_tmp, cur_msg_c+1, bit_counter+1);
        //cur_msg_bin[++byte_counter] = '\n';
        //dsc_msg_to_fifo(&dsc_msg_fifo, cur_msg, bit_counter);
        for (i = 0; i < DSC_NUM_DEVS; i++) {
            if (!dsc_devs[i]->binary) {
                fmt_msg_len = format_dsc_msg(s_tmp, cur_msg, bit_counter);
//                copied = dsc_msg_to_fifo(&(dsc_devs[i]->r_fifo), cur_msg, bit_counter);
                copied = dsc_msg_to_fifo(&(dsc_devs[i]->r_fifo), s_tmp, fmt_msg_len);
                copied += dsc_msg_to_fifo(&(dsc_devs[i]->r_fifo), "\nC ", 3);
//                copied += dsc_msg_to_fifo(&(dsc_devs[i]->r_fifo), cur_msg_c+1, bit_counter-1);
                fmt_msg_len = format_dsc_msg(s_tmp, cur_msg_c, bit_counter);
                copied += dsc_msg_to_fifo(&(dsc_devs[i]->r_fifo), s_tmp, fmt_msg_len);
                copied += dsc_msg_to_fifo(&(dsc_devs[i]->r_fifo), "\n", 1);
            } else {
                // client //copied = dsc_msg_to_fifo(&(dsc_devs[i]->r_fifo), cur_msg_c, bit_counter);
                copied = dsc_msg_to_fifo(&(dsc_devs[i]->r_fifo), cur_msg_bin, byte_counter+1);
                //memset(cur_msg_bin, 0, BUF_LEN);
            }
            if (copied != -ENOSPC) {
                dsc_devs[i]->msg_len[dsc_devs[i]->idx_w] = copied;
                dsc_devs[i]->idx_w = (dsc_devs[i]->idx_w + 1) % MSG_FIFO_MAX;
                wake_up_interruptible(&wq);
            }
        }
    }
    bit_counter = 0;
    bin_bit_counter = 0;
    byte_counter = 0;
    memset(cur_msg_bin, 0, BUF_LEN);

    return HRTIMER_NORESTART;
}

static enum hrtimer_restart bit_timer_callback(struct hrtimer *timer) {
    char bit;
    int n, b;
    struct timespec read_delay = { .tv_sec = 0, .tv_nsec = US_TO_NS(300) };
    //bit = 48 + gpio_get_value(keybus[1].gpio);
    if (bit_counter == 8 && !kfifo_is_empty(&dsc_write_fifo) && cur_msg_bin[0] == 0x05) {
        n = kfifo_get(&dsc_write_fifo, &write_c);
        gpio_direction_output(keybus[1].gpio, (write_c >> (7-(bit_counter-8))) & 0x01);
        writing = true;
    }
    if (writing && bit_counter > 8 && bit_counter <= 15) {
        //gpio_direction_output(keybus[1].gpio);
        //gpio_set_value(keybus[1].gpio, (write_c >> (bit_counter-11)) & 0x01);
        gpio_direction_output(keybus[1].gpio, (write_c >> (7-(bit_counter-8))) & 0x01);
//        gpio_direction_input(keybus[1].gpio);
    }
    if (bit_counter > 15 && writing) {
        writing = false;
        //gpio_direction_input(keybus[1].gpio);
    }
/*    
    //bin_bit_counter++;
    if (bin_bit_counter % 8 == 0) {
        byte_counter++;
    } else if (bin_bit_counter == 9) {
        bin_bit_counter += 7;
        byte_counter++;
    }
 */ 
//    b = bit_counter;
    bit_counter++;
    usleep_range(250, 280);
    bit = 48 + gpio_get_value(keybus[1].gpio);
    cur_msg_c[bit_counter] = bit;
   /* 
    if (bit_counter == 8 || bit_counter == 10) {
        cur_msg[bit_counter] = ' ';
        cur_msg_c[bit_counter++] = ' ';
        //byte_counter++;
        //bin_bit_counter = 0;
    } else if (bit_counter > 10 && (bit_counter - 10) % 9 == 0) {
        cur_msg[bit_counter] = ' ';
        cur_msg_c[bit_counter++] = ' ';
        //byte_counter ++;
        //bin_bit_counter = 0;
    }
    // Reset clock
    hrtimer_forward_now(&msg_timer, msg_ktime);
*/
    return HRTIMER_NORESTART;
}

static irqreturn_t clk_isr(int irq, void *data) {
    //char bit = '0';
/*    if (start_bit) {
        start_bit = false;
        return;
    }
*/ // Only triggered on rising edge, don't need above
    //
    // Start clock
    // Read bit
    if (writing) {
        //gpio_set_value(keybus[1].gpio, 1);
        gpio_direction_input(keybus[1].gpio);
    }
    hrtimer_start(&msg_timer, msg_ktime, HRTIMER_MODE_REL);
    //hrtimer_start(&bit_timer, bit_ktime, HRTIMER_MODE_REL);

    //bit = 48 + gpio_get_value(keybus[1].gpio);

    if (bit_counter >= BUF_LEN-1) {
        printk (KERN_ERR "dsc: overflowed bit counter\n");    
        return IRQ_HANDLED;
    }
      
    cur_msg[bit_counter] = gpio_get_value(keybus[1].gpio) == 0 ? '0' : '1';
    cur_msg_bin[byte_counter] = (cur_msg_bin[byte_counter] << 1) | (cur_msg[bit_counter] == '0' ? 0 : 1);

/*    if (bin_bit_counter % 8 == 0) {
        byte_counter++;
    } else if (bin_bit_counter == 9) {
        bin_bit_counter += 7;
        byte_counter++;
    }
  */  
//    bit_counter++;
    
    if (bit_counter == 7 || bit_counter == 8) {
//        cur_msg[bit_counter] = ' ';
//        cur_msg_c[bit_counter++] = ' ';
        byte_counter++;
        //bin_bit_counter = 0;
    } else if (bit_counter > 8 && (bit_counter) % 8 == 0) {
//        cur_msg[bit_counter] = ' ';
//        cur_msg_c[bit_counter++] = ' ';
        byte_counter ++;
        //bin_bit_counter = 0;
    }
    // Reset clock

    //hrtimer_forward_now(&msg_timer, msg_ktime);
    hrtimer_start(&bit_timer, bit_ktime, HRTIMER_MODE_REL);
    return IRQ_HANDLED;
}

static int dsc_msg_to_fifo(struct kfifo *fifo, char *msg, int len) {
    unsigned int copied;
    if (kfifo_avail(fifo) < len) {
        //printk (KERN_ERR "dsc: No space left in FIFO for %d\n", len);
        return -ENOSPC;
    }
    copied = kfifo_in(fifo, msg, len);
    if (copied != len) {
        printk (KERN_ERR "dsc: Short write to FIFO: %d/%dn", copied, len);
    }
    //msg_len[dsc_msg_idx_wr] = copied;
    //dsc_msg_idx_wr = (dsc_msg_idx_wr+1) % MSG_FIFO_MAX;
    return copied;
}

static int dsc_open(struct inode *inode, struct file *filp) {
    if (dev_open) return -EBUSY;
    struct dsc_dev *dev;
    dev = container_of(inode->i_cdev, struct dsc_dev, cdev);
    filp->private_data = dev;
    //memset(cur_msg_bin, 0, BUF_LEN);
    dev_open++;
    return 0;
}

static int format_dsc_msg(char *outbuf, char *text_msg, int len) {
    int i = 0, j = 0;
    for (i = 0; i < len; i++) {
        if (i == 8 || i == 9 || i == 17) {
            outbuf[j] = ' ';
            j++;
        } else if ((i > 17) && ((i-1) % 8 == 0)) {
        outbuf[j] = ' ';
            j++;
        }
        outbuf[j++] = text_msg[i];        
    }
    return (j-1);
}

static int dsc_release(struct inode *inode, struct file *file) {
    dev_open--;
    return 0;
}
static ssize_t dsc_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset) {
    int retval;
    unsigned int copied = 0;
/*    if (kfifo_is_empty(&dsc_msg_fifo)) {
        printk (KERN_WARNING "dsc: FIFO empty\n");
        return 0;
    }
*/
    struct dsc_dev *d = filp->private_data;
    struct kfifo *fifo = &d->r_fifo;
    if (wait_event_interruptible(wq, !kfifo_is_empty(fifo))) {
        printk (KERN_WARNING "dsc: read: restart syscall\n");
        return -ERESTARTSYS;
    }
    if (length < d->msg_len[d->idx_r]) {
        printk (KERN_WARNING "dsc: stored msg longer than read\n");
    }
    retval = kfifo_to_user(fifo, buffer, (length >= d->msg_len[d->idx_r]) ? d->msg_len[d->idx_r] : length, &copied);
    if (retval == 0 && (copied != d->msg_len[d->idx_r])) {
        printk (KERN_WARNING "dsc: Short read from fifo: %d/%d\n", copied, d->msg_len[d->idx_r]);
    } else if (retval != 0) {
        printk (KERN_WARNING "dsc: Error reading from kfifo: %d\n", retval);
    }
    d->idx_r = (d->idx_r + 1) % MSG_FIFO_MAX;

/*
    if (wait_event_interruptible(wq, !kfifo_is_empty(&dsc_msg_fifo))) {
        printk (KERN_WARNING "dsc: read: restart syscall\n");
        return -ERESTARTSYS;
    }
    retval = kfifo_to_user(&dsc_msg_fifo, buffer, msg_len[dsc_msg_idx_rd], &copied);
    if (retval == 0 && (copied != msg_len[dsc_msg_idx_rd])) {
        printk (KERN_WARNING "dsc: Short read from fifo: %d/%d\n", copied, msg_len[dsc_msg_idx_rd]);
    } else if (retval != 0) {
        printk (KERN_WARNING "dsc: Error reading from kfifo: %d\n", retval);
    }
    dsc_msg_idx_rd = (dsc_msg_idx_rd + 1) % MSG_FIFO_MAX;
*/
    return retval ? retval : copied;

}
static ssize_t dsc_write(struct file *filp, const char *buff, size_t len, loff_t *off) {
    //printk (KERN_ERR "dsc: Sorry, this operation isn't supported.\n");
    //return -EINVAL;
    int ret;
    char kbuf[FIFO_SIZE];
    int copy_max = len < FIFO_SIZE ? len : FIFO_SIZE;    
    ret = copy_from_user(kbuf, buff, copy_max);
    if (ret != 0) {
        printk(KERN_WARNING "dsc: short write: %d/%d\n", copy_max-ret, len);
    }
    dsc_msg_to_fifo(&dsc_write_fifo, kbuf, copy_max);
    //bit_counter = copy_max;
    //hrtimer_start(&msg_timer, msg_ktime, HRTIMER_MODE_REL);
    return copy_max;
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
        ret = request_irq(keybus_irqs[i], clk_isr, IRQF_TRIGGER_RISING |  IRQF_DISABLED, "dscmod#clk", NULL);
        if (ret) {
            printk(KERN_ERR "Unable to request IRQ: %d\n", ret);
            return ret;
        }
    return 0;
}

static int dsc_init_timer(void) {
    // MSG_POST_WAIT_MS
    msg_ktime = ktime_set(0, MS_TO_NS(MSG_POST_WAIT_MS));
    hrtimer_init(&msg_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    msg_timer.function = &msg_timer_callback;

    bit_ktime = ktime_set(0, US_TO_NS(400)); // 700 works ok for reading
    hrtimer_init(&bit_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    bit_timer.function = &bit_timer_callback;
    return 0;
}

int ungpio_irq(void) {
    free_irq(keybus_irqs[0], NULL);
    gpio_free_array(leds, ARRAY_SIZE(leds));
    gpio_free_array(keybus, ARRAY_SIZE(keybus));

    return 0;
}

int init_dsc_dev(struct dsc_dev *d, int index) {
    int ret1, ret2;
    dev_t dev;
    d->idx_r = 0;
    d->idx_w = 0;
    ret1 = kfifo_alloc(&(d->r_fifo), FIFO_SIZE, GFP_KERNEL);
    ret2 = kfifo_alloc(&(d->w_fifo), FIFO_SIZE, GFP_KERNEL);
    if (ret1 || ret2) {
        printk(KERN_ERR "dsc: error in kfifo_alloc\n");
        return -1;
    }
    cdev_init(&d->cdev, &fops);
    ret1 = alloc_chrdev_region(&dev, 0, 1, d->name);
    dsc_major = MAJOR(dev);
    if (ret1 < 0) {
        printk(KERN_WARNING "dsc: couldn't get major %d\n", dsc_major);
        return -1;
    }
    ret2 = cdev_add(&d->cdev, dev, 1);
    if (ret2) {
        printk (KERN_WARNING "dsc: error %d adding device\n", ret2);
        return ret2;
    }
    d->cl = class_create(THIS_MODULE, d->name);
    if (IS_ERR(d->cl)) {
        printk (KERN_WARNING "dsc: error creating class\n");
        return -1;
    }
    d->dv = device_create(d->cl, NULL, dev, NULL, d->name);
    if (IS_ERR(d->dv)) {
        printk (KERN_WARNING "dsc: error creating dev\n");
        class_destroy(d->cl);
        return -1;
    }
    d->dev = dev;
    return (ret1 | ret2);

}

void destroy_dsc_dev(struct dsc_dev *d) {
   device_destroy(d->cl, d->dev);
   class_destroy(d->cl);
}
 
static int __init dsc_init(void)
{
    int ret = 0;
    void *ptr_err = NULL;

    struct timeval tv = ktime_to_timeval(ktime_get_real());
    printk(KERN_INFO "DSC GPIO v%s at %d\n", VERSION, (int)tv.tv_sec);
    INIT_KFIFO(dsc_msg_fifo);
    INIT_KFIFO(dsc_write_fifo);
    dsc_init_timer();
    memset(cur_msg_bin, 0, BUF_LEN);
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
    init_dsc_dev(&dsc_txt, 0);
    init_dsc_dev(&dsc_bin, 1);

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
   hrtimer_cancel(&msg_timer);
   ungpio_irq();
   device_destroy(cl_dsc, MKDEV(major, 0));
   class_destroy(cl_dsc);
   unregister_chrdev(major, DEV_NAME);
   destroy_dsc_dev(&dsc_txt);
   destroy_dsc_dev(&dsc_bin);

}
   


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Denver Abrey");
MODULE_DESCRIPTION("Kernel module to speak DSC KeyBus via GPIO");
MODULE_VERSION(VERSION);

module_init(dsc_init);
module_exit(dsc_exit);
