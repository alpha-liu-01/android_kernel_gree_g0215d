/*
 * Raon IR blaster on the Gree G0215D.
 *
 * The stock 3.18 driver is built in and not published. This follows that
 * driver's /dev/remote_ctrl_dev ABI: write() and read() are I2C transfers,
 * and the GPIO ioctls pulse the enable, init and reset lines the same way.
 */

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define RAON_MAX_XFER		2048

#define RAON_IOCTL_GET_GPIO_MODE	0x1000
#define RAON_IOCTL_SET_GPIO_MODE	0x1001
#define RAON_IOCTL_GET_GPIO_VALUE	0x1002
#define RAON_IOCTL_SET_GPIO_VALUE	0x1003
#define RAON_IOCTL_SET_RESET		0x1004
#define RAON_IOCTL_MODE_BOOTLOADER	0x40047205
#define RAON_IOCTL_MODE_USERIR		0x40047206
#define RAON_IOCTL_WRITE		0x40047207
#define RAON_IOCTL_READ			0x40047208

struct raon_msg {
	__u32 unused;
	__u16 length;
	__u16 pad;
	__u64 buf;
};

struct raon_data {
	struct i2c_client *client;
	struct mutex lock;
	int en_gpio;
	int init_gpio;
	int reset_gpio;
	int mode;
	int saved_value;
	dev_t devt;
	struct cdev cdev;
	struct class *class;
	struct device *dev;
	u8 xfer[RAON_MAX_XFER];
};

static struct raon_data *raon_dev;

static int raon_gpio_ok(int gpio)
{
	return gpio_is_valid(gpio) && gpio != 0;
}

static void raon_wake_pulse(struct raon_data *d)
{
	if (!raon_gpio_ok(d->init_gpio))
		return;

	gpio_get_value(d->init_gpio);
	gpio_direction_output(d->init_gpio, 0);
	msleep(10);
	gpio_set_value(d->init_gpio, 1);
	msleep(10);
	d->mode = 1;
	d->saved_value = 1;
}

static void raon_reset_pulse(struct raon_data *d, int init_level)
{
	if (d->mode != 1 && raon_gpio_ok(d->init_gpio))
		gpio_direction_output(d->init_gpio, init_level);

	gpio_set_value(d->reset_gpio, 1);
	msleep(1);
	gpio_set_value(d->reset_gpio, 0);
	msleep(10);
	gpio_set_value(d->reset_gpio, 1);
	msleep(64);

	if (!raon_gpio_ok(d->init_gpio))
		return;
	if (d->mode != 1)
		gpio_direction_input(d->init_gpio);
	else
		gpio_set_value(d->init_gpio, d->saved_value);
}

static ssize_t raon_write(struct file *filp, const char __user *buf,
			   size_t count, loff_t *ppos)
{
	struct raon_data *d = raon_dev;
	int ret = -EIO;
	int try;

	if (!d || !d->client)
		return 0;
	if (count > RAON_MAX_XFER)
		count = RAON_MAX_XFER;

	mutex_lock(&d->lock);
	if (copy_from_user(d->xfer, buf, count)) {
		pr_err("[remote_ctrl] copy_from_user failed\n");
		mutex_unlock(&d->lock);
		return 0;
	}
	/*
	 * One transfer has to run until the chip finishes the burst. Timing
	 * out and sending again cuts the code. A NACK means the previous
	 * burst is still finishing, so wait and try this buffer once more.
	 */
	raon_wake_pulse(d);
	pr_info("[remote_ir]: tx len=%zu %02x %02x %02x %02x %02x %02x %02x %02x\n",
		count,
		d->xfer[0], d->xfer[1], d->xfer[2], d->xfer[3],
		count > 4 ? d->xfer[4] : 0, count > 5 ? d->xfer[5] : 0,
		count > 6 ? d->xfer[6] : 0, count > 7 ? d->xfer[7] : 0);
	for (try = 0; try < 16; try++) {
		ret = i2c_master_send(d->client, d->xfer, count);
		if (ret == (int)count)
			break;
		if (ret != -ENOTCONN && ret != -EIO)
			break;
		msleep(100);
	}
	if (ret > 0)
		pr_info("[remote_ir]:remote_ctrl_write   i2c_master_send=%d  success!\n", ret);
	else
		pr_info("[remote_ir]:remote_ctrl_write   i2c_master_send=%d  fail!\n", ret);
	mutex_unlock(&d->lock);
	return ret;
}

static ssize_t raon_read(struct file *filp, char __user *buf,
			  size_t count, loff_t *ppos)
{
	struct raon_data *d = raon_dev;
	struct i2c_msg msg;
	int ret;

	if (!d || !d->client)
		return -ENODEV;
	if (count > RAON_MAX_XFER)
		count = RAON_MAX_XFER;

	msg.addr = d->client->addr;
	msg.flags = I2C_M_RD;
	msg.len = count;
	msg.buf = d->xfer;

	mutex_lock(&d->lock);
	ret = i2c_transfer(d->client->adapter, &msg, 1);
	if (ret == 1)
		ret = copy_to_user(buf, d->xfer, count) ? -EFAULT : count;
	else
		ret = ret < 0 ? ret : -EIO;
	mutex_unlock(&d->lock);
	return ret;
}

static long raon_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct raon_data *d = raon_dev;
	struct raon_msg msg;
	int ret = -EINVAL;
	u16 length;
	void __user *ubuf;

	if (!d || !d->client || !raon_gpio_ok(d->init_gpio))
		return -EINVAL;

	mutex_lock(&d->lock);
	switch (cmd) {
	case RAON_IOCTL_GET_GPIO_MODE:
		ret = d->mode;
		break;
	case RAON_IOCTL_SET_GPIO_MODE:
		if (arg == 0) {
			gpio_direction_input(d->init_gpio);
			d->mode = 0;
			ret = gpio_get_value(d->init_gpio);
		} else if (arg == 1) {
			gpio_direction_output(d->init_gpio, 1);
			d->mode = 1;
			ret = 0;
		}
		break;
	case RAON_IOCTL_GET_GPIO_VALUE:
		ret = gpio_get_value(d->init_gpio);
		d->saved_value = ret;
		break;
	case RAON_IOCTL_SET_GPIO_VALUE:
		gpio_set_value(d->init_gpio, arg ? 1 : 0);
		d->saved_value = arg ? 1 : 0;
		ret = 0;
		break;
	case RAON_IOCTL_SET_RESET:
		if (!raon_gpio_ok(d->reset_gpio)) {
			ret = -1;
			break;
		}
		gpio_set_value(d->reset_gpio, arg ? 1 : 0);
		ret = 0;
		break;
	case RAON_IOCTL_MODE_BOOTLOADER:
	case RAON_IOCTL_MODE_USERIR:
		if (!raon_gpio_ok(d->reset_gpio)) {
			ret = -1;
			break;
		}
		raon_reset_pulse(d, cmd == RAON_IOCTL_MODE_BOOTLOADER ? 1 : 0);
		ret = 0;
		break;
	case RAON_IOCTL_WRITE:
		if (copy_from_user(&msg, (void __user *)arg, sizeof(msg))) {
			ret = -EFAULT;
			break;
		}
		length = msg.length > RAON_MAX_XFER ? RAON_MAX_XFER : msg.length;
		ubuf = (void __user *)(uintptr_t)msg.buf;
		if (!ubuf || copy_from_user(d->xfer, ubuf, length)) {
			ret = -EFAULT;
			break;
		}
		ret = i2c_master_send(d->client, d->xfer, length);
		break;
	case RAON_IOCTL_READ:
		if (copy_from_user(&msg, (void __user *)arg, sizeof(msg))) {
			ret = -EFAULT;
			break;
		}
		length = msg.length > RAON_MAX_XFER ? RAON_MAX_XFER : msg.length;
		ubuf = (void __user *)(uintptr_t)msg.buf;
		if (!ubuf || !length || copy_from_user(d->xfer, ubuf, length)) {
			ret = -EFAULT;
			break;
		} else {
			struct i2c_msg xfer[2];

			xfer[0].addr = d->client->addr;
			xfer[0].flags = 0;
			xfer[0].len = 2;
			xfer[0].buf = d->xfer;
			xfer[1].addr = d->client->addr;
			xfer[1].flags = I2C_M_RD;
			xfer[1].len = length;
			xfer[1].buf = d->xfer;
			ret = i2c_transfer(d->client->adapter, xfer, 2);
			if (ret == 2 && copy_to_user(ubuf, d->xfer, length))
				ret = -EFAULT;
			else if (ret == 2)
				ret = length;
		}
		break;
	default:
		break;
	}
	mutex_unlock(&d->lock);
	return ret;
}

static const struct file_operations raon_fops = {
	.owner = THIS_MODULE,
	.write = raon_write,
	.read = raon_read,
	.unlocked_ioctl = raon_ioctl,
	.llseek = noop_llseek,
};

static int raon_request_out(int gpio, const char *name, int value)
{
	int ret;

	if (!raon_gpio_ok(gpio))
		return 0;
	ret = gpio_request(gpio, name);
	if (ret) {
		pr_err("[remote_ir]: gpio_request %s (%d) failed %d\n",
		       name, gpio, ret);
		return ret;
	}
	return gpio_direction_output(gpio, value);
}

static int raon_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	struct raon_data *d;
	struct pinctrl *pinctrl;
	struct pinctrl_state *state;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("[remote_ir]:i2c_check_functionality failed.\n");
		return -ENODEV;
	}

	d = devm_kzalloc(&client->dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	d->client = client;
	mutex_init(&d->lock);
	d->en_gpio = of_get_named_gpio(np, "remote-ctrl,en-gpio", 0);
	d->init_gpio = of_get_named_gpio(np, "remote-ctrl,init-gpio", 0);
	d->reset_gpio = of_get_named_gpio(np, "remote-ctrl,reset-gpio", 0);
	pr_info("[remote_ir]: en_gpio = %d,init_gpio:%d,reset_gpio:%d.\n",
		d->en_gpio, d->init_gpio, d->reset_gpio);

	pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(pinctrl)) {
		pr_err("[remote_ir]:Device does not use pinctrl\n");
	} else {
		state = pinctrl_lookup_state(pinctrl, "raon_remote_default");
		if (IS_ERR(state))
			pr_err("[remote_ir]:Can not get ts default pinstate\n");
		else
			pinctrl_select_state(pinctrl, state);
	}

	if (!raon_request_out(d->en_gpio, "raon_rc_en", 1))
		msleep(25);
	if (!raon_request_out(d->init_gpio, "raon_rc_int", 1)) {
		gpio_set_value(d->init_gpio, 1);
		d->mode = 1;
		d->saved_value = 1;
	}
	if (!raon_request_out(d->reset_gpio, "raon_rc_reset", 1)) {
		gpio_set_value(d->reset_gpio, 0);
		msleep(10);
		gpio_set_value(d->reset_gpio, 1);
	}

	ret = alloc_chrdev_region(&d->devt, 0, 1, "remote_ctrl_dev");
	if (ret) {
		pr_err("[remote_ir]: alloc_chrdev_region fail.\n");
		return ret;
	}
	cdev_init(&d->cdev, &raon_fops);
	d->cdev.owner = THIS_MODULE;
	ret = cdev_add(&d->cdev, d->devt, 1);
	if (ret) {
		pr_err("[remote_ir]: cdev_add fail.\n");
		goto err_region;
	}
	d->class = class_create(THIS_MODULE, "remote_ctrl_class");
	if (IS_ERR(d->class)) {
		pr_err("[remote_ir]: class_create fail.\n");
		ret = PTR_ERR(d->class);
		goto err_cdev;
	}
	d->dev = device_create(d->class, &client->dev, d->devt, NULL,
			       "remote_ctrl_dev");
	if (IS_ERR(d->dev)) {
		pr_err("[remote_ir]: device_create fail.\n");
		ret = PTR_ERR(d->dev);
		goto err_class;
	}

	i2c_set_clientdata(client, d);
	raon_dev = d;
	return 0;

err_class:
	class_destroy(d->class);
err_cdev:
	cdev_del(&d->cdev);
err_region:
	unregister_chrdev_region(d->devt, 1);
	return ret;
}

static int raon_remove(struct i2c_client *client)
{
	struct raon_data *d = i2c_get_clientdata(client);

	if (!d)
		return 0;
	raon_dev = NULL;
	device_destroy(d->class, d->devt);
	class_destroy(d->class);
	cdev_del(&d->cdev);
	unregister_chrdev_region(d->devt, 1);
	if (raon_gpio_ok(d->en_gpio))
		gpio_free(d->en_gpio);
	if (raon_gpio_ok(d->init_gpio))
		gpio_free(d->init_gpio);
	if (raon_gpio_ok(d->reset_gpio))
		gpio_free(d->reset_gpio);
	return 0;
}

static const struct of_device_id raon_of_match[] = {
	{ .compatible = "raon,remote-ctrl" },
	{ }
};
MODULE_DEVICE_TABLE(of, raon_of_match);

static const struct i2c_device_id raon_id[] = {
	{ "raon_remote_ctrl", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, raon_id);

static struct i2c_driver raon_driver = {
	.driver = {
		.name = "raon_remote_ctrl",
		.of_match_table = raon_of_match,
	},
	.probe = raon_probe,
	.remove = raon_remove,
	.id_table = raon_id,
};
module_i2c_driver(raon_driver);

MODULE_DESCRIPTION("Gree G0215D Raon IR blaster");
MODULE_LICENSE("GPL v2");
