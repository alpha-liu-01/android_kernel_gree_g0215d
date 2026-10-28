/*
 * Goodix GF816M touch keys for the Gree G0215D.
 *
 * The two capacitive keys next to the front Home switch are read by the
 * GF816M fingerprint sensor, not by the Atmel touchscreen. The stock
 * kernel reported them from its Goodix SPI driver on an input device
 * named "tiny4412-key". This driver keeps that name and the key path,
 * and also exposes /dev/goodix_fp for the stock Goodix userspace.
 * Key mode is handled here. Image mode leaves the status register for
 * userspace and wakes it on netlink protocol 25. The sensor firmware
 * lives in its own flash and is not rewritten here.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/netlink.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/poll.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <net/netlink.h>
#include <net/net_namespace.h>

#define GF_W			0xf0
#define GF_R			0xf1

#define GF_REG_VERSION		0x8000
#define GF_REG_HEARTBEAT	0x8040
#define GF_REG_MODE		0x8043
#define GF_REG_CONFIG		0x8047
#define GF_REG_STATUS		0x8140
#define GF_REG_IMAGE		0x8141
#define GF_REG_FW_STATE		0x41e4

#define GF_FW_RUNNING		0xbe
#define GF_HEARTBEAT_ALIVE	0xc6
#define GF_HEARTBEAT_HOST	0xaa
#define GF_VERSION_FIRST	0x47

#define GF_MODE_IMAGE		0x00
#define GF_MODE_KEY		0x01
#define GF_MODE_SLEEP		0x02
#define GF_MODE_FF		0x03
#define GF_MODE_DEBUG		0x56

#define GF_STA_READY		BIT(7)
#define GF_STA_IMAGE		BIT(6)
#define GF_STA_HOME_MASK	BIT(5)
#define GF_STA_HOME		BIT(4)
#define GF_STA_BACK_MASK	BIT(3)
#define GF_STA_BACK		BIT(2)
#define GF_STA_MENU_MASK	BIT(1)
#define GF_STA_MENU		BIT(0)

#define GF_SPI_HZ		1000000
#define GF_XFER_DELAY_US	5
#define GF_MAX_XFER		256
#define GF_MAX_USER_XFER	16384
#define GF_ESD_MS		1000
#define GF_NETLINK_ROUTE	25
#define GF_NL_MSG_LEN		32

/* libfphal ioctl numbers, 64-bit. */
#define GF_IOC_RESET		0x00006700
#define GF_IOC_TRANSFER		0xc0106701
#define GF_IOC_SETSPEED		0x40046702
#define GF_IOC_POWER_OFF	0x00006703
#define GF_IOC_POWER_ON		0x00006704
#define GF_IOC_DISABLE_IRQ	0x00006705
#define GF_IOC_ENABLE_IRQ	0x00006706
#define GF_IOC_SENDKEY		0x40086707

struct gf_ioc_transfer {
	u8 cmd;
	u8 pad;
	u16 addr;
	u32 len;
	u64 buf;
};

struct gf_key_event {
	u32 code;
	u32 value;
};

static bool report_home;
module_param(report_home, bool, 0644);
MODULE_PARM_DESC(report_home,
	"Report finger-on-sensor as KEY_HOME (the switch already sends it)");

/* Stock GF816M config, written to 0x8047 at every probe and reset. */
static const u8 gf816m_cfg[249] = {
	0x41, 0x3c, 0x3c, 0xe4, 0x0c, 0x30, 0x5f, 0x03, 0x00, 0x80, 0x50, 0x41,
	0x41, 0xe4, 0x0c, 0x30, 0x2f, 0x03, 0x02, 0x03, 0x11, 0xa0, 0x0d, 0xfa,
	0x14, 0x01, 0x00, 0x0f, 0x0f, 0xb2, 0x3f, 0xb3, 0x33, 0x03, 0x30, 0x20,
	0x00, 0x05, 0x0e, 0x09, 0x0a, 0x0b, 0x22, 0x95, 0x08, 0x10, 0x12, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18,
	0xe9, 0x25, 0x04, 0xca, 0xa4, 0x26, 0x66, 0x00, 0x00, 0x00, 0x01, 0x00,
	0x01, 0x0f, 0x96, 0x00, 0x01, 0x02, 0x85, 0x00, 0x03, 0x20, 0x20, 0x50,
	0x3e, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x03, 0x09, 0x00, 0x31, 0x00,
	0x07, 0x14, 0x41, 0x00, 0x50, 0x00, 0x00, 0x00, 0x40, 0x00, 0x04, 0x00,
	0x32, 0x01, 0xa0, 0x00, 0x00, 0x79, 0xc8, 0x00, 0x00, 0x00, 0x28, 0x00,
	0x05, 0x04, 0x50, 0x00, 0x08, 0x00, 0x07, 0x00, 0x20, 0x00, 0x18, 0x00,
	0x3b, 0x00, 0x5d, 0x00, 0x22, 0x00, 0x00, 0x00, 0x03, 0x07, 0x80, 0x00,
	0x20, 0x00, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x01,
};

struct gf816m {
	struct spi_device *spi;
	struct input_dev *input;
	struct regulator *vdd;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_active;
	struct pinctrl_state *pins_suspend;
	struct delayed_work esd_work;
	struct mutex lock;
	int reset_gpio;
	int irq_gpio;
	int irq;
	u8 mode;
	bool back_down;
	bool menu_down;
	bool home_down;
	bool irq_enabled;
	bool misc_ready;
	int open_count;
	u8 tx[GF_MAX_XFER + 3];
	u8 rx[GF_MAX_XFER + 1];
};

static struct gf816m *g_gf;
static struct sock *gf_nl;
static u32 gf_nl_pid;
static struct fasync_struct *gf_async;

static void gf_nl_send_irq(void);

static int gf_write(struct gf816m *gf, u16 addr, const u8 *data, size_t len)
{
	struct spi_transfer xfer = {
		.tx_buf = gf->tx,
		.len = len + 3,
		.delay_usecs = GF_XFER_DELAY_US,
	};
	struct spi_message msg;

	if (len > GF_MAX_XFER)
		return -EINVAL;

	gf->tx[0] = GF_W;
	gf->tx[1] = addr >> 8;
	gf->tx[2] = addr & 0xff;
	memcpy(&gf->tx[3], data, len);

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	return spi_sync(gf->spi, &msg);
}

static int gf_write_byte(struct gf816m *gf, u16 addr, u8 val)
{
	return gf_write(gf, addr, &val, 1);
}

static int gf_read(struct gf816m *gf, u16 addr, u8 *data, size_t len)
{
	struct spi_transfer addr_xfer = {
		.tx_buf = gf->tx,
		.len = 3,
		.delay_usecs = GF_XFER_DELAY_US,
	};
	struct spi_transfer data_xfer = {
		.tx_buf = gf->tx + 3,
		.rx_buf = gf->rx,
		.len = len + 1,
		.delay_usecs = GF_XFER_DELAY_US,
	};
	struct spi_message msg;
	int ret;

	if (len > GF_MAX_XFER)
		return -EINVAL;

	gf->tx[0] = GF_W;
	gf->tx[1] = addr >> 8;
	gf->tx[2] = addr & 0xff;
	spi_message_init(&msg);
	spi_message_add_tail(&addr_xfer, &msg);
	ret = spi_sync(gf->spi, &msg);
	if (ret)
		return ret;

	memset(gf->tx + 3, 0, len + 1);
	gf->tx[3] = GF_R;
	spi_message_init(&msg);
	spi_message_add_tail(&data_xfer, &msg);
	ret = spi_sync(gf->spi, &msg);
	if (ret)
		return ret;

	memcpy(data, &gf->rx[1], len);
	return 0;
}

static int gf_read_byte(struct gf816m *gf, u16 addr, u8 *val)
{
	return gf_read(gf, addr, val, 1);
}

static void gf_hw_reset(struct gf816m *gf)
{
	gpio_set_value(gf->reset_gpio, 1);
	mdelay(5);
	gpio_set_value(gf->reset_gpio, 0);
	mdelay(16);
	gpio_set_value(gf->reset_gpio, 1);
	msleep(70);
}

/* Reset, check that flash firmware runs, then load config and mode. */
static int gf_init_chip(struct gf816m *gf)
{
	struct device *dev = &gf->spi->dev;
	u8 fw_state = 0, ver[16];
	int ret;

	gf_hw_reset(gf);
	msleep(300);

	ret = gf_read_byte(gf, GF_REG_FW_STATE, &fw_state);
	if (ret)
		return ret;
	if (fw_state != GF_FW_RUNNING) {
		dev_err(dev, "no running firmware, 0x%04x = 0x%02x\n",
			GF_REG_FW_STATE, fw_state);
		return -ENODEV;
	}

	ret = gf_read(gf, GF_REG_VERSION, ver, sizeof(ver));
	if (ret)
		return ret;
	dev_info(dev, "chip %.6s fw %02x.%02x.%02x\n", ver, ver[7], ver[8],
		 ver[9]);

	ret = gf_write(gf, GF_REG_CONFIG, gf816m_cfg, sizeof(gf816m_cfg));
	if (ret)
		return ret;

	ret = gf_write_byte(gf, GF_REG_MODE, gf->mode);
	if (ret)
		return ret;

	return gf_write_byte(gf, GF_REG_STATUS, 0);
}

static void gf_report(struct gf816m *gf, unsigned int code, bool *down,
		      bool pressed)
{
	if (*down == pressed)
		return;
	*down = pressed;
	input_report_key(gf->input, code, pressed);
	input_sync(gf->input);
}

static void gf_release_all(struct gf816m *gf)
{
	gf_report(gf, KEY_BACK, &gf->back_down, false);
	gf_report(gf, KEY_MENU, &gf->menu_down, false);
	gf_report(gf, KEY_HOME, &gf->home_down, false);
}

static void gf_report_status(struct gf816m *gf, u8 status)
{
	if (status & GF_STA_BACK_MASK)
		gf_report(gf, KEY_BACK, &gf->back_down, status & GF_STA_BACK);
	if (status & GF_STA_MENU_MASK)
		gf_report(gf, KEY_MENU, &gf->menu_down, status & GF_STA_MENU);
	if (report_home && (status & GF_STA_HOME_MASK))
		gf_report(gf, KEY_HOME, &gf->home_down, status & GF_STA_HOME);
	if (!(status & (GF_STA_BACK_MASK | GF_STA_MENU_MASK | GF_STA_HOME_MASK)))
		gf_release_all(gf);
}

static bool gf_mode_is_capture(u8 mode)
{
	return mode == GF_MODE_IMAGE || mode == GF_MODE_FF ||
	       mode == GF_MODE_DEBUG;
}

static irqreturn_t gf_irq(int irq, void *data)
{
	struct gf816m *gf = data;
	struct device *dev = &gf->spi->dev;
	u8 status = 0, mode = 0;
	bool wake_user = false;
	bool image_irq = false;

	mutex_lock(&gf->lock);

	if (gf_read_byte(gf, GF_REG_STATUS, &status))
		goto out;
	if (!(status & GF_STA_READY)) {
		dev_dbg(dev, "invalid irq, status 0x%02x\n", status);
		goto out;
	}
	if (gf_read_byte(gf, GF_REG_MODE, &mode))
		goto out;

	dev_dbg(dev, "status 0x%02x mode 0x%02x\n", status, mode);

	/*
	 * Once libfphal has armed O_ASYNC, it owns the status byte. That
	 * includes calibration, which waits for SIGIO while the chip is
	 * still in key mode. Clearing 0x8140 here leaves the daemon busy
	 * forever and every later mode change is rejected.
	 *
	 * A ready image (bits 7 and 6) is SIGIO only. GF_Navigation clears
	 * bit 7 as soon as a netlink message arrives, and the reader then
	 * rejects the frame. The daemon also refuses that frame until it
	 * has seen a HOME down/up (bit 5, no bit 6) and set g_touch_flag.
	 * Those statuses are SIGIO too, and are not cleared here. They do
	 * not go out on netlink: KeyProcessThread would call device_action
	 * while capture is in state 2. BACK and MENU stay with the kernel,
	 * including while the chip is left in image mode after cancel.
	 */
	if (gf_async && gf_mode_is_capture(mode)) {
		if ((status & (GF_STA_READY | GF_STA_IMAGE)) ==
		    (GF_STA_READY | GF_STA_IMAGE) ||
		    (status & GF_STA_HOME_MASK)) {
			wake_user = true;
			image_irq = true;
		} else {
			gf_report_status(gf, status);
			gf_write_byte(gf, GF_REG_STATUS, 0);
		}
		goto out;
	}

	if (gf_async || (gf_mode_is_capture(mode) && gf_nl_pid)) {
		wake_user = true;
		image_irq = gf_async && (status & GF_STA_IMAGE);
		if (!image_irq && mode == GF_MODE_KEY)
			gf_report_status(gf, status);
		goto out;
	}

	if (mode == GF_MODE_KEY)
		gf_report_status(gf, status);

	gf_write_byte(gf, GF_REG_STATUS, 0);
out:
	mutex_unlock(&gf->lock);
	if (wake_user) {
		kill_fasync(&gf_async, SIGIO, POLL_IN);
		if (!image_irq)
			gf_nl_send_irq();
	}
	return IRQ_HANDLED;
}

/*
 * The stock driver answers 0xc6 in 0x8040 with 0xaa about once a second
 * and reloads the chip when 0x8040 or 0x8000 read back wrong.
 */
static void gf_esd_work(struct work_struct *work)
{
	struct gf816m *gf = container_of(work, struct gf816m, esd_work.work);
	struct device *dev = &gf->spi->dev;
	u8 hb = 0, ver = 0;
	int tries;

	mutex_lock(&gf->lock);
	/* Userspace owns the chip while /dev/goodix_fp is open. */
	if (gf->open_count) {
		mutex_unlock(&gf->lock);
		goto resched;
	}
	for (tries = 0; tries < 2; tries++) {
		gf_read_byte(gf, GF_REG_HEARTBEAT, &hb);
		gf_read_byte(gf, GF_REG_VERSION, &ver);
		if (hb == GF_HEARTBEAT_ALIVE && ver == GF_VERSION_FIRST)
			break;
	}

	if (tries < 2)
		gf_write_byte(gf, GF_REG_HEARTBEAT, GF_HEARTBEAT_HOST);
	mutex_unlock(&gf->lock);

	if (tries == 2) {
		dev_warn(dev, "chip not responding (0x%02x 0x%02x), reset\n",
			 hb, ver);
		disable_irq(gf->irq);
		mutex_lock(&gf->lock);
		gf_release_all(gf);
		if (gf_init_chip(gf))
			dev_err(dev, "reinit failed\n");
		mutex_unlock(&gf->lock);
		enable_irq(gf->irq);
	}

resched:
	queue_delayed_work(system_freezable_power_efficient_wq, &gf->esd_work,
			   msecs_to_jiffies(GF_ESD_MS));
}

static ssize_t mode_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct gf816m *gf = dev_get_drvdata(dev);
	u8 mode = 0xff;

	mutex_lock(&gf->lock);
	gf_read_byte(gf, GF_REG_MODE, &mode);
	mutex_unlock(&gf->lock);
	return scnprintf(buf, PAGE_SIZE, "0x%02x\n", mode);
}

static ssize_t mode_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct gf816m *gf = dev_get_drvdata(dev);
	u8 mode;
	int ret;

	ret = kstrtou8(buf, 0, &mode);
	if (ret)
		return ret;

	mutex_lock(&gf->lock);
	gf->mode = mode;
	ret = gf_write_byte(gf, GF_REG_MODE, mode);
	mutex_unlock(&gf->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(mode);

static ssize_t status_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct gf816m *gf = dev_get_drvdata(dev);
	u8 status = 0, hb = 0, fw = 0;

	mutex_lock(&gf->lock);
	gf_read_byte(gf, GF_REG_STATUS, &status);
	gf_read_byte(gf, GF_REG_HEARTBEAT, &hb);
	gf_read_byte(gf, GF_REG_FW_STATE, &fw);
	mutex_unlock(&gf->lock);
	return scnprintf(buf, PAGE_SIZE,
			 "status 0x%02x heartbeat 0x%02x fw_state 0x%02x irq_gpio %d\n",
			 status, hb, fw, gpio_get_value(gf->irq_gpio));
}
static DEVICE_ATTR_RO(status);

static struct attribute *gf_attrs[] = {
	&dev_attr_mode.attr,
	&dev_attr_status.attr,
	NULL,
};

static const struct attribute_group gf_attr_group = {
	.attrs = gf_attrs,
};

static int gf_power_on(struct gf816m *gf)
{
	struct device *dev = &gf->spi->dev;
	int ret;

	gf->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(gf->vdd))
		return PTR_ERR(gf->vdd);

	if (regulator_count_voltages(gf->vdd) > 0) {
		ret = regulator_set_voltage(gf->vdd, 2800000, 3300000);
		if (ret)
			dev_warn(dev, "vdd set_voltage failed: %d\n", ret);
	}

	ret = regulator_enable(gf->vdd);
	if (ret)
		return ret;

	msleep(10);
	return 0;
}

static int gf_setup_gpios(struct gf816m *gf)
{
	struct device *dev = &gf->spi->dev;
	struct device_node *np = dev->of_node;
	int ret;

	gf->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(gf->pinctrl)) {
		gf->pins_active = pinctrl_lookup_state(gf->pinctrl,
						       "pmx_fp_active");
		gf->pins_suspend = pinctrl_lookup_state(gf->pinctrl,
							"pmx_fp_suspend");
		if (!IS_ERR(gf->pins_active))
			pinctrl_select_state(gf->pinctrl, gf->pins_active);
	}

	gf->reset_gpio = of_get_named_gpio(np, "goodix,spi_reset_gpio", 0);
	gf->irq_gpio = of_get_named_gpio(np, "goodix,spi_irq_gpio", 0);
	if (!gpio_is_valid(gf->reset_gpio) || !gpio_is_valid(gf->irq_gpio))
		return -EINVAL;

	ret = devm_gpio_request_one(dev, gf->reset_gpio, GPIOF_OUT_INIT_HIGH,
				    "gf816m_reset");
	if (ret)
		return ret;

	ret = devm_gpio_request_one(dev, gf->irq_gpio, GPIOF_IN, "gf816m_irq");
	if (ret)
		return ret;

	gf->irq = gpio_to_irq(gf->irq_gpio);
	return gf->irq < 0 ? gf->irq : 0;
}

static int gf_user_write(struct gf816m *gf, u16 addr, const u8 *data, u32 len)
{
	u8 *tx;
	struct spi_transfer xfer = {
		.delay_usecs = GF_XFER_DELAY_US,
	};
	struct spi_message msg;
	int ret;

	tx = kmalloc(len + 3, GFP_KERNEL);
	if (!tx)
		return -ENOMEM;
	tx[0] = GF_W;
	tx[1] = addr >> 8;
	tx[2] = addr & 0xff;
	memcpy(tx + 3, data, len);
	xfer.tx_buf = tx;
	xfer.len = len + 3;
	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	ret = spi_sync(gf->spi, &msg);
	kfree(tx);
	return ret;
}

static int gf_user_read(struct gf816m *gf, u16 addr, u8 *data, u32 len)
{
	u8 *tx, *rx;
	struct spi_transfer addr_xfer = {
		.delay_usecs = GF_XFER_DELAY_US,
		.len = 3,
	};
	struct spi_transfer data_xfer = {
		.delay_usecs = GF_XFER_DELAY_US,
		.len = len + 1,
	};
	struct spi_message msg;
	int ret;

	tx = kzalloc(len + 1, GFP_KERNEL);
	rx = kzalloc(len + 1, GFP_KERNEL);
	if (!tx || !rx) {
		ret = -ENOMEM;
		goto out;
	}
	tx[0] = GF_W;
	tx[1] = addr >> 8;
	tx[2] = addr & 0xff;
	addr_xfer.tx_buf = tx;
	spi_message_init(&msg);
	spi_message_add_tail(&addr_xfer, &msg);
	ret = spi_sync(gf->spi, &msg);
	if (ret)
		goto out;

	memset(tx, 0, len + 1);
	tx[0] = GF_R;
	data_xfer.tx_buf = tx;
	data_xfer.rx_buf = rx;
	spi_message_init(&msg);
	spi_message_add_tail(&data_xfer, &msg);
	ret = spi_sync(gf->spi, &msg);
	if (!ret)
		memcpy(data, rx + 1, len);
out:
	kfree(tx);
	kfree(rx);
	return ret;
}

static int gf_ioc_transfer(struct gf816m *gf, unsigned long arg)
{
	struct gf_ioc_transfer xfer;
	u8 *buf;
	int ret;

	if (copy_from_user(&xfer, (void __user *)arg, sizeof(xfer)))
		return -EFAULT;
	if (!xfer.len || xfer.len > GF_MAX_USER_XFER)
		return -EINVAL;
	buf = kmalloc(xfer.len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	if (xfer.cmd == GF_W) {
		if (copy_from_user(buf, (void __user *)(uintptr_t)xfer.buf,
				   xfer.len))
			ret = -EFAULT;
		else
			ret = gf_user_write(gf, xfer.addr, buf, xfer.len);
	} else if (xfer.cmd == GF_R) {
		ret = gf_user_read(gf, xfer.addr, buf, xfer.len);
		if (!ret && copy_to_user((void __user *)(uintptr_t)xfer.buf,
					 buf, xfer.len))
			ret = -EFAULT;
	} else {
		ret = -EINVAL;
	}
	kfree(buf);
	return ret;
}

static void gf_nl_input(struct sk_buff *skb)
{
	struct nlmsghdr *nlh;

	if (!skb || skb->len < NLMSG_HDRLEN)
		return;
	nlh = nlmsg_hdr(skb);
	gf_nl_pid = nlh->nlmsg_pid;
}

static int gf_nl_init(void)
{
	struct netlink_kernel_cfg cfg = {
		.input = gf_nl_input,
		.flags = NL_CFG_F_NONROOT_RECV | NL_CFG_F_NONROOT_SEND,
	};

	gf_nl = netlink_kernel_create(&init_net, GF_NETLINK_ROUTE, &cfg);
	if (!gf_nl)
		return -ENOMEM;
	return 0;
}

static void gf_nl_send_irq(void)
{
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	char msg[GF_NL_MSG_LEN] = "irq";

	if (!gf_nl || !gf_nl_pid)
		return;
	skb = nlmsg_new(GF_NL_MSG_LEN, GFP_KERNEL);
	if (!skb)
		return;
	nlh = nlmsg_put(skb, 0, 0, 0, GF_NL_MSG_LEN, 0);
	if (!nlh) {
		kfree_skb(skb);
		return;
	}
	memcpy(nlmsg_data(nlh), msg, sizeof(msg));
	NETLINK_CB(skb).portid = 0;
	NETLINK_CB(skb).dst_group = 0;
	if (netlink_unicast(gf_nl, skb, gf_nl_pid, MSG_DONTWAIT) < 0)
		pr_debug_ratelimited("gf816m netlink send failed\n");
}

static void gf_set_irq(struct gf816m *gf, bool enable)
{
	mutex_lock(&gf->lock);
	if (enable == gf->irq_enabled) {
		mutex_unlock(&gf->lock);
		return;
	}
	gf->irq_enabled = enable;
	mutex_unlock(&gf->lock);
	if (enable)
		enable_irq(gf->irq);
	else
		disable_irq(gf->irq);
}

static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gf816m *gf = filp->private_data;
	struct gf_key_event key;
	u32 speed;
	int ret = 0;
	bool do_irq = false, irq_en = false;

	if (!gf)
		return -ENODEV;

	mutex_lock(&gf->lock);
	switch (cmd) {
	case GF_IOC_RESET:
		gf_hw_reset(gf);
		msleep(300);
		break;
	case GF_IOC_TRANSFER:
		ret = gf_ioc_transfer(gf, arg);
		break;
	case GF_IOC_SETSPEED:
		if (copy_from_user(&speed, (void __user *)arg, sizeof(speed))) {
			ret = -EFAULT;
			break;
		}
		if (speed && speed < 1000)
			speed *= 1000000;
		if (speed) {
			gf->spi->max_speed_hz = speed;
			ret = spi_setup(gf->spi);
		}
		break;
	case GF_IOC_POWER_ON:
	case GF_IOC_POWER_OFF:
		/* The rail stays on so the capacitive keys keep working. */
		break;
	case GF_IOC_DISABLE_IRQ:
		do_irq = true;
		irq_en = false;
		break;
	case GF_IOC_ENABLE_IRQ:
		do_irq = true;
		irq_en = true;
		break;
	case GF_IOC_SENDKEY:
		if (copy_from_user(&key, (void __user *)arg, sizeof(key))) {
			ret = -EFAULT;
			break;
		}
		if (key.code && key.code < KEY_CNT) {
			input_report_key(gf->input, key.code, key.value);
			input_sync(gf->input);
		}
		break;
	default:
		ret = -ENOTTY;
		break;
	}
	mutex_unlock(&gf->lock);
	if (do_irq)
		gf_set_irq(gf, irq_en);
	return ret;
}

/*
 * libfphal reads one frame with read(), not the transfer ioctl. The
 * frame starts at the byte after the status register: 13222 payload
 * bytes, then a big-endian sum, 13225 bytes in total.
 */
static ssize_t gf_dev_read(struct file *filp, char __user *buf, size_t count,
			   loff_t *ppos)
{
	struct gf816m *gf = filp->private_data;
	u8 *kbuf;
	int ret;

	if (!gf)
		return -ENODEV;
	if (!count || count > GF_MAX_USER_XFER)
		return -EINVAL;
	kbuf = kmalloc(count, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;
	mutex_lock(&gf->lock);
	ret = gf_user_read(gf, GF_REG_IMAGE, kbuf, count);
	mutex_unlock(&gf->lock);
	if (!ret) {
		if (copy_to_user(buf, kbuf, count))
			ret = -EFAULT;
		else
			ret = count;
	}
	kfree(kbuf);
	return ret;
}

static int gf_dev_open(struct inode *inode, struct file *filp)
{
	struct gf816m *gf = g_gf;

	if (!gf)
		return -ENODEV;
	filp->private_data = gf;
	mutex_lock(&gf->lock);
	gf->open_count++;
	mutex_unlock(&gf->lock);
	return 0;
}

static int gf_fasync(int fd, struct file *filp, int on)
{
	return fasync_helper(fd, filp, on, &gf_async);
}

static int gf_dev_release(struct inode *inode, struct file *filp)
{
	struct gf816m *gf = filp->private_data;
	bool restore = false;

	gf_fasync(-1, filp, 0);
	mutex_lock(&gf->lock);
	if (gf->open_count > 0)
		gf->open_count--;
	restore = !gf->open_count;
	mutex_unlock(&gf->lock);
	if (!restore)
		return 0;
	gf_set_irq(gf, true);
	mutex_lock(&gf->lock);
	gf->mode = GF_MODE_KEY;
	if (gf_init_chip(gf))
		dev_err(&gf->spi->dev, "restore key mode failed\n");
	mutex_unlock(&gf->lock);
	return 0;
}

static const struct file_operations gf_fops = {
	.owner = THIS_MODULE,
	.open = gf_dev_open,
	.read = gf_dev_read,
	.release = gf_dev_release,
	.unlocked_ioctl = gf_ioctl,
	.fasync = gf_fasync,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gf_ioctl,
#endif
};

static struct miscdevice gf_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "goodix_fp",
	.fops = &gf_fops,
};

static int gf_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct gf816m *gf;
	int ret;

	gf = devm_kzalloc(dev, sizeof(*gf), GFP_KERNEL);
	if (!gf)
		return -ENOMEM;

	gf->spi = spi;
	gf->mode = GF_MODE_KEY;
	g_gf = gf;
	mutex_init(&gf->lock);
	INIT_DELAYED_WORK(&gf->esd_work, gf_esd_work);
	spi_set_drvdata(spi, gf);

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	spi->max_speed_hz = GF_SPI_HZ;
	ret = spi_setup(spi);
	if (ret)
		return ret;

	ret = gf_setup_gpios(gf);
	if (ret) {
		dev_err(dev, "gpio setup failed: %d\n", ret);
		return ret;
	}

	ret = gf_power_on(gf);
	if (ret) {
		dev_err(dev, "power on failed: %d\n", ret);
		return ret;
	}

	ret = gf_init_chip(gf);
	if (ret) {
		dev_err(dev, "chip init failed: %d\n", ret);
		goto err_power;
	}

	gf->input = devm_input_allocate_device(dev);
	if (!gf->input) {
		ret = -ENOMEM;
		goto err_power;
	}

	gf->input->name = "tiny4412-key";
	gf->input->id.bustype = BUS_SPI;
	__set_bit(EV_KEY, gf->input->evbit);
	/*
	 * Only the three navigation keys. Advertising every key code also
	 * sets BTN_TOUCH, and EventHub then treats the device as a stylus
	 * and drops the keyboard class.
	 */
	__set_bit(KEY_HOME, gf->input->keybit);
	__set_bit(KEY_MENU, gf->input->keybit);
	__set_bit(KEY_BACK, gf->input->keybit);

	ret = input_register_device(gf->input);
	if (ret)
		goto err_power;

	ret = devm_request_threaded_irq(dev, gf->irq, NULL, gf_irq,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"gf816m", gf);
	if (ret) {
		dev_err(dev, "irq %d request failed: %d\n", gf->irq, ret);
		goto err_power;
	}
	gf->irq_enabled = true;

	ret = misc_register(&gf_misc);
	if (ret) {
		dev_err(dev, "/dev/goodix_fp failed: %d\n", ret);
	} else {
		gf->misc_ready = true;
		if (gf_nl_init())
			dev_err(dev, "netlink %d failed\n", GF_NETLINK_ROUTE);
	}

	ret = sysfs_create_group(&dev->kobj, &gf_attr_group);
	if (ret)
		dev_warn(dev, "sysfs group failed: %d\n", ret);

	queue_delayed_work(system_freezable_power_efficient_wq, &gf->esd_work,
			   msecs_to_jiffies(GF_ESD_MS));
	return 0;

err_power:
	regulator_disable(gf->vdd);
	return ret;
}

static int gf_remove(struct spi_device *spi)
{
	struct gf816m *gf = spi_get_drvdata(spi);

	sysfs_remove_group(&spi->dev.kobj, &gf_attr_group);
	devm_free_irq(&spi->dev, gf->irq, gf);
	cancel_delayed_work_sync(&gf->esd_work);
	if (gf->misc_ready)
		misc_deregister(&gf_misc);
	if (gf_nl) {
		netlink_kernel_release(gf_nl);
		gf_nl = NULL;
	}
	g_gf = NULL;
	regulator_disable(gf->vdd);
	return 0;
}

static const struct of_device_id gf816m_of_match[] = {
	{ .compatible = "goodix,gf" },
	{ }
};
MODULE_DEVICE_TABLE(of, gf816m_of_match);

static struct spi_driver gf816m_driver = {
	.driver = {
		.name = "gf816m_keys",
		.owner = THIS_MODULE,
		.of_match_table = gf816m_of_match,
	},
	.probe = gf_probe,
	.remove = gf_remove,
};
module_spi_driver(gf816m_driver);

MODULE_DESCRIPTION("Goodix GF816M keys and fingerprint for Gree G0215D");
MODULE_LICENSE("GPL v2");
