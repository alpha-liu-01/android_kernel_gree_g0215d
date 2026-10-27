/*
 * Goodix GF816M touch keys for the Gree G0215D.
 *
 * The two capacitive keys next to the front Home switch are read by the
 * GF816M fingerprint sensor, not by the Atmel touchscreen. The stock
 * kernel reported them from its Goodix SPI driver on an input device
 * named "tiny4412-key". This driver keeps that name and only does the
 * key path: power, reset, config, and the key bits of the IRQ status.
 * The sensor firmware lives in its own flash and is not rewritten here.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>

#define GF_W			0xf0
#define GF_R			0xf1

#define GF_REG_VERSION		0x8000
#define GF_REG_HEARTBEAT	0x8040
#define GF_REG_MODE		0x8043
#define GF_REG_CONFIG		0x8047
#define GF_REG_STATUS		0x8140
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
#define GF_STA_HOME_MASK	BIT(5)
#define GF_STA_HOME		BIT(4)
#define GF_STA_BACK_MASK	BIT(3)
#define GF_STA_BACK		BIT(2)
#define GF_STA_MENU_MASK	BIT(1)
#define GF_STA_MENU		BIT(0)

#define GF_SPI_HZ		1000000
#define GF_XFER_DELAY_US	5
#define GF_MAX_XFER		256
#define GF_ESD_MS		1000

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
	u8 tx[GF_MAX_XFER + 3];
	u8 rx[GF_MAX_XFER + 1];
};

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

static irqreturn_t gf_irq(int irq, void *data)
{
	struct gf816m *gf = data;
	struct device *dev = &gf->spi->dev;
	u8 status = 0, mode = 0;

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

	switch (mode) {
	case GF_MODE_IMAGE:
	case GF_MODE_KEY:
		if (status & GF_STA_BACK_MASK)
			gf_report(gf, KEY_BACK, &gf->back_down,
				  status & GF_STA_BACK);
		if (status & GF_STA_MENU_MASK)
			gf_report(gf, KEY_MENU, &gf->menu_down,
				  status & GF_STA_MENU);
		if (report_home && (status & GF_STA_HOME_MASK))
			gf_report(gf, KEY_HOME, &gf->home_down,
				  status & GF_STA_HOME);
		if (!(status & (GF_STA_BACK_MASK | GF_STA_MENU_MASK |
				GF_STA_HOME_MASK)))
			gf_release_all(gf);
		break;
	case GF_MODE_SLEEP:
		dev_warn(dev, "irq in sleep mode\n");
		break;
	default:
		dev_warn(dev, "irq in mode 0x%02x\n", mode);
		break;
	}

	gf_write_byte(gf, GF_REG_STATUS, 0);
out:
	mutex_unlock(&gf->lock);
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
	input_set_capability(gf->input, EV_KEY, KEY_BACK);
	input_set_capability(gf->input, EV_KEY, KEY_MENU);
	input_set_capability(gf->input, EV_KEY, KEY_HOME);

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

MODULE_DESCRIPTION("Goodix GF816M touch keys for Gree G0215D");
MODULE_LICENSE("GPL v2");
