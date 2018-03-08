/*
 *
 * Tinker board Touchscreen MCU driver.
 *
 * Copyright (c) 2016 ASUSTek Computer Inc.
 * Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/string.h>
#include <linux/reboot.h>
#include "tinker_mcu.h"

#define BL_DEBUG 0
static struct tinker_mcu_data *g_mcu_data;
static int connected = 0;
static int lcd_bright_level = 0;
static int is_mcu_power_on = 0;

//https://github.com/raspberrypi/linux/blob/17b06b5ef4c3d9c78524e5e23ae4c4f8e3d74aa1/drivers/gpu/drm/panel/panel-raspberrypi-touchscreen.c#L435
//linux/drivers/gpu/drm/panel/panel-raspberrypi-touchscreen.c

/* I2C registers of the Atmel microcontroller. */
enum REG_ADDR {
	REG_ID = 0x80,
	REG_PORTA, // BIT(2) for horizontal flip, BIT(3) for vertical flip
	REG_PORTB,
	REG_PORTC,
	REG_PORTD,
	REG_POWERON,
	REG_PWM,
	REG_DDRA,
	REG_DDRB,
	REG_DDRC,
	REG_DDRD,
	REG_TEST,
	REG_WR_ADDRL,
	REG_WR_ADDRH,
	REG_READH,
	REG_READL,
	REG_WRITEH,
	REG_WRITEL,
	REG_ID2,
};

static int tinker_mcu_i2c_read(struct tinker_mcu_data *ts, u8 reg)
{
	return i2c_smbus_read_byte_data(ts->client, reg);
}

static void tinker_mcu_i2c_write(struct tinker_mcu_data *ts,
				      u8 reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(ts->client, reg, val);
	if (ret)
		LOG_ERR("I2C write failed: %d\n", ret);
}

static int init_cmd_check(struct tinker_mcu_data *mcu_data)
{
	int ret;
	ret = tinker_mcu_i2c_read(mcu_data, REG_ID);
	if (ret < 0)
		goto error;

	printk(KERN_ERR "recv_cmds: 0x%X\n", ret);
	if (ret != 0xDE && ret != 0xC3) {
		LOG_ERR("read wrong info\n");
		ret = -EINVAL;
		goto error;
	}
	return 0;
error:
	return ret;
}



int tinker_mcu_screen_power_up(void)
{
	int i=0;
	if (!connected)
		return -ENODEV;

	LOG_INFO("\n");

	/*Turn off at boot, so we can cleanly sequence powering on.*/
	is_mcu_power_on =0;
	tinker_mcu_i2c_write(g_mcu_data, REG_POWERON, 0);

	msleep(800);

	tinker_mcu_i2c_write(g_mcu_data, REG_POWERON, 1);
	/* Wait for nPWRDWN to go low to indicate poweron is done. */
	for (i = 0; i < 100; i++) {
		if (tinker_mcu_i2c_read(g_mcu_data, REG_PORTB) & 1)
			break;
	}
	tinker_mcu_i2c_write(g_mcu_data, REG_PORTA, BIT(2));
	is_mcu_power_on =1;

	return 0;
}
EXPORT_SYMBOL_GPL(tinker_mcu_screen_power_up);

int tinker_mcu_set_bright(int bright)
{
	if (!connected)
		return -ENODEV;

	if (bright > 0xff || bright < 0)
		return -EINVAL;

	LOG_INFO("bright = 0x%x\n", bright);

	if (bright==0) {
		tinker_mcu_i2c_write(g_mcu_data, REG_PWM, 0);
		tinker_mcu_i2c_write(g_mcu_data, REG_POWERON, 0);
		udelay(1);
		is_mcu_power_on=0;
	} else {
		tinker_mcu_i2c_write(g_mcu_data, REG_PWM, bright);
	}

	lcd_bright_level = bright;

	return 0;
}
EXPORT_SYMBOL_GPL(tinker_mcu_set_bright);

int tinker_mcu_is_mcu_power_on(void)
{
	return is_mcu_power_on;
}
EXPORT_SYMBOL_GPL(tinker_mcu_is_mcu_power_on);


static ssize_t tinker_mcu_bl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    if(BL_DEBUG) LOG_INFO("get bright = 0x%x\n", lcd_bright_level);

    return sprintf(buf, "%d\n", lcd_bright_level);
}

static ssize_t tinker_mcu_bl_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	value = simple_strtoul(buf, NULL, 0);

	if((value < 0) || (value > 255)) {
		LOG_ERR("Invalid value for backlight setting, value = %d\n", value);
	} else
		tinker_mcu_set_bright(value);

	return strnlen(buf, count);
}
static DEVICE_ATTR(tinker_mcu_bl, S_IRUGO | S_IWUSR, tinker_mcu_bl_show, tinker_mcu_bl_store);

int tinker_mcu_is_connected(void)
{
	return connected;
}
EXPORT_SYMBOL_GPL(tinker_mcu_is_connected);

static int tinker_mcu_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct tinker_mcu_data *mcu_data;
	int ret;

	LOG_INFO("address = 0x%x\n", client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		LOG_ERR("I2C check functionality failed\n");
		return -ENODEV;
	}

	mcu_data = kzalloc(sizeof(struct tinker_mcu_data), GFP_KERNEL);
	if (mcu_data == NULL) {
		LOG_ERR("no memory for device\n");
		return -ENOMEM;
	}

	mcu_data->client = client;
	i2c_set_clientdata(client, mcu_data);
	g_mcu_data = mcu_data;

	ret = init_cmd_check(mcu_data);
	if (ret < 0) {
		LOG_ERR("init_cmd_check failed, %d\n", ret);
		goto error;
	}
	connected = 1;

	ret = device_create_file(&client->dev, &dev_attr_tinker_mcu_bl);
	if (ret != 0) {
		dev_err(&client->dev, "Failed to create tinker_mcu_bl sysfs files %d\n", ret);
		return ret;
	}

	return 0;

error:
	kfree(mcu_data);
	return ret;
}

static int tinker_mcu_remove(struct i2c_client *client)
{
	struct tinker_mcu_data *mcu_data = i2c_get_clientdata(client);
	connected = 0;
	kfree(mcu_data);
	return 0;
}

static void tinker_mcu_shutdown(struct i2c_client *client)
{
	is_mcu_power_on = 0;
	tinker_mcu_i2c_write(g_mcu_data, REG_POWERON, 0);
	return;
}

static const struct i2c_device_id tinker_mcu_id[] = {
	{"tinker_mcu", 0},
	{},
};

static struct i2c_driver tinker_mcu_driver = {
	.driver = {
		.name = "tinker_mcu",
	},
	.probe = tinker_mcu_probe,
	.remove = tinker_mcu_remove,
	.shutdown = tinker_mcu_shutdown,
	.id_table = tinker_mcu_id,
};

static int bl_reboot_notifier_call(struct notifier_block *self, unsigned long event, void *data)
{
	tinker_mcu_set_bright(0);
	return NOTIFY_OK;
}

static struct notifier_block bl_reboot_notifier = {
	.notifier_call = bl_reboot_notifier_call,
};

static int __init tinker_mcu_init(void)
{
	register_reboot_notifier(&bl_reboot_notifier);
	return i2c_add_driver(&tinker_mcu_driver);
}

static void __exit tinker_mcu_exit(void)
{
	unregister_reboot_notifier(&bl_reboot_notifier);
	i2c_del_driver(&tinker_mcu_driver);
}

subsys_initcall_sync(tinker_mcu_init);
module_exit(tinker_mcu_exit);

MODULE_DESCRIPTION("Tinker Board TouchScreen MCU driver");
MODULE_LICENSE("GPL v2");
