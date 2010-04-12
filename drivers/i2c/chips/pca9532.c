/*
    pca9532.c - 16-bit I2C-bus and SMBus expander optimized for dimming LEDs in
                256 discrete steps. Input/outputs not used as LED drivers can
		be used as regular GPIOs

    Copyright (C) 2008 Embedded Artists AB <info@embeddedartists.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; version 2 of the License.
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon-sysfs.h>

enum pca9532_cmd
{
	PCA9532_INPUT_0		= 0,
	PCA9532_INPUT_1		= 1,
	PCA9532_PSC_0		= 2,
	PCA9532_PWM_0		= 3,
	PCA9532_PSC_1		= 4,
	PCA9532_PWM_1		= 5,
	PCA9532_LS_0		= 6,
	PCA9532_LS_1		= 7,
	PCA9532_LS_2		= 8,
	PCA9532_LS_3		= 9,
};

static int pca9532_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int pca9532_remove(struct i2c_client *client);


/* following are the sysfs callback functions */
static ssize_t pca9532_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	int err = 0;
	int val = 0;

	struct sensor_device_attribute *psa = to_sensor_dev_attr(attr);
	struct i2c_client *client = to_i2c_client(dev);


	/* write to control register */
	err = i2c_master_send(client, (char*)&psa->index ,1);
	
	/* read value from register */
	err = i2c_master_recv(client, (char*)&val, 1);

	if (err < 0) {
		/* failed to read value */
		val = err;
	}	

	return sprintf(buf, "%d\n", val);
}

static ssize_t pca9532_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct sensor_device_attribute *psa = to_sensor_dev_attr(attr);
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 0);
	if (val > 0xff)
		return -EINVAL;
	i2c_smbus_write_byte_data(client, psa->index, val);
	return count;
}

/* Define the device attributes */

#define PCA9532_ENTRY_RO(name, cmd_idx) \
	static SENSOR_DEVICE_ATTR(name, S_IRUGO, pca9532_show, NULL, cmd_idx)

#define PCA9532_ENTRY_RW(name, cmd_idx) \
	static SENSOR_DEVICE_ATTR(name, S_IRUGO | S_IWUSR, pca9532_show, \
				  pca9532_store, cmd_idx)

PCA9532_ENTRY_RO(input0, PCA9532_INPUT_0);
PCA9532_ENTRY_RO(input1, PCA9532_INPUT_1);
PCA9532_ENTRY_RW(psc0,   PCA9532_PSC_0);
PCA9532_ENTRY_RW(pwm0,   PCA9532_PWM_0);
PCA9532_ENTRY_RW(psc1,   PCA9532_PSC_1);
PCA9532_ENTRY_RW(pwm1,   PCA9532_PWM_1);
PCA9532_ENTRY_RW(ls0,    PCA9532_LS_0);
PCA9532_ENTRY_RW(ls1,    PCA9532_LS_1);
PCA9532_ENTRY_RW(ls2,    PCA9532_LS_2);
PCA9532_ENTRY_RW(ls3,    PCA9532_LS_3);

static struct attribute *pca9532_attributes[] = {
	&sensor_dev_attr_input0.dev_attr.attr,
	&sensor_dev_attr_input1.dev_attr.attr,
	&sensor_dev_attr_psc0.dev_attr.attr,
	&sensor_dev_attr_pwm0.dev_attr.attr,
	&sensor_dev_attr_psc1.dev_attr.attr,
	&sensor_dev_attr_pwm1.dev_attr.attr,
	&sensor_dev_attr_ls0.dev_attr.attr,
	&sensor_dev_attr_ls1.dev_attr.attr,
	&sensor_dev_attr_ls2.dev_attr.attr,
	&sensor_dev_attr_ls3.dev_attr.attr,
	NULL
};

static struct attribute_group pca9532_defattr_group = {
	.attrs = pca9532_attributes,
};

static int pca9532_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		goto exit;

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj,
				 &pca9532_defattr_group);
	if (err)
		goto exit;

	return 0;

exit:
	return err;
}

static int pca9532_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &pca9532_defattr_group);
	kfree(i2c_get_clientdata(client));
	return 0;
}


static const struct i2c_device_id pca9532_id[] = {
	{ "pca9532", 0 },
	{ }
};

static struct i2c_driver pca9532_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "pca9532",
	},
	.probe 		= pca9532_probe,
	.remove		= pca9532_remove,
	.id_table	= pca9532_id,
};

static int __init pca9532_init(void)
{
	return i2c_add_driver(&pca9532_driver);
}

static void __exit pca9532_exit(void)
{
	i2c_del_driver(&pca9532_driver);
}

MODULE_AUTHOR("Embedded Artists AB <support@embeddedartists.com>");
MODULE_DESCRIPTION("PCA9532 driver");
MODULE_LICENSE("GPL");

module_init(pca9532_init);
module_exit(pca9532_exit);

