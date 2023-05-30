/**
 * Sensortek STK3A5X/STK3311 Ambient Light and Proximity Sensor
 *
 * Copyright (c) 2015, Intel Corporation.
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License. See the file COPYING in the main
 * directory of this archive for more details.
 *
 * IIO driver for STK3A5X/STK3311. 7-bit I2C address: 0x48.
 */

#include <linux/acpi.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_irq.h>

#define STK3A5X_REG_STATE 0x00
#define STK3A5X_REG_PSCTRL 0x01
#define STK3A5X_REG_ALSCTRL 0x02
#define STK3A5X_REG_INT 0x04
#define STK3A5X_REG_THDH_PS 0x06
#define STK3A5X_REG_THDL_PS 0x08
#define STK3A5X_REG_FLAG 0x10
#define STK3A5X_REG_PS_DATA_MSB 0x11
#define STK3A5X_REG_PS_DATA_LSB 0x12
#define STK3A5X_REG_ALS_DATA_MSB 0x13
#define STK3A5X_REG_ALS_DATA_LSB 0x14
#define STK3A5X_REG_ID 0x3E
#define STK3A5X_MAX_REG 0xFF

#define STK3A5X_STATE_EN_PS BIT(0)
#define STK3A5X_STATE_EN_ALS BIT(1)
#define STK3A5X_STATE_STANDBY 0x00

#define STK3A5X_CHIP_ID_VAL 0x52
#define STK3A5X_PSINT_EN 0x01
#define STK3A5X_PS_MAX_VAL 0xFFFF

#define STK3A5X_DRIVER_NAME "stk3a5x"
#define STK3A5X_REGMAP_NAME "stk3a5x_regmap"
#define STK3A5X_EVENT "stk3a5x_event"

#define STK3A5X_PS_SCALE_AVAILABLE "1 2 4 8"
#define STK3A5X_ALS_SCALE_AVAILABLE "1 4 16 64"

#define STK3A5X_PS_IT_AVAILABLE \
	"0.000096 0.000192 0.000384 0.000768 0.001536 0.003007 0.006140"

#define STK3A5X_ALS_IT_AVAILABLE \
	"0.025 0.050 0.100 0.0200 "

#define STK3A5X_REGFIELD(name)                                  \
	do                                                          \
	{                                                           \
		data->reg_##name =                                      \
			devm_regmap_field_alloc(&client->dev, regmap,       \
									stk3a5x_reg_field_##name);  \
		if (IS_ERR(data->reg_##name))                           \
		{                                                       \
			dev_err(&client->dev, "reg field alloc failed.\n"); \
			return PTR_ERR(data->reg_##name);                   \
		}                                                       \
	} while (0)

static const struct reg_field stk3a5x_reg_field_state =
	REG_FIELD(STK3A5X_REG_STATE, 0, 2);
static const struct reg_field stk3a5x_reg_field_als_gain =
	REG_FIELD(STK3A5X_REG_ALSCTRL, 4, 5);
static const struct reg_field stk3a5x_reg_field_ps_gain =
	REG_FIELD(STK3A5X_REG_PSCTRL, 4, 5);
static const struct reg_field stk3a5x_reg_field_als_it =
	REG_FIELD(STK3A5X_REG_ALSCTRL, 0, 3);
static const struct reg_field stk3a5x_reg_field_ps_it =
	REG_FIELD(STK3A5X_REG_PSCTRL, 0, 3);
static const struct reg_field stk3a5x_reg_field_int_ps =
	REG_FIELD(STK3A5X_REG_INT, 0, 0);
static const struct reg_field stk3a5x_reg_field_flag_psint =
	REG_FIELD(STK3A5X_REG_FLAG, 4, 4);
static const struct reg_field stk3a5x_reg_field_flag_nf =
	REG_FIELD(STK3A5X_REG_FLAG, 0, 0);

static const struct reg_field stk3a5x_reg_field_config1 =
	REG_FIELD(0x01, 0, 7);
static const struct reg_field stk3a5x_reg_field_config2 =
	REG_FIELD(0x02, 0, 7);
static const struct reg_field stk3a5x_reg_field_config3 =
	REG_FIELD(0x03, 0, 7);
static const struct reg_field stk3a5x_reg_field_config4 =
	REG_FIELD(0x04, 0, 7);
static const struct reg_field stk3a5x_reg_field_config5 =
	REG_FIELD(0x05, 0, 7);
static const struct reg_field stk3a5x_reg_field_config6 =
	REG_FIELD(0x4E, 0, 7);
static const struct reg_field stk3a5x_reg_field_config7 =
	REG_FIELD(0xDB, 0, 7);
static const struct reg_field stk3a5x_reg_field_config8 =
	REG_FIELD(0x81, 0, 7);
static const struct reg_field stk3a5x_reg_field_config9 =
	REG_FIELD(0xA0, 0, 7);
static const struct reg_field stk3a5x_reg_field_config10 =
	REG_FIELD(0xA1, 0, 7);	
static const struct reg_field stk3a5x_reg_field_config11 =
	REG_FIELD(0xAA, 0, 7);
static const struct reg_field stk3a5x_reg_field_config12 =
	REG_FIELD(0xF6, 0, 7);
static const struct reg_field stk3a5x_reg_field_config13 =
	REG_FIELD(0xFA, 0, 7);

/* Estimate maximum proximity values with regard to measurement scale. */
static const int stk3a5x_ps_max[4] = {
	10000,
	10000,
	10000,
	10000};

static const int stk3a5x_ps_scale_table[][2] = {
	{0, 1}, {1, 2}, {2, 4}, {3, 8}};

static const int stk3a5x_als_scale_table[][2] = {
	{0, 1}, {1, 4}, {2, 16}, {3, 64}};

/* Integration time in seconds, microseconds */
static const int stk3a5x_ps_it_table[][2] = {
	{0, 96},
	{1, 192},
	{2, 384},
	{3, 768},
	{4, 1536},
	{5, 3070},
	{6, 6140},
};

static const int stk3a5x_als_it_table[][2] = {
	{3, 25},
	{4, 50},
	{5, 100},
	{6, 200},
};

struct stk3a5x_data
{
	struct i2c_client *client;
	struct mutex lock;
	int gpio_int;
	struct gpio_desc *gpio_int_desc;
	int irq;
	bool als_enabled;
	bool ps_enabled;
	u64 timestamp;
	struct regmap *regmap;
	struct regmap_field *reg_state;
	struct regmap_field *reg_als_gain;
	struct regmap_field *reg_ps_gain;
	struct regmap_field *reg_als_it;
	struct regmap_field *reg_ps_it;
	struct regmap_field *reg_int_ps;
	struct regmap_field *reg_flag_psint;
	struct regmap_field *reg_flag_nf;

	struct regmap_field *reg_config1;
	struct regmap_field *reg_config2;
	struct regmap_field *reg_config3;
	struct regmap_field *reg_config4;
	struct regmap_field *reg_config5;
	struct regmap_field *reg_config6;
	struct regmap_field *reg_config7;
	struct regmap_field *reg_config8;
	struct regmap_field *reg_config9;
	struct regmap_field *reg_config10;
	struct regmap_field *reg_config11;
	struct regmap_field *reg_config12;
	struct regmap_field *reg_config13;
};

static const struct iio_event_spec stk3a5x_events[] = {
	/* Proximity event */
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
						 BIT(IIO_EV_INFO_ENABLE),
	},
	/* Out-of-proximity event */
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
						 BIT(IIO_EV_INFO_ENABLE),
	},
};

static const struct iio_chan_spec stk3a5x_channels[] = {
	{
		.type = IIO_LIGHT,
		.info_mask_separate =
			BIT(IIO_CHAN_INFO_RAW) |
			BIT(IIO_CHAN_INFO_SCALE) |
			BIT(IIO_CHAN_INFO_INT_TIME),
	},
	{
		.type = IIO_PROXIMITY,
		.info_mask_separate =
			BIT(IIO_CHAN_INFO_RAW) |
			BIT(IIO_CHAN_INFO_SCALE) |
			BIT(IIO_CHAN_INFO_INT_TIME),
		.event_spec = stk3a5x_events,
		.num_event_specs = ARRAY_SIZE(stk3a5x_events),
	}};

static IIO_CONST_ATTR(in_illuminance_scale_available, STK3A5X_ALS_SCALE_AVAILABLE);

static IIO_CONST_ATTR(in_proximity_scale_available, STK3A5X_PS_SCALE_AVAILABLE);

static IIO_CONST_ATTR(in_illuminance_integration_time_available,
					  STK3A5X_ALS_IT_AVAILABLE);

static IIO_CONST_ATTR(in_proximity_integration_time_available,
					  STK3A5X_PS_IT_AVAILABLE);

static struct attribute *stk3a5x_attributes[] = {
	&iio_const_attr_in_illuminance_scale_available.dev_attr.attr,
	&iio_const_attr_in_proximity_scale_available.dev_attr.attr,
	&iio_const_attr_in_illuminance_integration_time_available.dev_attr.attr,
	&iio_const_attr_in_proximity_integration_time_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group stk3a5x_attribute_group = {
	.attrs = stk3a5x_attributes};

static int stk3a5x_get_index(const int table[][2], int table_size,
							 int val, int val2)
{
	int i;

	for (i = 0; i < table_size; i++)
	{
		if (val == table[i][0] && val2 == table[i][1])
			return i;
	}

	return -EINVAL;
}

static int stk3a5x_read_event(struct iio_dev *indio_dev,
							  const struct iio_chan_spec *chan,
							  enum iio_event_type type,
							  enum iio_event_direction dir,
							  enum iio_event_info info,
							  int *val, int *val2)
{
	u8 reg;
	__be16 buf;
	int ret;
	struct stk3a5x_data *data = iio_priv(indio_dev);

	if (info != IIO_EV_INFO_VALUE)
		return -EINVAL;

	/* Only proximity interrupts are implemented at the moment. */
	if (dir == IIO_EV_DIR_RISING)
		reg = STK3A5X_REG_THDH_PS;
	else if (dir == IIO_EV_DIR_FALLING)
		reg = STK3A5X_REG_THDL_PS;
	else
		return -EINVAL;

	mutex_lock(&data->lock);
	ret = regmap_bulk_read(data->regmap, reg, &buf, 2);
	mutex_unlock(&data->lock);
	if (ret < 0)
	{
		dev_err(&data->client->dev, "register read failed\n");
		return ret;
	}
	dev_err(&data->client->dev, "stk3a5x_read_event:0x%x=0x%x\n", reg, buf);
	*val = be16_to_cpu(buf);

	return IIO_VAL_INT;
}

static int stk3a5x_write_event(struct iio_dev *indio_dev,
							   const struct iio_chan_spec *chan,
							   enum iio_event_type type,
							   enum iio_event_direction dir,
							   enum iio_event_info info,
							   int val, int val2)
{
	u8 reg;
	__be16 buf;
	int ret;
	unsigned int index;
	struct stk3a5x_data *data = iio_priv(indio_dev);
	struct i2c_client *client = data->client;

	ret = regmap_field_read(data->reg_ps_gain, &index);
	if (ret < 0)
		return ret;

	if (val < 0 || val > stk3a5x_ps_max[index])
		return -EINVAL;

	if (dir == IIO_EV_DIR_RISING)
		reg = STK3A5X_REG_THDH_PS;
	else if (dir == IIO_EV_DIR_FALLING)
		reg = STK3A5X_REG_THDL_PS;
	else
		return -EINVAL;

	buf = cpu_to_be16(val);
	dev_err(&client->dev, "stk3a5x_write_event:0x%x=0x%x\n", reg, buf);
	ret = regmap_bulk_write(data->regmap, reg, &buf, 2);
	if (ret < 0)
		dev_err(&client->dev, "failed to set PS threshold!\n");

	return ret;
}

static int stk3a5x_read_event_config(struct iio_dev *indio_dev,
									 const struct iio_chan_spec *chan,
									 enum iio_event_type type,
									 enum iio_event_direction dir)
{
	unsigned int event_val;
	int ret;
	struct stk3a5x_data *data = iio_priv(indio_dev);

	ret = regmap_field_read(data->reg_int_ps, &event_val);
	if (ret < 0)
		return ret;
	dev_err(&data->client->dev, "stk3a5x_read_event_config:reg int ps=0x%x\n", event_val);
	return event_val;
}

static int stk3a5x_write_event_config(struct iio_dev *indio_dev,
									  const struct iio_chan_spec *chan,
									  enum iio_event_type type,
									  enum iio_event_direction dir,
									  int state)
{
	int ret;
	struct stk3a5x_data *data = iio_priv(indio_dev);
	struct i2c_client *client = data->client;

	if (state < 0 || state > 7)
		return -EINVAL;

	/* Set INT_PS value */
	mutex_lock(&data->lock);
	ret = regmap_field_write(data->reg_int_ps, state);
	if (ret < 0)
		dev_err(&client->dev, "failed to set interrupt mode\n");
	mutex_unlock(&data->lock);
	dev_err(&client->dev, "stk3a5x_write_event_config:reg int ps=0x%x\n", state);
	return ret;
}

static int stk3a5x_read_raw(struct iio_dev *indio_dev,
							struct iio_chan_spec const *chan,
							int *val, int *val2, long mask)
{
	u8 reg;
	__be16 buf;
	int ret;
	unsigned int index;
	struct stk3a5x_data *data = iio_priv(indio_dev);
	struct i2c_client *client = data->client;

	if (chan->type != IIO_LIGHT && chan->type != IIO_PROXIMITY)
		return -EINVAL;

	switch (mask)
	{
	case IIO_CHAN_INFO_RAW:
		dev_err(&client->dev, "R IIO_CHAN_INFO_RAW\n");
		if (chan->type == IIO_LIGHT)
			reg = STK3A5X_REG_ALS_DATA_MSB;
		else
			reg = STK3A5X_REG_PS_DATA_MSB;

		mutex_lock(&data->lock);
		ret = regmap_bulk_read(data->regmap, reg, &buf, 2);
		if (ret < 0)
		{
			dev_err(&client->dev, "R register read failed\n");
			mutex_unlock(&data->lock);
			return ret;
		}
		*val = be16_to_cpu(buf);
		mutex_unlock(&data->lock);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_INT_TIME:
		dev_err(&client->dev, "R IIO_CHAN_INFO_INT_TIME\n");
		if (chan->type == IIO_LIGHT)
			ret = regmap_field_read(data->reg_als_it, &index);
		else
			ret = regmap_field_read(data->reg_ps_it, &index);
		if (ret < 0)
			return ret;

		if (chan->type == IIO_LIGHT)
		{
			*val = stk3a5x_als_it_table[index][0];
			*val2 = stk3a5x_als_it_table[index][1];
		}
		else
		{
			*val = stk3a5x_ps_it_table[index][0];
			*val2 = stk3a5x_ps_it_table[index][1];
		}

		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_SCALE:
		dev_err(&client->dev, "R IIO_CHAN_INFO_SCALE\n");
		if (chan->type == IIO_LIGHT)
			ret = regmap_field_read(data->reg_als_gain, &index);
		else
			ret = regmap_field_read(data->reg_ps_gain, &index);
		if (ret < 0)
			return ret;

		if (chan->type == IIO_LIGHT)
		{
			*val = stk3a5x_als_scale_table[index][0];
			*val2 = stk3a5x_als_scale_table[index][1];
		}
		else
		{
			*val = stk3a5x_ps_scale_table[index][0];
			*val2 = stk3a5x_ps_scale_table[index][1];
		}

		return IIO_VAL_INT_PLUS_MICRO;
	}

	return -EINVAL;
}

static int stk3a5x_write_raw(struct iio_dev *indio_dev,
							 struct iio_chan_spec const *chan,
							 int val, int val2, long mask)
{
	int ret;
	int index;
	struct stk3a5x_data *data = iio_priv(indio_dev);
	struct i2c_client *client = data->client;
	if (chan->type != IIO_LIGHT && chan->type != IIO_PROXIMITY)
		return -EINVAL;

	switch (mask)
	{
	case IIO_CHAN_INFO_INT_TIME:
		dev_err(&client->dev, "W IIO_CHAN_INFO_RAW\n");
		if (chan->type == IIO_LIGHT)
		{
			index = stk3a5x_get_index(stk3a5x_als_it_table,
									  ARRAY_SIZE(stk3a5x_als_it_table),
									  val, val2);
		}
		else
		{
			index = stk3a5x_get_index(stk3a5x_ps_it_table,
									  ARRAY_SIZE(stk3a5x_ps_it_table),
									  val, val2);
		}

		if (index < 0)
			return -EINVAL;

		mutex_lock(&data->lock);
		if (chan->type == IIO_LIGHT)
			ret = regmap_field_write(data->reg_als_it, index);
		else
			ret = regmap_field_write(data->reg_ps_it, index);
		if (ret < 0)
			dev_err(&data->client->dev,
					"sensor configuration failed\n");
		mutex_unlock(&data->lock);
		return ret;

	case IIO_CHAN_INFO_SCALE:
		dev_err(&client->dev, "W IIO_CHAN_INFO_RAW\n");
		if (chan->type == IIO_LIGHT)
		{
			index = stk3a5x_get_index(stk3a5x_als_scale_table,
									  ARRAY_SIZE(stk3a5x_als_scale_table),
									  val, val2);
		}
		else
		{
			index = stk3a5x_get_index(stk3a5x_ps_scale_table,
									  ARRAY_SIZE(stk3a5x_ps_scale_table),
									  val, val2);
		}

		if (index < 0)
			return -EINVAL;

		mutex_lock(&data->lock);
		if (chan->type == IIO_LIGHT)
			ret = regmap_field_write(data->reg_als_gain, index);
		else
			ret = regmap_field_write(data->reg_ps_gain, index);
		if (ret < 0)
			dev_err(&data->client->dev,
					"sensor configuration failed\n");
		mutex_unlock(&data->lock);
		return ret;
	}

	return -EINVAL;
}

static const struct iio_info stk3a5x_info = {
	.driver_module = THIS_MODULE,
	.read_raw = stk3a5x_read_raw,
	.write_raw = stk3a5x_write_raw,
	.attrs = &stk3a5x_attribute_group,
	.read_event_value = stk3a5x_read_event,
	.write_event_value = stk3a5x_write_event,
	.read_event_config = stk3a5x_read_event_config,
	.write_event_config = stk3a5x_write_event_config,
};

static int stk3a5x_set_state(struct stk3a5x_data *data, u8 state)
{
	int ret;
	struct i2c_client *client = data->client;
	dev_err(&client->dev, "stk3a5x_set_state:state=0x%x\n", state);
	/* 3-bit state; 0b100 is not supported. */
	if (state > 7 || state == 4)
		return -EINVAL;

	mutex_lock(&data->lock);
	ret = regmap_field_write(data->reg_state, state);
	if (ret < 0)
	{
		dev_err(&client->dev, "failed to change sensor state\n");
	}
	else if (state != STK3A5X_STATE_STANDBY)
	{
		/* Don't reset the 'enabled' flags if we're going in standby */
		data->ps_enabled = !!(state & STK3A5X_STATE_EN_PS);
		data->als_enabled = !!(state & STK3A5X_STATE_EN_ALS);
	}
	mutex_unlock(&data->lock);

	return ret;
}

static int stk3a5x_init(struct iio_dev *indio_dev)
{
	int ret;
	int chipid;
	u8 state;
	struct stk3a5x_data *data = iio_priv(indio_dev);
	struct i2c_client *client = data->client;
	dev_err(&client->dev, "stk3a5x_init\n");
	ret = regmap_read(data->regmap, STK3A5X_REG_ID, &chipid);
	if (ret < 0)
		return ret;

	if (chipid != STK3A5X_CHIP_ID_VAL)
	{
		dev_err(&client->dev, "invalid chip id: 0x%x\n", chipid);
		return -ENODEV;
	}

	state = STK3A5X_STATE_EN_ALS | STK3A5X_STATE_EN_PS;
	ret = stk3a5x_set_state(data, state);
	if (ret < 0)
	{
		dev_err(&client->dev, "failed to enable sensor");
		return ret;
	}

	/* Enable PS interrupts */
	ret = regmap_field_write(data->reg_int_ps, STK3A5X_PSINT_EN);
	if (ret < 0)
		dev_err(&client->dev, "failed to enable interrupts!\n");
	// reg[0x01]
	ret = regmap_field_write(data->reg_config1, 0x33);
	if (ret < 0)
		dev_err(&client->dev, "failed to config1 !\n");
	// reg[0x02]
	ret = regmap_field_write(data->reg_config2, 0x12);
	if (ret < 0)
		dev_err(&client->dev, "failed to config2 !\n");
	// reg[0x03]
	ret = regmap_field_write(data->reg_config3, 0x80);
	if (ret < 0)
		dev_err(&client->dev, "failed to config3 !\n");
	// reg[0x04]
	ret = regmap_field_write(data->reg_config4, 0x01);
	if (ret < 0)
		dev_err(&client->dev, "failed to config4 !\n");
	// reg[0x05]
	ret = regmap_field_write(data->reg_config5, 0x40);
	if (ret < 0)
		dev_err(&client->dev, "failed to config5 !\n");
	// reg[0x4E]
	ret = regmap_field_write(data->reg_config6, 0x10);
	if (ret < 0)
		dev_err(&client->dev, "failed to config6 !\n");
	// reg[0xDB]
	ret = regmap_field_write(data->reg_config7, 0x00);
	if (ret < 0)
		dev_err(&client->dev, "failed to config7 !\n");
	// reg[0x81]
	ret = regmap_field_write(data->reg_config8, 0x83);
	if (ret < 0)
		dev_err(&client->dev, "failed to config8 !\n");
	// reg[0xA0]
	ret = regmap_field_write(data->reg_config9, 0x10);
	if (ret < 0)
		dev_err(&client->dev, "failed to config9 !\n");
	// reg[0xA1]
	ret = regmap_field_write(data->reg_config10, 0x7F);
	if (ret < 0)
		dev_err(&client->dev, "failed to config10 !\n");
	// reg[0xAA]
	ret = regmap_field_write(data->reg_config11, 0x0C);
	if (ret < 0)
		dev_err(&client->dev, "failed to config10 !\n");
	// reg[0xF6]
	ret = regmap_field_write(data->reg_config12, 0x82);
	if (ret < 0)
		dev_err(&client->dev, "failed to config10 !\n");
	// reg[0xFA]
	ret = regmap_field_write(data->reg_config13, 0x01);
	if (ret < 0)
		dev_err(&client->dev, "failed to config10 !\n");

	return ret;
}

static bool stk3a5x_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg)
	{
	case STK3A5X_REG_ALS_DATA_MSB:
	case STK3A5X_REG_ALS_DATA_LSB:
	case STK3A5X_REG_PS_DATA_LSB:
	case STK3A5X_REG_PS_DATA_MSB:
	case STK3A5X_REG_FLAG:
		return true;
	default:
		return false;
	}
}

static struct regmap_config stk3a5x_regmap_config = {
	.name = STK3A5X_REGMAP_NAME,
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = STK3A5X_MAX_REG,
	.cache_type = REGCACHE_RBTREE,
	.volatile_reg = stk3a5x_is_volatile_reg,
};

static int stk3a5x_regmap_init(struct stk3a5x_data *data)
{
	struct regmap *regmap;
	struct i2c_client *client;

	client = data->client;
	regmap = devm_regmap_init_i2c(client, &stk3a5x_regmap_config);
	if (IS_ERR(regmap))
	{
		dev_err(&client->dev, "regmap initialization failed.\n");
		return PTR_ERR(regmap);
	}
	data->regmap = regmap;

	STK3A5X_REGFIELD(state);
	STK3A5X_REGFIELD(als_gain);
	STK3A5X_REGFIELD(ps_gain);
	STK3A5X_REGFIELD(als_it);
	STK3A5X_REGFIELD(ps_it);
	STK3A5X_REGFIELD(int_ps);
	STK3A5X_REGFIELD(flag_psint);
	STK3A5X_REGFIELD(flag_nf);

	STK3A5X_REGFIELD(config1);
	STK3A5X_REGFIELD(config2);
	STK3A5X_REGFIELD(config3);
	STK3A5X_REGFIELD(config4);
	STK3A5X_REGFIELD(config5);
	STK3A5X_REGFIELD(config6);
	STK3A5X_REGFIELD(config7);
	STK3A5X_REGFIELD(config8);
	STK3A5X_REGFIELD(config9);
	STK3A5X_REGFIELD(config10);
	STK3A5X_REGFIELD(config11);
	STK3A5X_REGFIELD(config12);
	STK3A5X_REGFIELD(config13);
	return 0;
}

static irqreturn_t stk3a5x_irq_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct stk3a5x_data *data = iio_priv(indio_dev);

	data->timestamp = iio_get_time_ns(indio_dev);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t stk3a5x_irq_event_handler(int irq, void *private)
{
	int ret;
	unsigned int dir;
	u64 event;

	struct iio_dev *indio_dev = private;
	struct stk3a5x_data *data = iio_priv(indio_dev);
	struct i2c_client *client = data->client;
	/* Read FLAG_NF to figure out what threshold has been met. */
	mutex_lock(&data->lock);
	ret = regmap_field_read(data->reg_flag_nf, &dir);
	if (ret < 0)
	{
		dev_err(&data->client->dev, "register read failed\n");
		mutex_unlock(&data->lock);
		return ret;
	}
	dev_err(&client->dev, "stk3a5x_irq_event_handler:flag nf=0x%x\n", dir);

	event = IIO_UNMOD_EVENT_CODE(IIO_PROXIMITY, 1,
								 IIO_EV_TYPE_THRESH,
								 (dir ? IIO_EV_DIR_FALLING : IIO_EV_DIR_RISING));
	iio_push_event(indio_dev, event, data->timestamp);

	/* Reset the interrupt flag */
	ret = regmap_field_write(data->reg_flag_psint, 0);
	if (ret < 0)
		dev_err(&data->client->dev, "failed to reset interrupts\n");
	mutex_unlock(&data->lock);

	return IRQ_HANDLED;
}

static int stk3a5x_gpio_init(struct i2c_client *client, struct stk3a5x_data *data)
{
	if (0 != of_property_read_u32(client->dev.of_node, "stk_int", &data->gpio_int))
	{
		dev_err(&client->dev, "iio allocation failed!\n");
		return -EINVAL;
	}
	data->gpio_int_desc = gpio_to_desc(data->gpio_int);
	data->irq = of_irq_get_byname(client->dev.of_node, "stk_int");
	return 0;
}
static int stk3a5x_probe(struct i2c_client *client,
						 const struct i2c_device_id *id)
{
	int ret;
	struct iio_dev *indio_dev;
	struct stk3a5x_data *data;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
	{
		dev_err(&client->dev, "iio allocation failed!\n");
		return -ENOMEM;
	}

	data = iio_priv(indio_dev);
	data->client = client;
	i2c_set_clientdata(client, indio_dev);
	mutex_init(&data->lock);
	stk3a5x_gpio_init(client, data);
	ret = stk3a5x_regmap_init(data);
	if (ret < 0)
		return ret;

	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &stk3a5x_info;
	indio_dev->name = STK3A5X_DRIVER_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = stk3a5x_channels;
	indio_dev->num_channels = ARRAY_SIZE(stk3a5x_channels);

	ret = stk3a5x_init(indio_dev);
	if (ret < 0)
		return ret;

	if (client->irq > 0)
	{
		ret = devm_request_threaded_irq(&client->dev, client->irq,
										stk3a5x_irq_handler,
										stk3a5x_irq_event_handler,
										IRQF_TRIGGER_FALLING |
											IRQF_ONESHOT,
										STK3A5X_EVENT, indio_dev);
		if (ret < 0)
		{
			dev_err(&client->dev, "request irq %d failed\n",
					client->irq);
			goto err_standby;
		}
	}

	ret = iio_device_register(indio_dev);
	if (ret < 0)
	{
		dev_err(&client->dev, "device_register failed\n");
		goto err_standby;
	}

	return 0;

err_standby:
	stk3a5x_set_state(data, STK3A5X_STATE_STANDBY);
	return ret;
}

static int stk3a5x_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);

	iio_device_unregister(indio_dev);
	return stk3a5x_set_state(iio_priv(indio_dev), STK3A5X_STATE_STANDBY);
}

#ifdef CONFIG_PM_SLEEP
static int stk3a5x_suspend(struct device *dev)
{
	struct stk3a5x_data *data;

	data = iio_priv(i2c_get_clientdata(to_i2c_client(dev)));

	return stk3a5x_set_state(data, STK3A5X_STATE_STANDBY);
}

static int stk3a5x_resume(struct device *dev)
{
	u8 state = 0;
	struct stk3a5x_data *data;

	data = iio_priv(i2c_get_clientdata(to_i2c_client(dev)));
	if (data->ps_enabled)
		state |= STK3A5X_STATE_EN_PS;
	if (data->als_enabled)
		state |= STK3A5X_STATE_EN_ALS;

	return stk3a5x_set_state(data, state);
}

static SIMPLE_DEV_PM_OPS(stk3a5x_pm_ops, stk3a5x_suspend, stk3a5x_resume);

#define STK3A5X_PM_OPS (&stk3a5x_pm_ops)
#else
#define STK3A5X_PM_OPS NULL
#endif

static const struct i2c_device_id stk3a5x_i2c_id[] = {
	{"STK3A5X", 0},
	{"STK3311", 0},
	{}};
MODULE_DEVICE_TABLE(i2c, stk3a5x_i2c_id);

static const struct acpi_device_id stk3a5x_acpi_id[] = {
	{"STK3A5X", 0},
	{"STK3311", 0},
	{}};

MODULE_DEVICE_TABLE(acpi, stk3a5x_acpi_id);

#ifdef CONFIG_OF
static const struct of_device_id stk3a5x_of_match[] = {
	{.compatible = "stk,stk3a5x"},
	{}};
MODULE_DEVICE_TABLE(of, stk3a5x_of_match);
#endif

static struct i2c_driver stk3a5x_driver = {
	.driver = {
		.name = "stk3a5x",
		.pm = STK3A5X_PM_OPS,
		.acpi_match_table = ACPI_PTR(stk3a5x_acpi_id),
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(stk3a5x_of_match)
#endif
	},
	.probe = stk3a5x_probe,
	.remove = stk3a5x_remove,
	.id_table = stk3a5x_i2c_id};

module_i2c_driver(stk3a5x_driver);

MODULE_AUTHOR("Tiberiu Breana <tiberiu.a.breana@intel.com>");
MODULE_DESCRIPTION("STK3A5X Ambient Light and Proximity Sensor driver");
MODULE_LICENSE("GPL v2");
