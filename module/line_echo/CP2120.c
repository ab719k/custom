/* Modify from SC18IS600 -> CP2120 */

/*
    i2c-CP2120.c - I2C driver for NXP CP2120 SPI to I2C
    Copyright (C) 2011 Lapin Roman <lampus.lapin@gmail.com>
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#define DEBUG 1

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>

#define SPI_CP2120_NAME		"i2c-CP2120"
#define I2C_IRQ_PIN				AT91_PIN_PA28

/*  Register offsets */
#define CP2120_IOCONFIG		0x00
#define CP2120_IOSTATE		0x01
#define CP2120_I2CCLOCK		0x02
#define CP2120_I2CTO		0x03
#define CP2120_I2CSTAT		0x04
#define CP2120_I2CADR		0x05

/* Bitfields in IOConfig */
#define CP2120_IO0_OFFSET	0
#define CP2120_IO0_SIZE		2
#define CP2120_IO1_OFFSET	2
#define CP2120_IO1_SIZE		2
#define CP2120_IO2_OFFSET	4
#define CP2120_IO2_SIZE		2
#define CP2120_IO3_OFFSET	6
#define CP2120_IO3_SIZE		2

/* Bitfields in IOState */
#define CP2120_GPIO_OFFSET	0
#define CP2120_GPIO_SIZE		6

/* Bitfields in I2CTO */
#define CP2120_TE_OFFSET		0
#define CP2120_TE_SIZE		1
#define CP2120_TO_OFFSET		1
#define CP2120_TO_SIZE		7

/* Bitfields in I2CAdr */
#define CP2120_ADR_OFFSET	1
#define CP2120_ADR_SIZE		7

/* Pin configuration */
#define CP2120_PIN_BIDIR		0x00
#define CP2120_PIN_INPUT		0x01
#define CP2120_PIN_PUSHPULL	0x02
#define CP2120_PIN_OPENDRAIN	0x03

/* I2C Status */
#define CP2120_STAT_OK		0xF0 // Transmission successfull
#define CP2120_STAT_ANACK	0xF1 // Device address not acknowledged
#define CP2120_STAT_DNACK	0xF2
#define CP2120_STAT_BUSY		0xF3 // I2C-bus busy
#define CP2120_STAT_TO		0xF4 // I2C-bus time-out
#define CP2120_STAT_INVCNT	0xF5 // I2C-bus invalid data count

/* Commands */
#define CP2120_CMD_WRBLK		0x00 // Write N bytes to I2C-bus slave device
#define CP2120_CMD_RDBLK		0x01 // Read N bytes from I2C-bus slave device
#define CP2120_CMD_RDAWR		0x02 // I2C-bus read after write
#define CP2120_CMD_RDBUF		0x06 // Read buffer
#define CP2120_CMD_WRAWR		0x03 // I2C-bus write after write
#define CP2120_CMD_SPICON		0x18 // SPI configuration
#define CP2120_CMD_WRREG		0x20 // Write to internal register
#define CP2120_CMD_RDREG		0x20 // Read from internal register
#define CP2120_CMD_PWRDWN		0x30 // Power-down mode

/* Bit manipulation macros */
#define CP2120_BIT(name) \
	(1 << CP2120_##name##_OFFSET)
#define CP2120_BF(name,value) \
	(((value) & ((1 << CP2120_##name##_SIZE) - 1)) << CP2120_##name##_OFFSET)
#define CP2120_BFEXT(name,value) \
	(((value) >> CP2120_##name##_OFFSET) & ((1 << CP2120_##name##_SIZE) - 1))
#define CP2120_BFINS(name,value,old) \
	( ((old) & ~(((1 << CP2120_##name##_SIZE) - 1) << CP2120_##name##_OFFSET)) \
	  | CP2120_BF(name,value))

#define fill_wrblk_msg(addr, num) \
	CP2120_spi_msg[0] = CP2120_CMD_WRBLK; \
	CP2120_spi_msg[1] = num; \
	CP2120_spi_msg[2] = (u8)addr<<1
	
#define fill_rdblk_msg(addr, num) \
	CP2120_spi_msg[0] = CP2120_CMD_RDBLK; \
	CP2120_spi_msg[1] = num; \
	CP2120_spi_msg[2] = (u8)addr<<1|0x01

#define fill_rdawr_msg(addr, numw, numr) \
	CP2120_spi_msg[0] = CP2120_CMD_RDAWR; \
	CP2120_spi_msg[1] = numw; \
	CP2120_spi_msg[2] = numr; \
	CP2120_spi_msg[3] = (u8)addr<<1; \
	CP2120_spi_msg[numw+4] = (u8)addr<<1|0x01;

#define MAX_CHIPS 10
#define CP2120_FUNC (I2C_FUNC_SMBUS_QUICK | I2C_FUNC_SMBUS_BYTE | \
		   I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA | \
		   I2C_FUNC_SMBUS_BLOCK_DATA)

static unsigned short chip_addr[MAX_CHIPS];
module_param_array(chip_addr, ushort, NULL, S_IRUGO);
MODULE_PARM_DESC(chip_addr,
		 "Chip addresses (up to 10, between 0x03 and 0x77)");

static unsigned long functionality = CP2120_FUNC;
module_param(functionality, ulong, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(functionality, "Override functionality bitfield");

struct CP2120_chip {
	u8 pointer;
	u16 words[256];		/* Byte operations use the LSB as per SMBus
				   specification */
};

u8 *CP2120_spi_msg;
struct spi_device *CP2120_spi_dev;

static struct CP2120_chip *CP2120_chips;

void debug_spi_msg(u8 num)
{
	u8 i;
	u8 debug_str[256] = {0, };
	
	if(num>0)
		snprintf(debug_str,	256, "%02x", CP2120_spi_msg[0]);
	for(i=1; i<num; i++)
		snprintf(debug_str, 256, "%s, %02x", debug_str, CP2120_spi_msg[i]);
	printk(KERN_DEBUG"CP2120_spi_msg={%s}", debug_str);
}

/* Return negative errno on error. */
static s32 CP2120_xfer(struct i2c_adapter * adap, u16 addr, unsigned short flags,
	char read_write, u8 command, int size, union i2c_smbus_data * data)
{
	s32 ret;
	int i, len;
	struct CP2120_chip *chip = NULL;

	/* Search for the right chip */
	for (i = 0; i < MAX_CHIPS && chip_addr[i]; i++) {
		if (addr == chip_addr[i]) {
			chip = CP2120_chips + i;
			break;
		}
	}
	if (!chip)
		return -ENODEV;

	switch (size) {

	case I2C_SMBUS_QUICK:
		fill_wrblk_msg(addr, 0);
		if (read_write == I2C_SMBUS_READ)
			CP2120_spi_msg[2] |= 0x01;
		dev_dbg(&adap->dev, "smbus quick - addr 0x%02x\n", addr);
		debug_spi_msg(3);
		ret=spi_write(CP2120_spi_dev, CP2120_spi_msg, 3);
		
		ret = 0;
		break;

	case I2C_SMBUS_BYTE:
		if (read_write == I2C_SMBUS_WRITE) {
			chip->pointer = command;
			fill_wrblk_msg(addr, 1);
			CP2120_spi_msg[3] = command;
			dev_dbg(&adap->dev, "smbus byte - addr 0x%02x, "
					"wrote 0x%02x.\n",
					addr, command);
			debug_spi_msg(4);
		} else {
			data->byte = chip->words[chip->pointer++] & 0xff;
			fill_rdblk_msg(addr, 1);
			dev_dbg(&adap->dev, "smbus byte - addr 0x%02x, "
					"read  0x%02x.\n",
					addr, data->byte);
			debug_spi_msg(3);
		}

		ret = 0;
		break;

	case I2C_SMBUS_BYTE_DATA:
		
		if (read_write == I2C_SMBUS_WRITE) {
			chip->words[command] &= 0xff00;
			chip->words[command] |= data->byte;
			fill_wrblk_msg(addr, 2);
			CP2120_spi_msg[3] = command;
			CP2120_spi_msg[4] = data->byte;
			dev_dbg(&adap->dev, "smbus byte data - addr 0x%02x, "
					"wrote 0x%02x at 0x%02x.\n",
					addr, data->byte, command);
			debug_spi_msg(5);
		} else {
			data->byte = chip->words[command] & 0xff;
			fill_rdawr_msg(addr, 1, 1);
			CP2120_spi_msg[4] = command;
			dev_dbg(&adap->dev, "smbus byte data - addr 0x%02x, "
					"read  0x%02x at 0x%02x.\n",
					addr, data->byte, command);
			debug_spi_msg(6);
		}
		chip->pointer = command + 1;

		ret = 0;
		break;

	case I2C_SMBUS_WORD_DATA:
		if (read_write == I2C_SMBUS_WRITE) {
			chip->words[command] = data->word;
			fill_wrblk_msg(addr, 3);
			CP2120_spi_msg[3] = command;
			CP2120_spi_msg[4] = (u8)(data->word&0x00FF);
			CP2120_spi_msg[5] = (u8)(data->word>>8);
			dev_dbg(&adap->dev, "smbus word data - addr 0x%02x, "
					"wrote 0x%04x at 0x%02x.\n",
					addr, data->word, command);
			debug_spi_msg(6);
		} else {
			data->word = chip->words[command];
			fill_rdawr_msg(addr, 1, 2);
			CP2120_spi_msg[4] = command;
			dev_dbg(&adap->dev, "smbus word data - addr 0x%02x, "
					"read  0x%04x at 0x%02x.\n",
					addr, data->word, command);
			debug_spi_msg(6);
		}

		ret = 0;
		break;

	case I2C_SMBUS_BLOCK_DATA:
		len = data->block[0];
		if (read_write == I2C_SMBUS_WRITE) {
			fill_wrblk_msg(addr, len+1);
			CP2120_spi_msg[3] = command;
			CP2120_spi_msg[4] = len;
			for (i = 0; i < len; i++) {
				chip->words[command + i] &= 0xff00;
				chip->words[command + i] |= data->block[1 + i];
				CP2120_spi_msg[5 + i] = data->block[1 + i];
			}
			dev_dbg(&adap->dev, "i2c block data - addr 0x%02x, "
					"wrote %d bytes at 0x%02x.\n",
					addr, len, command);
			debug_spi_msg(len+5);
		} else {
			for (i = 0; i < len; i++) {
				data->block[1 + i] =
					chip->words[command + i] & 0xff;
			}
			dev_dbg(&adap->dev, "i2c block data - addr 0x%02x, "
					"read  %d bytes at 0x%02x.\n",
					addr, len, command);
		}

		ret = 0;
		break;

	default:
		dev_dbg(&adap->dev, "Unsupported I2C/SMBus command\n");
		ret = -EOPNOTSUPP;
		break;
	} /* switch (size) */

	return ret;
}


void CP2120_irq_process(unsigned long arg) {
    printk(KERN_INFO "I2C_IRQ!!!\n");
    CP2120_spi_msg[0] = CP2120_CMD_RDBUF;
    //spi_write_then_read(CP2120_spi_dev, CP2120_spi_msg, 1, u8 * rxbuf, unsigned n_rx);
}

DECLARE_TASKLET(CP2120_irq_tasklet,CP2120_irq_process,0);

static irqreturn_t CP2120_i2c_irq(int irqno, void *dev_id)
{
	tasklet_hi_schedule(&CP2120_irq_tasklet);
	return IRQ_HANDLED;
}

static u32 CP2120_func(struct i2c_adapter *adapter)
{
	return CP2120_FUNC & functionality;
}

static const struct i2c_algorithm smbus_algorithm = {
	.functionality	= CP2120_func,
	.smbus_xfer	= CP2120_xfer,
};

static struct i2c_adapter CP2120_adapter = {
	.owner		= THIS_MODULE,
	.class		= I2C_CLASS_HWMON | I2C_CLASS_SPD,
	.algo		= &smbus_algorithm,
	.name		= "CP2120-i2c",
};

static int __devinit CP2120_probe(struct spi_device *spi) {
	int ret;
	
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	spi->max_speed_hz = 1200000;
	ret = spi_setup(spi);
	if(ret<0)
		return ret;
	CP2120_adapter.dev.parent = &spi->dev;
	CP2120_spi_dev = spi;
	return 0;
	//return sysfs_create_group(&spi->dev.kobj, &CP2120_attr_group);
}

static int __devexit CP2120_remove(struct spi_device *spi) {
	//sysfs_remove_group(&spi->dev.kobj, &CP2120_attr_group);
	return 0;
}

static struct spi_driver CP2120_driver = {
	.driver = {
		.name	= SPI_CP2120_NAME,
		.owner	= THIS_MODULE,
	},
	.probe	= CP2120_probe,
	.remove	= __devexit_p(CP2120_remove),
};

static int __init i2c_CP2120_init(void)
{
	int i, ret;

	if (!chip_addr[0]) {
		printk(KERN_ERR "i2c-stub: Please specify a chip address\n");
		return -ENODEV;
	}

	for (i = 0; i < MAX_CHIPS && chip_addr[i]; i++) {
		if (chip_addr[i] < 0x03 || chip_addr[i] > 0x77) {
			printk(KERN_ERR "i2c-stub: Invalid chip address "
			       "0x%02x\n", chip_addr[i]);
			return -EINVAL;
		}

		printk(KERN_INFO "i2c-stub: Virtual chip at 0x%02x\n",
		       chip_addr[i]);
	}

	/* Allocate memory for all chips at once */
	CP2120_chips = kzalloc(i * sizeof(struct CP2120_chip), GFP_KERNEL);
	if (!CP2120_chips) {
		printk(KERN_ERR "i2c-stub: Out of memory\n");
		return -ENOMEM;
	}
	
	/* TODO: Move all at91_* to the board file!!! */
	at91_set_GPIO_periph(I2C_IRQ_PIN,0);
	if(at91_set_gpio_input(I2C_IRQ_PIN, 0)) {
		printk(KERN_DEBUG"Could not set pin %i for GPIO input.\n", I2C_IRQ_PIN);
	}
	if(at91_set_deglitch(I2C_IRQ_PIN, 1)) {
		printk(KERN_DEBUG"Could not set pin %i for GPIO deglitch.\n", I2C_IRQ_PIN);
	}

	/** Request IRQ for pin */
	// |IRQF_TRIGGER_RISING
	if(request_irq(I2C_IRQ_PIN, CP2120_i2c_irq, IRQF_TRIGGER_NONE, "i2c-CP2120", NULL))  {
		printk(KERN_DEBUG"Can't register IRQ %d\n", I2C_IRQ_PIN);
		return -EIO;
	}
	
	CP2120_spi_msg = kzalloc(256 * sizeof(u8), GFP_KERNEL);
	ret = spi_register_driver(&CP2120_driver);
	if(ret) {
		kfree(CP2120_chips);
		return ret;
	}

	ret = i2c_add_adapter(&CP2120_adapter);
	if (ret)
		kfree(CP2120_chips);
	return ret;
}

static void __exit i2c_CP2120_exit(void)
{
	i2c_del_adapter(&CP2120_adapter);
	spi_unregister_driver(&CP2120_driver);
	tasklet_kill(&CP2120_irq_tasklet);
	free_irq(I2C_IRQ_PIN,0);
	kfree(CP2120_spi_msg);
	kfree(CP2120_chips);
}

MODULE_AUTHOR("Amit B <ab719k@gmail.com>");
MODULE_DESCRIPTION("I2C driver for CP2120 SPI to I2C bridge");
MODULE_LICENSE("GPL");

module_init(i2c_CP2120_init);
module_exit(i2c_CP2120_exit);
