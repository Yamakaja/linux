// SPDX-License-Identifier: GPL-2.0
/*
 * LDPC BER Tester Driver
 *
 * Copyright 2021 David Winter
 *
 * This driver complements the HDL module at
 * https://github.com/Yamakaja/ldpc_ber_tester.git
 *
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/fpga/adi-axi-common.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/sysfs.h>

/* Register map */

/* Core information / basics */
#define AXI_BER_REG_VERSION		0x0000
#define AXI_BER_REG_CORE_ID		0x0004
#define AXI_BER_REG_SCRATCH		0x0008
#define AXI_BER_REG_MAGIC		0x000c
#define AXI_BER_REG_MAGIC_VAL		0x4350444c /* LDPC */

/* Control registers */
#define AXI_BER_REG_CONTROL		0x0040
#define AXI_BER_BIT_CONTROL_ENABLE	BIT(0)
#define AXI_BER_BIT_CONTROL_SW_RESETN	BIT(1)
#define AXI_BER_REG_AWGN_CFG		0x0044
#define AXI_BER_BIT_AWGN_CFG_FACTOR	GENMASK(15, 0)
#define AXI_BER_BIT_AWGN_CFG_OFFSET	GENMASK(23, 16)
#define AXI_BER_REG_DIN_BEATS		0x0048
#define AXI_BER_REG_SDFEC_CTRL_WORD	0x004c

#define AXI_BER_REG_LAST_MASK_LSB	0x0050

#define AXI_BER_REG_INT_ENABLE		0x0060
#define AXI_BER_REG_INT_CLEAR		0x0064
#define AXI_BER_REG_INT_STATUS		0x0068
#define AXI_BER_BIT_INT_LAST_FAILED	BIT(0)

/* Output status registers */
#define AXI_BER_REG_FINISHED_BLOCKS_LSB	0x0080
#define AXI_BER_REG_BIT_ERRORS_LSB	0x0088

/* Debug registers */
#define AXI_BER_REG_IN_FLIGHT		0x0090
#define AXI_BER_REG_LAST_STATUS		0x0094

#define AXI_BER_REG_ITER_COUNT_LSB	0x0098
#define AXI_BER_REG_FAILED_BLOCKS_LSB	0x00a0
#define AXI_BER_REG_LAST_FAILED_LSB	0x00a8

/* Private driver data */

#define BER_FAIL_FIFO_SIZE 1024

/**
 * struct axi_ldpc_ber_tester_state - Driver data for ldpc ber tester
 * @regs: Device base address for MMIO
 * @lock: Secures access to device struct contents except for kfifo
 * @irq: The IRQ ID assigned to this driver
 * @irq_mask: The current interrupt mask configured for this device. Cached to avoid device
 *	accesses.
 * @collect_errors: Whether LAST_FAILED interrupts should be enabled to collect errors. This is
 *	kept track of seperately because the driver will automatically disable
 *	interrupts when the fifo is full, and this value is used to decide if they should be
 *	re-enabled one some space has been freed.
 * @last_failed_fifo: Stores up to BER_FAIL_FIFO_SIZE block IDs of blocks that failed to decode.
 *	These are written in an interrupt handler and read with the mutex 'lock' held in the sysfs
 *	attribute 'last_failed'.
 */
struct axi_ldpc_ber_tester_state {
	void __iomem	*regs;
	struct mutex	lock;
	int		irq;
	u32		irq_mask;
	bool		collect_errors;
	DECLARE_KFIFO(last_failed_fifo, u64, BER_FAIL_FIFO_SIZE);
};

typedef enum {
	AXI_BER_VERSION,
	AXI_BER_CORE_ID,
	AXI_BER_SCRATCH,
	AXI_BER_MAGIC,

	AXI_BER_CONTROL,
	AXI_BER_CONTROL_EN,
	AXI_BER_CONTROL_RESETN,
	AXI_BER_AWGN_CFG,
	AXI_BER_AWGN_CFG_FACTOR,
	AXI_BER_AWGN_CFG_OFFSET,
	AXI_BER_DIN_BEATS,
	AXI_BER_SDFEC_CTRL_WORD,
	AXI_BER_LAST_MASK,
	AXI_BER_INT_ENABLE,
	AXI_BER_INT_CLEAR,
	AXI_BER_INT_STATUS,

	AXI_BER_FINISHED_BLOCKS,
	AXI_BER_BIT_ERRORS,

	AXI_BER_IN_FLIGHT,
	AXI_BER_LAST_STATUS,
	AXI_BER_ITER_COUNT,
	AXI_BER_FAILED_BLOCKS,
	AXI_BER_LAST_FAILED,

	AXI_BER_COLLECT_ERRORS
} ber_attribute_id;

static inline unsigned int ber_read(struct axi_ldpc_ber_tester_state *st, const u32 reg)
{
	return ioread32(st->regs + reg);
}

static inline void ber_write(struct axi_ldpc_ber_tester_state *st, const u32 reg, const u32 val)
{
	iowrite32(val, st->regs + reg);
}

struct ber_attribute {
	struct device_attribute attr;
	ber_attribute_id id;
};

static void axi_ldpc_ber_tester_int_enable(struct axi_ldpc_ber_tester_state *st,
						  bool last_failed)
{
	st->irq_mask = AXI_BER_BIT_INT_LAST_FAILED * last_failed;
	ber_write(st, AXI_BER_REG_INT_ENABLE, st->irq_mask);
}


#define to_ber_attribute(x) container_of(x, struct ber_attribute, attr)

struct ber_register_mapping_element {
	u8	reg;
	u8	type;
	u8	offset;
	u8	length;
};

#define BER_REG_TYPE_HEX	BIT(0)
#define BER_REG_TYPE_LONG	BIT(1)
#define BER_REG_TYPE_QUAD_WORD	BIT(2)
#define BER_REG_TYPE_SUB	BIT(3)
#define BER_REG_TYPE_FIFO	BIT(4)
#define BER_REG_TYPE_PHONY	BIT(5)

static const struct ber_register_mapping_element ber_register_mapping[] = {
	[AXI_BER_VERSION]	= { .reg = AXI_BER_REG_VERSION,	.type = BER_REG_TYPE_HEX },
	[AXI_BER_CORE_ID]	= { .reg = AXI_BER_REG_CORE_ID,	.type = BER_REG_TYPE_HEX },
	[AXI_BER_SCRATCH]	= { .reg = AXI_BER_REG_SCRATCH,	.type = BER_REG_TYPE_HEX },
	[AXI_BER_MAGIC]		= { .reg = AXI_BER_REG_MAGIC,	.type = BER_REG_TYPE_HEX },
	[AXI_BER_CONTROL]	= { .reg = AXI_BER_REG_CONTROL,	.type = BER_REG_TYPE_HEX },
	[AXI_BER_CONTROL_EN] = {
		.reg	= AXI_BER_REG_CONTROL,
		.type	= BER_REG_TYPE_SUB,
		.offset	= 0,
		.length	= 1
	},
	[AXI_BER_CONTROL_RESETN] = {
		.reg	= AXI_BER_REG_CONTROL,
		.type	= BER_REG_TYPE_SUB,
		.offset	= 1,
		.length	= 1
	},
	[AXI_BER_AWGN_CFG]	= { .reg = AXI_BER_REG_AWGN_CFG, .type = BER_REG_TYPE_HEX },
	[AXI_BER_AWGN_CFG_FACTOR] = {
		.reg	= AXI_BER_REG_AWGN_CFG,
		.type	= BER_REG_TYPE_HEX | BER_REG_TYPE_SUB,
		.offset = 0,
		.length = 16
	},
	[AXI_BER_AWGN_CFG_OFFSET] = {
		.reg	= AXI_BER_REG_AWGN_CFG,
		.type	= BER_REG_TYPE_HEX | BER_REG_TYPE_SUB,
		.offset = 16,
		.length = 8
	},
	[AXI_BER_DIN_BEATS]	= { .reg = AXI_BER_REG_DIN_BEATS },
	[AXI_BER_SDFEC_CTRL_WORD] = {
		.reg	= AXI_BER_REG_SDFEC_CTRL_WORD,
		.type	= BER_REG_TYPE_HEX
	},
	[AXI_BER_LAST_MASK] = {
		.reg	= AXI_BER_REG_LAST_MASK_LSB,
		.type	= BER_REG_TYPE_HEX | BER_REG_TYPE_QUAD_WORD
	},
	[AXI_BER_INT_ENABLE] = { .reg = AXI_BER_REG_INT_ENABLE, .type = BER_REG_TYPE_HEX },
	[AXI_BER_INT_CLEAR] = { .reg = AXI_BER_REG_INT_CLEAR, .type = BER_REG_TYPE_HEX },
	[AXI_BER_INT_STATUS] = { .reg = AXI_BER_REG_INT_STATUS, .type = BER_REG_TYPE_HEX },
	[AXI_BER_FINISHED_BLOCKS] = {
		.reg	= AXI_BER_REG_FINISHED_BLOCKS_LSB,
		.type	= BER_REG_TYPE_LONG
	},
	[AXI_BER_BIT_ERRORS] = { .reg = AXI_BER_REG_BIT_ERRORS_LSB, .type = BER_REG_TYPE_LONG },
	[AXI_BER_IN_FLIGHT]	= { .reg = AXI_BER_REG_IN_FLIGHT },
	[AXI_BER_LAST_STATUS]	= { .reg = AXI_BER_REG_LAST_STATUS, .type = BER_REG_TYPE_HEX },
	[AXI_BER_ITER_COUNT]	= { .reg = AXI_BER_REG_ITER_COUNT_LSB, .type = BER_REG_TYPE_LONG },
	[AXI_BER_FAILED_BLOCKS]	= {
		.reg	= AXI_BER_REG_FAILED_BLOCKS_LSB,
		.type	= BER_REG_TYPE_LONG
	},
	[AXI_BER_LAST_FAILED] = {
		.reg	= AXI_BER_REG_LAST_FAILED_LSB,
		.type	= BER_REG_TYPE_LONG | BER_REG_TYPE_FIFO
	},
	[AXI_BER_COLLECT_ERRORS] = { .type = BER_REG_TYPE_PHONY }

};

static ssize_t axi_ldpc_ber_tester_show(struct device *dev,
					struct device_attribute *dev_attr,
					char *buf)
{
	struct axi_ldpc_ber_tester_state *st = dev_get_drvdata(dev);
	const struct ber_attribute *attr = to_ber_attribute(dev_attr);
	const struct ber_register_mapping_element *map = &ber_register_mapping[attr->id];
	u32 i;
	int ret;
	u32 data[4];
	u64 long_data;
	u64 last_failed[32];
	int written = 0;

	mutex_lock(&st->lock);

	switch (map->type) {
	case 0:
	case BER_REG_TYPE_HEX:
		data[0] = ber_read(st, map->reg);
		ret = sysfs_emit(buf, map->type ? "0x%08x\n" : "%u\n", data[0]);
		break;
	case BER_REG_TYPE_LONG:
	case BER_REG_TYPE_LONG | BER_REG_TYPE_HEX:
		long_data = ber_read(st, map->reg) | ((u64)(ber_read(st, map->reg+4)) << 32);
		ret = sysfs_emit(buf, map->type == BER_REG_TYPE_LONG ? "%llu\n" : "0x%016llx\n",
				 long_data);
		break;
	case BER_REG_TYPE_SUB:
	case BER_REG_TYPE_SUB | BER_REG_TYPE_HEX:
		data[0] = (ber_read(st, map->reg) >> (map->offset)) & ((1U << map->length) - 1);
		ret = sysfs_emit(buf, map->type == BER_REG_TYPE_SUB ? "%u\n" : "0x%08x\n", data[0]);
		break;
	case BER_REG_TYPE_QUAD_WORD | BER_REG_TYPE_HEX:
		for (i = 0; i < 4; i++)
			data[i] = ber_read(st, map->reg + i*4);

		ret = sysfs_emit(buf, "0x%08x 0x%08x 0x%08x 0x%08x\n",
				 data[0], data[1], data[2], data[3]);
		break;
	case BER_REG_TYPE_FIFO | BER_REG_TYPE_LONG:
		ret = kfifo_out_peek(&st->last_failed_fifo, &last_failed[0], ARRAY_SIZE(last_failed));
		for (i = 0; i < ret && PAGE_SIZE - written > 20; i++)
			written += scnprintf(&buf[written], PAGE_SIZE-written, "%llu\n",
					     last_failed[i]);
		ret = written;
		break;
	case BER_REG_TYPE_PHONY:
		switch (attr->id) {
		case AXI_BER_COLLECT_ERRORS:
			ret = sysfs_emit(buf, "%d\n", (int) st->collect_errors);
			break;
		default:
			ret = - EINVAL;
			break;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	};

	mutex_unlock(&st->lock);
	return ret;
}

static ssize_t axi_ldpc_ber_tester_store(struct device *dev,
					 struct device_attribute *dev_attr,
					 const char *buf,
					 size_t count)
{
	struct axi_ldpc_ber_tester_state *st = dev_get_drvdata(dev);
	const struct ber_attribute *attr = to_ber_attribute(dev_attr);
	const struct ber_register_mapping_element *map = &ber_register_mapping[attr->id];
	u32 i;
	int ret;
	u32 data[4];
	u32 len;
	u32 mask;
	u64 long_data;
	const char *ptr;
	u8 stack_buffer[64];

	mutex_lock(&st->lock);

	switch (map->type) {
	case 0:
	case BER_REG_TYPE_HEX:
		ret = kstrtou32(buf, 0, &data[0]);
		if (ret) {
			ret = -EINVAL;
			break;
		}

		ber_write(st, map->reg, data[0]);
		ret = count;
		break;
	case BER_REG_TYPE_LONG:
	case BER_REG_TYPE_LONG | BER_REG_TYPE_HEX:
		ret = kstrtou64(buf, 0, &long_data);
		if (ret) {
			ret = -EINVAL;
			break;
		}

		ber_write(st, map->reg, (u32) (long_data & 0xFFFFFFFFULL));
		ber_write(st, map->reg + 4, (u32) (long_data >> 32));
		ret = count;
		break;
	case BER_REG_TYPE_SUB:
	case BER_REG_TYPE_SUB | BER_REG_TYPE_HEX:
		ret = kstrtou32(buf, 0, &data[0]);
		if (ret) {
			ret = -EINVAL;
			break;
		}

		data[1] = ber_read(st, map->reg);
		mask = ((1UL << map->length) - 1) << map->offset;

		ber_write(st, map->reg, (mask & (data[0] << map->offset)) | (data[1] & ~mask));
		ret = count;
		break;
	case BER_REG_TYPE_QUAD_WORD | BER_REG_TYPE_HEX:
		ptr = buf;
		for (i = 0; i < 4; i++) {
			ptr = strpbrk(buf, " \n");
			if (ptr == NULL || ptr - buf >= sizeof(stack_buffer)) {
				ret = -EINVAL;
				break;
			}

			memcpy(stack_buffer, buf, ptr - buf);
			stack_buffer[ptr - buf] = '\0';

			ret = kstrtou32(stack_buffer, 0, &data[i]);
			if (ret) {
				ret = -EINVAL;
				break;
			}

			buf = ptr+1;
		}

		if (ret)
			break;

		for (i = 0; i < 4; i++)
			ber_write(st, map->reg + 4*i, data[i]);

		ret = count;
		break;
	case BER_REG_TYPE_FIFO | BER_REG_TYPE_LONG:
		ret = kstrtou32(buf, 0, &data[0]);
		if (ret) {
			ret = -EINVAL;
			break;
		}
		len = kfifo_len(&st->last_failed_fifo);
		if (data[0] < len) {
			len = data[0];
			for (i = 0; i < len; i++)
				kfifo_skip(&st->last_failed_fifo);
		} else
			kfifo_reset_out(&st->last_failed_fifo);

		if (len && !(st->irq_mask & AXI_BER_BIT_INT_LAST_FAILED) && st->collect_errors)
			axi_ldpc_ber_tester_int_enable(st, true);

		ret = count;
		break;

	case BER_REG_TYPE_PHONY:
		switch (attr->id) {
		case AXI_BER_COLLECT_ERRORS:
			ret = kstrtou32(buf, 0, &data[0]);
			if (ret) {
				ret = -EINVAL;
				break;
			}
			data[0] = !!data[0];

			if (data[0] != st->collect_errors)
				axi_ldpc_ber_tester_int_enable(st, data[0]);

			st->collect_errors = data[0];

			ret = count;
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	};

	mutex_unlock(&st->lock);
	return ret;
}

#define __BER_ATTR(_name, _id, _mode) {								\
	.attr = __ATTR(_name, _mode, axi_ldpc_ber_tester_show, axi_ldpc_ber_tester_store),	\
	.id = _id										\
}
#define BER_ATTR(_name, _id, _mode)								\
	struct ber_attribute dev_attr_##_name = __BER_ATTR(_name, _id, _mode)

static BER_ATTR(version, AXI_BER_VERSION, S_IRUGO);
static BER_ATTR(core_id, AXI_BER_CORE_ID, S_IRUGO);
static BER_ATTR(scratch, AXI_BER_SCRATCH, S_IWUSR | S_IRUGO);
static BER_ATTR(magic, AXI_BER_MAGIC, S_IRUGO);
static BER_ATTR(control, AXI_BER_CONTROL, S_IWUSR | S_IRUGO);
static BER_ATTR(control_enable, AXI_BER_CONTROL_EN, S_IWUSR | S_IRUGO);
static BER_ATTR(control_resetn, AXI_BER_CONTROL_RESETN, S_IWUSR | S_IRUGO);
static BER_ATTR(awgn_cfg, AXI_BER_AWGN_CFG, S_IWUSR | S_IRUGO);
static BER_ATTR(awgn_cfg_factor, AXI_BER_AWGN_CFG_FACTOR, S_IWUSR | S_IRUGO);
static BER_ATTR(awgn_cfg_offset, AXI_BER_AWGN_CFG_OFFSET, S_IWUSR | S_IRUGO);
static BER_ATTR(din_beats, AXI_BER_DIN_BEATS, S_IWUSR | S_IRUGO);
static BER_ATTR(sdfec_ctrl_word, AXI_BER_SDFEC_CTRL_WORD, S_IWUSR | S_IRUGO);
static BER_ATTR(last_mask, AXI_BER_LAST_MASK, S_IWUSR | S_IRUGO);
static BER_ATTR(int_enable, AXI_BER_INT_ENABLE, S_IWUSR | S_IRUGO);
static BER_ATTR(int_clear, AXI_BER_INT_CLEAR, S_IWUSR);
static BER_ATTR(int_status, AXI_BER_INT_STATUS, S_IRUGO);
static BER_ATTR(finished_blocks, AXI_BER_FINISHED_BLOCKS, S_IRUGO);
static BER_ATTR(bit_errors, AXI_BER_BIT_ERRORS, S_IRUGO);
static BER_ATTR(in_flight, AXI_BER_IN_FLIGHT, S_IRUGO);
static BER_ATTR(last_status, AXI_BER_LAST_STATUS, S_IRUGO);
static BER_ATTR(iter_count, AXI_BER_ITER_COUNT, S_IRUGO);
static BER_ATTR(failed_blocks, AXI_BER_FAILED_BLOCKS, S_IRUGO);
static BER_ATTR(last_failed, AXI_BER_LAST_FAILED, S_IWUSR | S_IRUGO);
static BER_ATTR(collect_errors, AXI_BER_COLLECT_ERRORS, S_IWUSR | S_IRUGO);

static struct attribute *ldpc_ber_tester_attrs[] = {
	&dev_attr_version.attr.attr,
	&dev_attr_core_id.attr.attr,
	&dev_attr_scratch.attr.attr,
	&dev_attr_magic.attr.attr,
	&dev_attr_control.attr.attr,
	&dev_attr_control_enable.attr.attr,
	&dev_attr_control_resetn.attr.attr,
	&dev_attr_awgn_cfg.attr.attr,
	&dev_attr_awgn_cfg_factor.attr.attr,
	&dev_attr_awgn_cfg_offset.attr.attr,
	&dev_attr_din_beats.attr.attr,
	&dev_attr_sdfec_ctrl_word.attr.attr,
	&dev_attr_last_mask.attr.attr,
	&dev_attr_int_enable.attr.attr,
	&dev_attr_int_clear.attr.attr,
	&dev_attr_int_status.attr.attr,
	&dev_attr_finished_blocks.attr.attr,
	&dev_attr_bit_errors.attr.attr,
	&dev_attr_in_flight.attr.attr,
	&dev_attr_last_status.attr.attr,
	&dev_attr_iter_count.attr.attr,
	&dev_attr_failed_blocks.attr.attr,
	&dev_attr_last_failed.attr.attr,
	&dev_attr_collect_errors.attr.attr,
	NULL
};
ATTRIBUTE_GROUPS(ldpc_ber_tester);

static irqreturn_t axi_ldpc_ber_tester_irq_thread(int irq, void *dev_id)
{
	struct axi_ldpc_ber_tester_state *st = dev_id;
	u64 last_failed;
	int ret;

	WARN_ON(st->irq != irq);

	// Read current block index from device
	last_failed = (u64) ber_read(st, AXI_BER_REG_LAST_FAILED_LSB) | (
			((u64) ber_read(st, AXI_BER_REG_LAST_FAILED_LSB+4)) << 32);

	// Clear interrupt, this also allows a new failure to be captured
	ber_write(st, AXI_BER_REG_INT_CLEAR, AXI_BER_BIT_INT_LAST_FAILED);

	// Save captured value to fifo. If fifo is at capacity, disable interrupt
	ret = kfifo_put(&st->last_failed_fifo, last_failed);

	if (!ret) {
		mutex_lock(&st->lock);
		axi_ldpc_ber_tester_int_enable(st, false);
		mutex_unlock(&st->lock);
	}

	return IRQ_HANDLED;
}

static void axi_ldpc_ber_tester_int_disable(void *data)
{
	struct axi_ldpc_ber_tester_state *st = data;

	axi_ldpc_ber_tester_int_enable(st, false);
}

static int axi_ldpc_ber_tester_probe(struct platform_device *pdev)
{
	struct axi_ldpc_ber_tester_state *st;
	u32 magic;
	const u32 *version_info;
	u32 actual_version;
	int ret;

	version_info = of_device_get_match_data(&pdev->dev);
	if (!version_info) {
		dev_err(&pdev->dev, "Failed to probe device with no matching version info?!\n");
		return -ENODEV;
	}

	st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	mutex_init(&st->lock);
	dev_set_drvdata(&pdev->dev, st);

	st->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);

	st->irq = platform_get_irq(pdev, 0);
	if (st->irq < 0) {
		dev_err(&pdev->dev, "Failed to acquire irq!\n");
		return -ENODEV;
	}

	magic = ber_read(st, AXI_BER_REG_MAGIC);
	if (magic != AXI_BER_REG_MAGIC_VAL) {
		dev_err(&pdev->dev, "Device has invalid magic identifier, expected 0x%08x, got 0x%08x\n",
				AXI_BER_REG_MAGIC_VAL, magic);
		return -ENODEV;
	}

	actual_version = ber_read(st, AXI_BER_REG_VERSION);
	if (actual_version != *version_info) {
		dev_err(&pdev->dev, "Unexpected core version!"
			"Expected %d.%.2d.%c, found %d.%.2d.%c.\n",
			ADI_AXI_PCORE_VER_MAJOR(*version_info),
			ADI_AXI_PCORE_VER_MINOR(*version_info),
			ADI_AXI_PCORE_VER_PATCH(*version_info),
			ADI_AXI_PCORE_VER_MAJOR(actual_version),
			ADI_AXI_PCORE_VER_MINOR(actual_version),
			ADI_AXI_PCORE_VER_PATCH(actual_version));
		return -ENODEV;
	}

	INIT_KFIFO(st->last_failed_fifo);

	ret = devm_request_threaded_irq(&pdev->dev, st->irq, NULL,
					axi_ldpc_ber_tester_irq_thread,
					IRQF_ONESHOT, NULL, st);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request threaded irq handler: %d\n", ret);
		return -ENODEV;
	}

	axi_ldpc_ber_tester_int_enable(st, true);
	return devm_add_action_or_reset(&pdev->dev, axi_ldpc_ber_tester_int_disable, st);
}

static const u32 ber_versions[] = {
	ADI_AXI_PCORE_VER(1, 0, 'a')
};

static const struct of_device_id axi_ldpc_ber_tester_match[] = {
	{ .compatible = "cel,axi-ldpc-ber-tester-1.00.a", .data = &ber_versions[0] },
	{ /* end of list */ }
};
MODULE_DEVICE_TABLE(of, axi_ldpc_ber_tester_of_match);

static struct platform_driver axi_ldpc_ber_tester_driver = {
	.driver = {
		.name = "axi_ldpc_ber_tester",
		.of_match_table = axi_ldpc_ber_tester_match,
		.dev_groups = ldpc_ber_tester_groups
	},
	.probe = axi_ldpc_ber_tester_probe
};
module_platform_driver(axi_ldpc_ber_tester_driver);

MODULE_AUTHOR("David Winter <dastw@gmx.net>");
MODULE_DESCRIPTION("LDPC BER Tester AXI Driver");
MODULE_LICENSE("GPL v2");
