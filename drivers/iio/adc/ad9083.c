// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for AD9083
 *
 * Copyright 2021 Analog Devices Inc.
 */
#include <linux/iio/iio.h>
#include <linux/module.h>
#include "cf_axi_adc.h"
#include <linux/jesd204/jesd204.h>
#include <linux/jesd204/jesd204-of.h>
#include "ad9083/adi_ad9083.h"

#define IN_OUT_BUFF_SZ		3
#define MAX_REG_ADDR		0x1000
#define CHIPID_AD9083		0x00EA
#define CHIPID_MASK		0xFFFF

/** struct ad9083_jesd204_priv - Private data to be returned to the driver
 * @phy:			ad9083 phy
 */
struct ad9083_jesd204_priv {
	struct ad9083_phy *phy;
};

/**
 * struct ad9083_phy - Physical layer
 * @adi_ad9083:			Device structure
 * @chip_info:			Chip info and channels descritions
 * @jdev:			JESD204 device
 * @jesd204_link:		JESD204 link configuration settings
 * @jesd_param:			Defines JESD Parameters
 * @vmax:			Full scale valtage
 * @fc:				Cut-off frequency of low-pass filter
 * @rterm:			Termination resistor: 100Ohm, 200Ohm, open
 * @en_hp:			Enable/disable high perfermance
 * @backoff:			The backoff in terms of noiseterms of noise, 100 * dB
 * @finmax:			Max input
 * @nco_freq_hz:		NCO frequency
 * @decimation:			Decimation config
 * @nco0_datapath_mode:		NCO data path
 * @sampling_frequency_hz:	Sampling frequency of the device per channel
 */
struct ad9083_phy {
	adi_ad9083_device_t	adi_ad9083;
	struct axiadc_chip_info	chip_info;
	struct jesd204_dev	*jdev;
	struct jesd204_link	jesd204_link;
	adi_cms_jesd_param_t	jesd_param;
	u32 vmax;
	u64 fc;
	u32 rterm;
	u32 en_hp;
	u32 backoff;
	u64 finmax;
	u64 nco_freq_hz[3];
	u8 decimation[4];
	u8 nco0_datapath_mode;
	u64 sampling_frequency_hz;
};

static int ad9083_udelay(void *user_data, unsigned int us)
{
	usleep_range(us, (us * 110) / 100);

	return 0;
}

static int ad9083_log_write(void *user_data, s32 log_type, const char *message,
			va_list argp)
{
	struct axiadc_converter *conv = user_data;
	char logMessage[160];

	vsnprintf(logMessage, sizeof(logMessage), message, argp);

	switch (log_type) {
	case ADI_CMS_LOG_NONE:
		break;
	case ADI_CMS_LOG_MSG:
		dev_dbg(&conv->spi->dev, "%s", logMessage);
		break;
	case ADI_CMS_LOG_WARN:
		dev_dbg(&conv->spi->dev, "%s", logMessage);
		break;
	case ADI_CMS_LOG_ERR:
		dev_dbg(&conv->spi->dev, "%s", logMessage);
		break;
	case ADI_CMS_LOG_SPI:
		break;
	case ADI_CMS_LOG_API:
		dev_dbg(&conv->spi->dev, "%s", logMessage);
		break;
	case ADI_CMS_LOG_ALL:
		dev_dbg(&conv->spi->dev, "%s", logMessage);
		break;
	}

	return 0;
}

static int ad9083_spi_xfer(void *user_data, u8 *wbuf,
			u8 *rbuf, u32 len)
{
	struct axiadc_converter *conv = user_data;
	int ret;
	struct spi_transfer t = {
		.tx_buf = wbuf,
		.rx_buf = rbuf,
		.len = len,
	};

	ret = spi_sync_transfer(conv->spi, &t, 1);

	dev_dbg(&conv->spi->dev, "%s: reg=0x%X, val=0x%X",
		(wbuf[0] & 0x80) ? "rd" : "wr",
		(wbuf[0] & 0x7F) << 8 | wbuf[1],
		(wbuf[0] & 0x80) ? rbuf[2] : wbuf[2]);

	return ret;
}

int ad9083_register_write(adi_ad9083_device_t *h,
			const u16 address, const u8 data)
{
	int ret;
	u8 inData[IN_OUT_BUFF_SZ];
	u8 outData[IN_OUT_BUFF_SZ];

	if (address < MAX_REG_ADDR) {
		inData[0] = address >> 8;
		inData[1] = address;
		inData[2] = data;
		ret = ad9083_spi_xfer(h->hal_info.user_data, inData, outData, IN_OUT_BUFF_SZ);
		if (ret != 0)
			return ret;
	}

	return 0;
}

int ad9083_register_read(adi_ad9083_device_t *h,
			const u16 address, u8 *data)
{
	int ret;
	u8 inData[IN_OUT_BUFF_SZ];
	u8 outData[IN_OUT_BUFF_SZ];

	if (address < MAX_REG_ADDR) {
		inData[0] = (address >> 8) | 0x80;
		inData[1] = address;
		ret = ad9083_spi_xfer(h->hal_info.user_data, inData, outData, IN_OUT_BUFF_SZ);
		if (ret != 0)
			return ret;

		*data = outData[2];
	} else
		return -EINVAL;

	return 0;
}

static int ad9083_reg_access(struct iio_dev *indio_dev, unsigned int reg,
		unsigned int writeval, unsigned int *readval)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9083_phy *phy = conv->phy;
	int ret;
	u8 val;

	if (!readval)
		return ad9083_register_write(&phy->adi_ad9083, reg, writeval);

	ret = ad9083_register_read(&phy->adi_ad9083, reg, &val);
	if (ret < 0)
		return ret;

	*readval = val;

	return 0;
}

static int ad9083_jesd204_link_init(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9083_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9083_phy *phy = priv->phy;
	struct jesd204_link *link;

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__,
		__LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	link = &phy->jesd204_link;

	jesd204_copy_link_params(lnk, link);

	lnk->jesd_encoder = JESD204_ENCODER_8B10B;
	lnk->sample_rate = phy->sampling_frequency_hz;
	lnk->sample_rate_div = 1;
	lnk->link_id = 0;

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9083_jesd204_clks_enable(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__,
		__LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9083_jesd204_link_enable(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__,
		 __LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	return JESD204_STATE_CHANGE_DONE;
}

static const struct jesd204_dev_data jesd204_ad9083_init = {
	.state_ops = {
		[JESD204_OP_LINK_INIT] = {
			.per_link = ad9083_jesd204_link_init,
		},
		[JESD204_OP_CLOCKS_ENABLE] = {
			.per_link = ad9083_jesd204_clks_enable,
		},
		[JESD204_OP_LINK_ENABLE] = {
			.per_link = ad9083_jesd204_link_enable,
			.post_state_sysref = true,
		},
	},

	.max_num_links = 1,
	.num_retries = 3,
	.sizeof_priv = sizeof(struct ad9083_jesd204_priv),
};

static int ad9083_request_clks(struct axiadc_converter *conv)
{
	conv->clk = devm_clk_get(&conv->spi->dev, "adc_ref_clk");
	if (IS_ERR(conv->clk))
		return PTR_ERR(conv->clk);

	return 0;
}

static int ad9083_setup(struct axiadc_converter *conv)
{
	struct ad9083_phy *phy = conv->phy;
	adi_cms_chip_id_t chip_id;
	int ret;

	ret = ad9083_request_clks(conv);
	if (ret)
		return ret;

	ret = adi_ad9083_device_chip_id_get(&phy->adi_ad9083, &chip_id);
	if (ret < 0)
		return ret;

	if ((chip_id.prod_id & CHIPID_MASK) != CHIPID_AD9083)
		return -ENOENT;

	/* software reset, resistor is not mounted */
	ret = adi_ad9083_device_reset(&phy->adi_ad9083, AD9083_SOFT_RESET);
	if (ret < 0)
		return ret;

	ret = adi_ad9083_device_init(&phy->adi_ad9083);
	if (ret < 0)
		return ret;

	ret = adi_ad9083_device_clock_config_set(&phy->adi_ad9083,
			phy->sampling_frequency_hz * phy->jesd204_link.num_converters,
			clk_get_rate(conv->clk));
	if (ret < 0)
		return ret;

	ret = adi_ad9083_rx_adc_config_set(&phy->adi_ad9083, phy->vmax, phy->fc,
					   phy->rterm, phy->en_hp, phy->backoff, phy->finmax);
	if (ret < 0)
		return ret;

	ret = adi_ad9083_rx_datapath_config_set(&phy->adi_ad9083,
						phy->nco0_datapath_mode, phy->decimation, phy->nco_freq_hz);
	if (ret < 0)
		return ret;

	ret = adi_ad9083_jtx_startup(&phy->adi_ad9083, &phy->jesd_param);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad9083_parse_dt(struct ad9083_phy *phy, struct device *dev)
{
	struct device_node *np = dev->of_node;

	/* AD9083 Config */
	of_property_read_u64(np, "adi,sampling-frequency",
			     &phy->sampling_frequency_hz);
	of_property_read_u32(np, "adi,vmax-microvolt", &phy->vmax);
	of_property_read_u64(np, "adi,fc", &phy->fc);
	of_property_read_u32(np, "adi,rterm", &phy->rterm);
	of_property_read_u32(np, "adi,en_hp", &phy->en_hp);
	of_property_read_u32(np, "adi,backoff", &phy->backoff);
	of_property_read_u64(np, "adi,finmax-hz", &phy->finmax);
	of_property_read_u64_array(np,
				   "adi,nco_freq-hz",
				   phy->nco_freq_hz,
				   ARRAY_SIZE(phy->nco_freq_hz));
	of_property_read_u8_array(np,
				   "adi,decimation",
				   phy->decimation,
				   ARRAY_SIZE(phy->decimation));
	of_property_read_u8(np, "adi,nco0_datapath_mode", &phy->nco0_datapath_mode);

	/* JESD Link Config */
	memset(&phy->jesd_param, 0, sizeof(adi_cms_jesd_param_t));

	phy->jesd_param.jesd_s = 1;
	phy->jesd_param.jesd_hd = 1;
	phy->jesd_param.jesd_scr = 1;

	JESD204_LNK_READ_NUM_LANES(dev, np, &phy->jesd204_link,
				&phy->jesd_param.jesd_l, 4);

	JESD204_LNK_READ_OCTETS_PER_FRAME(dev, np, &phy->jesd204_link,
					&phy->jesd_param.jesd_f, 8);

	JESD204_LNK_READ_FRAMES_PER_MULTIFRAME(dev, np, &phy->jesd204_link,
						&phy->jesd_param.jesd_k, 32);

	JESD204_LNK_READ_CONVERTER_RESOLUTION(dev, np, &phy->jesd204_link,
					      &phy->jesd_param.jesd_n, 16);

	JESD204_LNK_READ_BITS_PER_SAMPLE(dev, np, &phy->jesd204_link,
					 &phy->jesd_param.jesd_np, 16);

	JESD204_LNK_READ_NUM_CONVERTERS(dev, np, &phy->jesd204_link,
					&phy->jesd_param.jesd_m, 16);

	JESD204_LNK_READ_SUBCLASS(dev, np, &phy->jesd204_link,
				  &phy->jesd_param.jesd_subclass, JESD_SUBCLASS_0);

	return 0;
}

#define AIM_CHAN(_chan, _mod, _si, _bits, _sign)			\
	{ 								\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.modified = 1,						\
		.channel = _chan,					\
		.channel2 = _mod,					\
		.scan_index = _si,					\
		.scan_type = {						\
			.sign = _sign,					\
			.realbits = _bits,				\
			.storagebits = 16,				\
			.shift = 0,					\
		},							\
	}

static struct axiadc_chip_info axiadc_chip_info_tbl = {
	.name = "AD9083",
	.max_rate = 2000000000,
	.num_channels = 16,
	.channel[0] = AIM_CHAN(0, IIO_MOD_I, 0, 16, 'S'),
	.channel[1] = AIM_CHAN(0, IIO_MOD_Q, 1, 16, 'S'),
	.channel[2] = AIM_CHAN(1, IIO_MOD_I, 2, 16, 'S'),
	.channel[3] = AIM_CHAN(1, IIO_MOD_Q, 3, 16, 'S'),
	.channel[4] = AIM_CHAN(2, IIO_MOD_I, 4, 16, 'S'),
	.channel[5] = AIM_CHAN(2, IIO_MOD_Q, 5, 16, 'S'),
	.channel[6] = AIM_CHAN(3, IIO_MOD_I, 6, 16, 'S'),
	.channel[7] = AIM_CHAN(3, IIO_MOD_Q, 7, 16, 'S'),
	.channel[8] = AIM_CHAN(4, IIO_MOD_I, 8, 16, 'S'),
	.channel[9] = AIM_CHAN(4, IIO_MOD_Q, 9, 16, 'S'),
	.channel[10] = AIM_CHAN(5, IIO_MOD_I, 10, 16, 'S'),
	.channel[11] = AIM_CHAN(5, IIO_MOD_Q, 11, 16, 'S'),
	.channel[12] = AIM_CHAN(6, IIO_MOD_I, 12, 16, 'S'),
	.channel[13] = AIM_CHAN(6, IIO_MOD_Q, 13, 16, 'S'),
	.channel[14] = AIM_CHAN(7, IIO_MOD_I, 14, 16, 'S'),
	.channel[15] = AIM_CHAN(7, IIO_MOD_Q, 15, 16, 'S'),
};

static const struct iio_info ad9083_iio_info = {
	.debugfs_reg_access = &ad9083_reg_access,
};

static int ad9083_probe(struct spi_device *spi)
{
	struct axiadc_converter *conv;
	struct ad9083_phy *phy;
	struct jesd204_dev *jdev;
	struct ad9083_jesd204_priv *priv;
	int ret;

	jdev = devm_jesd204_dev_register(&spi->dev, &jesd204_ad9083_init);
	if (IS_ERR(jdev))
		return PTR_ERR(jdev);

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (!conv)
		return -ENOMEM;

	phy = devm_kzalloc(&spi->dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	spi_set_drvdata(spi, conv);

	conv->adc_clkscale.mult = 1;
	conv->adc_clkscale.div = 1;
	conv->spi = spi;
	conv->phy = phy;
	conv->chip_info = &axiadc_chip_info_tbl;
	conv->reg_access = ad9083_reg_access;

	if (jdev) {
		phy->jdev = jdev;
		priv = jesd204_dev_priv(jdev);
		priv->phy = phy;
	}

	phy->adi_ad9083.hal_info.user_data = conv;
	phy->adi_ad9083.hal_info.spi_xfer = ad9083_spi_xfer;
	phy->adi_ad9083.hal_info.delay_us = ad9083_udelay;
	phy->adi_ad9083.hal_info.sdo = SPI_SDIO;
	phy->adi_ad9083.hal_info.msb = SPI_MSB_FIRST;
	phy->adi_ad9083.hal_info.addr_inc = SPI_ADDR_INC_AUTO;
	phy->adi_ad9083.hal_info.log_write = ad9083_log_write;

	ret = ad9083_parse_dt(phy, &spi->dev);
	if (ret < 0) {
		dev_err(&spi->dev, "Parsing devicetree failed (%d)\n", ret);
		return -ENODEV;
	}

	ret = ad9083_setup(conv);
	if (ret < 0) {
		dev_err(&spi->dev, "ad9083_setup failed(%d)\n", ret);
		return -ENODEV;
	}

	return jesd204_fsm_start(jdev, JESD204_LINKS_ALL);
}

static const struct spi_device_id ad9083_id[] = {
	{ "ad9083", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, ad9083_id);

static const struct of_device_id ad9083_of_match[] = {
	{ .compatible = "adi,ad9083" },
	{}
};
MODULE_DEVICE_TABLE(of, ad9083_of_match);

static struct spi_driver ad9083_driver = {
	.driver = {
			.name = "ad9083",
			.of_match_table = ad9083_of_match,
		},
	.probe = ad9083_probe,
	.id_table = ad9083_id,
};
module_spi_driver(ad9083_driver);

MODULE_AUTHOR("Cristian Pop <cristian.pop@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9083 ADC");
MODULE_LICENSE("GPL v2");
