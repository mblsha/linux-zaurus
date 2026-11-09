// SPDX-License-Identifier: GPL-2.0-only
/*
 * H3970 ASIC3 SD host. Register layout, clock setup and R2 response packing
 * follow the Familiar ASIC3 drivers by SDG Systems and Ian Molton.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/workqueue.h>

#define ASIC3_CLOCK_ENABLE	0x0000
#define ASIC3_CLOCK_SELECT	0x0004
#define ASIC3_SD_HWCTRL		0x0400
#define ASIC3_EXTCF_SELECT	0x0700
#define ASIC3_EX0_EX1		(BIT(13) | BIT(14))
#define ASIC3_SD_CLOCKS		(BIT(9) | BIT(10))
#define ASIC3_SD_HOST_24M	BIT(0)
#define ASIC3_SD_BUS_24M	BIT(1)
#define ASIC3_SD_SUSPEND	BIT(0)
#define ASIC3_SD_CLEAR		BIT(2)
#define ASIC3_SD_CD_WP		(BIT(3) | BIT(4))
#define ASIC3_SD_POWER		BIT(6)
#define ASIC3_SD_MEM_ENABLE	BIT(14)

#define CFG_COMMAND		0x0408
#define CFG_ADDR_LO		0x0420
#define CFG_ADDR_HI		0x0424
#define CFG_CLOCK_GATE		0x0480
#define CFG_CLOCK_MODE		0x0484
#define CFG_POWER		0x0490
#define CFG_AUTO_POWER		0x0494
#define CFG_CARD_DETECT		0x0498
#define SD_COMMAND		0x1000
#define SD_ARG_LO		0x1008
#define SD_ARG_HI		0x100c
#define SD_STOP			0x1010
#define SD_BLOCK_COUNT		0x1014
#define SD_RESPONSE		0x1018
#define SD_STATUS		0x1038
#define SD_BUFFER_STATUS	0x103c
#define SD_STATUS_MASK		0x1040
#define SD_BUFFER_MASK		0x1044
#define SD_CLOCK		0x1048
#define SD_BLOCK_LENGTH		0x104c
#define SD_OPTION		0x1050
#define SD_ERROR0		0x1058
#define SD_ERROR1		0x105c
#define SD_DATA			0x1060
#define SD_RESET		0x11c0
#define SDIO_CLOCK_WAIT		0x1270

#define CMD_APP_PREFIX		BIT(6)
#define CMD_IDLE		(3 << 8)
#define CMD_R1			(4 << 8)
#define CMD_R1B			(5 << 8)
#define CMD_R2			(6 << 8)
#define CMD_R3			(7 << 8)
#define CMD_READ_DATA		(BIT(11) | BIT(12))

#define STATUS_RESPONSE_END	BIT(0)
#define STATUS_DATA_END		BIT(2)
#define BUFFER_CRC_ERROR	BIT(1)
#define BUFFER_DATA_TIMEOUT	BIT(3)
#define BUFFER_CMD_TIMEOUT	BIT(6)
#define BUFFER_READ_READY	BIT(8)
#define BUFFER_BUSY		BIT(14)
#define BUFFER_ERRORS		(GENMASK(6, 0) | BIT(15))
#define SD_CLOCK_ENABLE		BIT(8)
#define SD_CLOCK_DIV_64		BIT(4)
#define SD_CLOCK_DIV_128		BIT(5)

struct h3970_sd {
	void __iomem *regs;
	struct mmc_host *mmc;
	struct work_struct work;
	struct mmc_request *request;
	u16 data[256];
};

static void update16(void __iomem *reg, u16 mask, u16 value)
{
	writew((readw(reg) & ~mask) | value, reg);
}

static void h3970_sd_read_response(void __iomem *regs, struct mmc_command *cmd)
{
	unsigned int i;
	u16 response[8];

	if (!(cmd->flags & MMC_RSP_PRESENT))
		return;
	if (!(cmd->flags & MMC_RSP_136)) {
		cmd->resp[0] = readw(regs + SD_RESPONSE) |
			((u32)readw(regs + SD_RESPONSE + 4) << 16);
		return;
	}

	for (i = 0; i < ARRAY_SIZE(response); ++i)
		response[i] = readw(regs + SD_RESPONSE + 4 * i);
	cmd->resp[0] = (u32)response[7] << 24 | (u32)response[6] << 8 |
		       response[5] >> 8;
	cmd->resp[1] = (u32)response[5] << 24 | (u32)response[4] << 8 |
		       response[3] >> 8;
	cmd->resp[2] = (u32)response[3] << 24 | (u32)response[2] << 8 |
		       response[1] >> 8;
	cmd->resp[3] = (u32)response[1] << 24 | (u32)response[0] << 8 | 0xff;
}

static int h3970_sd_wait(void __iomem *regs, unsigned int reg, u16 mask,
			 u16 errors)
{
	u16 status, buffer;
	int ret;

	ret = read_poll_timeout(readw, status,
				(status & mask) || (readw(regs + SD_BUFFER_STATUS) & errors),
				30, 300000, false, regs + reg);
	buffer = readw(regs + SD_BUFFER_STATUS) & errors;
	if (buffer & (BUFFER_CMD_TIMEOUT | BUFFER_DATA_TIMEOUT))
		return -ETIMEDOUT;
	if (buffer)
		return -EILSEQ;
	return ret;
}

static void h3970_sd_work(struct work_struct *work)
{
	struct h3970_sd *host = container_of(work, struct h3970_sd, work);
	struct mmc_request *mrq = host->request;
	struct mmc_command *cmd = mrq->cmd;
	struct mmc_data *data = mrq->data;
	void __iomem *regs = host->regs;
	u16 command = cmd->opcode, errors = BUFFER_ERRORS, buffer;
	unsigned int i;

	/* Keep the bench card read-only, including direct MMC requests. */
	if ((data && (data->flags & MMC_DATA_WRITE)) ||
	    cmd->opcode == MMC_ERASE || cmd->opcode == MMC_SET_WRITE_PROT ||
	    cmd->opcode == MMC_CLR_WRITE_PROT) {
		cmd->error = -EROFS;
		goto done;
	}
	if (data && (!(data->flags & MMC_DATA_READ) || data->blocks != 1 ||
		     data->blksz < 2 || (data->blksz & 1) ||
		     data->blksz > sizeof(host->data))) {
		cmd->error = -EINVAL;
		goto done;
	}

	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_NONE:
		if (cmd->opcode == MMC_GO_IDLE_STATE)
			command |= CMD_IDLE;
		break;
	case MMC_RSP_R1:
		command |= CMD_R1;
		break;
	case MMC_RSP_R1B:
		command |= CMD_R1B;
		break;
	case MMC_RSP_R2:
		command |= CMD_R2;
		break;
	case MMC_RSP_R3:
		command |= CMD_R3;
		break;
	default:
		cmd->error = -EINVAL;
		goto done;
	}
	if (!(cmd->flags & MMC_RSP_CRC))
		errors &= ~BUFFER_CRC_ERROR;
	if (cmd->opcode == MMC_APP_CMD)
		command |= CMD_APP_PREFIX;

	cmd->error = read_poll_timeout(readw, buffer, !(buffer & BUFFER_BUSY),
				       30, 300000, false, regs + SD_BUFFER_STATUS);
	if (cmd->error)
		goto report;
	writew(0, regs + SD_STATUS);
	writew(0, regs + SD_BUFFER_STATUS);
	writew(0, regs + SD_ERROR0);
	writew(0, regs + SD_ERROR1);
	if (data) {
		writew(1, regs + SD_BLOCK_COUNT);
		writew(data->blksz, regs + SD_BLOCK_LENGTH);
		command |= CMD_READ_DATA;
	}
	writew(cmd->arg >> 16, regs + SD_ARG_HI);
	writew(cmd->arg, regs + SD_ARG_LO);
	writew(command, regs + SD_COMMAND);

	cmd->error = h3970_sd_wait(regs, SD_STATUS, STATUS_RESPONSE_END, errors);
	if (cmd->error)
		goto report;
	writew((u16)~STATUS_RESPONSE_END, regs + SD_STATUS);
	h3970_sd_read_response(regs, cmd);
	if (!data)
		goto done;

	data->error = h3970_sd_wait(regs, SD_BUFFER_STATUS, BUFFER_READ_READY,
				    BUFFER_ERRORS);
	if (data->error)
		goto report;
	writew((u16)~BUFFER_READ_READY, regs + SD_BUFFER_STATUS);
	for (i = 0; i < data->blksz / 2; ++i)
		host->data[i] = readw(regs + SD_DATA);
	data->error = h3970_sd_wait(regs, SD_STATUS, STATUS_DATA_END, BUFFER_ERRORS);
	if (data->error)
		goto report;
	writew((u16)~STATUS_DATA_END, regs + SD_STATUS);
	data->bytes_xfered = sg_copy_from_buffer(data->sg, data->sg_len,
						 host->data, data->blksz);
	if (data->bytes_xfered != data->blksz)
		data->error = -EIO;
	goto done;

report:
	dev_info_ratelimited(mmc_dev(host->mmc),
		"CMD%u arg=%08x error=%d status=%04x/%04x detail=%04x/%04x\n",
		cmd->opcode, cmd->arg, cmd->error ?: data->error,
		readw(regs + SD_STATUS), readw(regs + SD_BUFFER_STATUS),
		readw(regs + SD_ERROR0), readw(regs + SD_ERROR1));
done:
	mmc_request_done(host->mmc, mrq);
}

static void h3970_sd_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct h3970_sd *host = mmc_priv(mmc);

	host->request = mrq;
	schedule_work(&host->work);
}

static void h3970_sd_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct h3970_sd *host = mmc_priv(mmc);
	void __iomem *regs = host->regs;

	writew(0, regs + SD_CLOCK);
	if (ios->clock) {
		/* ASIC2 supplies 24.576 MHz. Stay below 400 kHz for Glasgow. */
		writew(1, regs + CFG_CLOCK_MODE);
		writew(SD_CLOCK_ENABLE | (ios->clock >= 384000 ?
		       SD_CLOCK_DIV_64 : SD_CLOCK_DIV_128), regs + SD_CLOCK);
	}
	/* Keep slot VDD asserted across rescans and kexec. */
}

static int h3970_sd_get_ro(struct mmc_host *mmc)
{
	return 1;
}

static const struct mmc_host_ops h3970_sd_ops = {
	.request = h3970_sd_request,
	.set_ios = h3970_sd_set_ios,
	.get_ro = h3970_sd_get_ro,
};

static void h3970_sd_enable(void __iomem *asic3, void __iomem *regs)
{
	update16(asic3 + ASIC3_SD_HWCTRL,
		 ASIC3_SD_SUSPEND | ASIC3_SD_CLEAR | ASIC3_SD_CD_WP, ASIC3_SD_CD_WP);
	update16(asic3 + ASIC3_CLOCK_ENABLE, ASIC3_EX0_EX1, ASIC3_EX0_EX1);
	usleep_range(1000, 2000);
	update16(asic3 + ASIC3_CLOCK_SELECT, ASIC3_SD_HOST_24M | ASIC3_SD_BUS_24M,
		 ASIC3_SD_HOST_24M);
	update16(asic3 + ASIC3_CLOCK_ENABLE, ASIC3_SD_CLOCKS, ASIC3_SD_CLOCKS);
	usleep_range(1000, 2000);
	update16(asic3 + ASIC3_EXTCF_SELECT, ASIC3_SD_MEM_ENABLE, ASIC3_SD_MEM_ENABLE);
	msleep(500);

	writew(0, regs + CFG_ADDR_HI);
	writew(0x0800, regs + CFG_ADDR_LO);
	writew(0x1f, regs + CFG_CLOCK_GATE);
	writew(2, regs + CFG_CARD_DETECT);
	writew(2, regs + CFG_COMMAND);
	writew(0, regs + SD_RESET);
	usleep_range(2000, 3000);
	writew(1, regs + SD_RESET);
	usleep_range(2000, 3000);
	/* One-bit bus, no C2 module, response timeout exponent 14. */
	writew(0xc0ee, regs + SD_OPTION);
	writew(0, regs + SD_CLOCK);
	writew(0, regs + SD_STOP);
	writew(0xffff, regs + SD_STATUS_MASK);
	writew(0xffff, regs + SD_BUFFER_MASK);
	writew(0x0100, regs + SDIO_CLOCK_WAIT);
	update16(asic3 + ASIC3_SD_HWCTRL, ASIC3_SD_POWER, ASIC3_SD_POWER);
	writew(0, regs + CFG_AUTO_POWER);
	writew(0x0200, regs + CFG_POWER);
	msleep(20);
}

static int h3970_sd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct h3970_sd *host;
	struct mmc_host *mmc;
	struct clk *clock;
	void __iomem *asic3;

	mmc = devm_mmc_alloc_host(dev, sizeof(*host));
	if (!mmc)
		return -ENOMEM;
	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(host->regs))
		return PTR_ERR(host->regs);
	asic3 = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(asic3))
		return PTR_ERR(asic3);
	clock = devm_clk_get_enabled(dev, "sd");
	if (IS_ERR(clock))
		return dev_err_probe(dev, PTR_ERR(clock), "failed to enable SD clock\n");

	INIT_WORK(&host->work, h3970_sd_work);
	mmc->ops = &h3970_sd_ops;
	mmc->caps = MMC_CAP_NONREMOVABLE;
	mmc->caps2 = MMC_CAP2_NO_SDIO | MMC_CAP2_NO_MMC;
	mmc->f_min = 192000;
	mmc->f_max = 384000;
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->max_blk_size = sizeof(host->data);
	mmc->max_blk_count = 1;
	/* The block queue accepts a page. MMC splits it into single CMD17s. */
	mmc->max_req_size = PAGE_SIZE;
	mmc->max_seg_size = PAGE_SIZE;
	mmc->max_segs = PAGE_SIZE / sizeof(host->data);
	platform_set_drvdata(pdev, mmc);

	h3970_sd_enable(asic3, host->regs);
	dev_info(dev, "ASIC3 SD enabled, status=%04x/%04x, slot power=%04x\n",
		 readw(host->regs + SD_STATUS), readw(host->regs + SD_BUFFER_STATUS),
		 readw(host->regs + CFG_POWER));
	return mmc_add_host(mmc);
}

static void h3970_sd_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct h3970_sd *host = mmc_priv(mmc);

	mmc_remove_host(mmc);
	cancel_work_sync(&host->work);
}

static const struct of_device_id h3970_sd_match[] = {
	{ .compatible = "compaq,h3970-asic3-sd" },
	{}
};

MODULE_DEVICE_TABLE(of, h3970_sd_match);

static struct platform_driver h3970_sd_driver = {
	.probe = h3970_sd_probe,
	.remove = h3970_sd_remove,
	.driver = {
		.name = "h3970-asic3-sd",
		.of_match_table = h3970_sd_match,
	},
};
module_platform_driver(h3970_sd_driver);
MODULE_DESCRIPTION("Compaq iPAQ H3970 ASIC3 SD host");
MODULE_LICENSE("GPL");
