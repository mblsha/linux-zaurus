// SPDX-License-Identifier: GPL-2.0-only
/*
 *  linux/drivers/mmc/host/pxa.c - PXA MMCI driver
 *
 *  Copyright (C) 2003 Russell King, All Rights Reserved.
 *
 *  This hardware is really sick:
 *   - No way to clear interrupts.
 *   - Have to turn off the clock whenever we touch the device.
 *   - Doesn't tell you how many data blocks were transferred.
 *  Yuck!
 *
 *	1 and 3 byte data transfers not supported
 *	max block length up to 1023
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/io.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/gfp.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/soc/pxa/cpu.h>
#include <linux/workqueue.h>

#include <linux/sizes.h>

#include <linux/platform_data/mmc-pxamci.h>

#include "pxamci.h"
#include "pxamci-internal.h"

#define DRIVER_NAME	"pxa2xx-mci"

#define PXAMCI_MAX_SEGS	32

#define mmc_has_26MHz()		(cpu_is_pxa300() || cpu_is_pxa310() \
				|| cpu_is_pxa935())

struct pxamci_host;

struct pxamci_dma {
	struct pxamci_host	*host;
	struct mmc_data		*data;
	struct dma_chan		*chan;
	dma_cookie_t		cookie;
	enum dma_data_direction dir;
	unsigned int		request_seq;
	unsigned int		timeout_ms;
	bool			started;
};

struct pxamci_host {
	struct mmc_host		*mmc;
	spinlock_t		lock;
	struct resource		*res;
	void __iomem		*base;
	struct clk		*clk;
	int			irq;
	unsigned long		clkrate;
	unsigned int		clkrt;
	unsigned int		cmdat;
	unsigned int		data_cmdat;
	unsigned int		imask;
	unsigned int		power_mode;
	unsigned long		detect_delay_ms;
	bool			use_ro_gpio;
	bool			pxa25x;
	bool			pxa27x;
	bool			pxa3xx_eject;
	bool			pxa27x_c0;
	bool			pxa27x_e56;
	bool			pxa320_b2;
	bool			sdio_mode;
	bool			fatal_error;
	bool			quarantined;
	bool			card_change_pending;
	bool			irq_requested;
	int			ios_error;
	struct gpio_desc	*power;
	struct pxamci_platform_data *pdata;

	struct mmc_request	*mrq;
	struct mmc_command	*cmd;
	bool			command_activated;
	bool			command_done_pending;
	unsigned int		command_done_stat;
	struct mmc_data		*data;

	struct dma_chan		*dma_chan_rx;
	struct dma_chan		*dma_chan_tx;
	struct pxamci_dma	*dma;
	unsigned int		request_seq;
	bool			dma_done;
	bool			data_done_pending;
	bool			data_finishing;
	bool			data_abort;
	bool			recovery_pending;
	bool			program_needed;
	bool			program_done;
	bool			program_wait;
	struct mmc_command	*program_cmd;
	unsigned int		data_done_stat;
	unsigned long		request_deadline;
	struct delayed_work	data_watchdog;
	struct work_struct	command_work;
	struct work_struct	data_work;
	struct workqueue_struct	*data_wq;
	struct work_struct	request_work;
};

static int pxamci_init_ocr(struct pxamci_host *host)
{
	struct mmc_host *mmc = host->mmc;
	int ret;

	ret = mmc_regulator_get_supply(mmc);
	if (ret < 0)
		return ret;

	if (IS_ERR(mmc->supply.vmmc)) {
		/* fall-back to platform data */
		mmc->ocr_avail = host->pdata ?
			host->pdata->ocr_mask :
			MMC_VDD_32_33 | MMC_VDD_33_34;
	}

	return 0;
}

static inline int pxamci_set_power(struct pxamci_host *host, unsigned int vdd)
{
	struct mmc_host *mmc = host->mmc;
	struct regulator *supply = mmc->supply.vmmc;

	if (!IS_ERR(supply))
		return mmc_regulator_set_ocr(mmc, supply, vdd);

	if (host->power) {
		bool on = !!((1 << vdd) & host->pdata->ocr_mask);

		gpiod_set_value_cansleep(host->power, on);
	}

	if (host->pdata && host->pdata->setpower)
		return host->pdata->setpower(mmc_dev(host->mmc), vdd);

	return 0;
}

static void pxamci_disable_functional_clock(struct pxamci_host *host)
{
	unsigned int clkrt = host->clkrt;

	host->clkrt = PXAMCI_CLKRT_OFF;
	host->mmc->actual_clock = 0;
	if (clkrt != PXAMCI_CLKRT_OFF)
		clk_disable_unprepare(host->clk);
}

static int pxamci_enable_functional_clock(struct pxamci_host *host)
{
	int ret;

	if (host->clkrt != PXAMCI_CLKRT_OFF)
		return 0;

	ret = clk_prepare_enable(host->clk);
	if (!ret)
		host->clkrt = 0;

	return ret;
}

static void pxamci_quarantine_host(struct pxamci_host *host)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&host->lock, flags);
	if (host->quarantined) {
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}
	host->quarantined = true;
	host->fatal_error = true;
	host->imask = pxamci_irq_mask_all(host->pxa25x);
	writel(host->imask, host->base + MMC_I_MASK);
	spin_unlock_irqrestore(&host->lock, flags);
	if (host->irq_requested)
		synchronize_irq(host->irq);

	pxamci_disable_functional_clock(host);
	WRITE_ONCE(host->sdio_mode, false);

	if (host->power_mode == MMC_POWER_OFF)
		return;

	ret = pxamci_set_power(host, 0);
	if (ret)
		dev_err(mmc_dev(host->mmc),
			"unable to power off disabled host: %d\n", ret);
	else
		host->power_mode = MMC_POWER_OFF;
}

static int pxamci_stop_clock(struct pxamci_host *host)
{
	unsigned int stat;
	int ret;

	if (readl(host->base + MMC_STAT) & STAT_CLK_EN) {
		writel(STOP_CLOCK, host->base + MMC_STRPCL);
		ret = readl_poll_timeout(host->base + MMC_STAT, stat,
					 !(stat & STAT_CLK_EN), 10, 10000);
		if (ret) {
			dev_err(mmc_dev(host->mmc), "unable to stop clock\n");
			pxamci_quarantine_host(host);
			return ret;
		}
	}

	return 0;
}

static void pxamci_enable_irq(struct pxamci_host *host, unsigned int mask)
{
	unsigned long flags;
	unsigned int new_mask;

	spin_lock_irqsave(&host->lock, flags);
	new_mask = pxamci_irq_mask_after_enable(host->imask, mask,
						 host->fatal_error);
	if (new_mask != host->imask) {
		host->imask = new_mask;
		writel(host->imask, host->base + MMC_I_MASK);
	}
	spin_unlock_irqrestore(&host->lock, flags);
}

static void pxamci_update_card_change(struct pxamci_host *host, int present,
				      bool event_pending)
{
	unsigned long flags;
	bool request_active;

	spin_lock_irqsave(&host->lock, flags);
	request_active = !!host->mrq;
	host->card_change_pending = pxamci_card_change_after_sample(
		host->card_change_pending, present, request_active, event_pending);
	spin_unlock_irqrestore(&host->lock, flags);
}

static int pxamci_get_cd(struct mmc_host *mmc)
{
	struct pxamci_host *host = mmc_priv(mmc);
	int present = mmc_gpio_get_cd(mmc);

	if (host->pxa3xx_eject)
		pxamci_update_card_change(host, present, false);

	return present;
}

static void pxamci_card_event(struct mmc_host *mmc)
{
	pxamci_get_cd(mmc);
}

static void pxamci_latch_card_event(struct mmc_host *mmc)
{
	struct pxamci_host *host = mmc_priv(mmc);

	if (!host->pxa3xx_eject || mmc->caps & MMC_CAP_NONREMOVABLE)
		return;

	pxamci_update_card_change(host, -EAGAIN, true);
}

static bool pxamci_card_unavailable_now(struct pxamci_host *host)
{
	bool nonremovable = host->mmc->caps & MMC_CAP_NONREMOVABLE;
	int present;

	if (!host->pxa3xx_eject || nonremovable)
		return false;

	present = pxamci_get_cd(host->mmc);
	return pxamci_card_unavailable(true, false,
				       READ_ONCE(host->card_change_pending),
				       READ_ONCE(host->mmc->trigger_card_event),
				       present);
}

static void pxamci_disable_irq(struct pxamci_host *host, unsigned int mask)
{
	unsigned long flags;
	unsigned int new_mask;

	spin_lock_irqsave(&host->lock, flags);
	new_mask = host->imask | mask;
	if (pxamci_irq_mask_write_needed(host->imask, new_mask,
					 host->fatal_error)) {
		host->imask = new_mask;
		writel(host->imask, host->base + MMC_I_MASK);
	} else
		host->imask = new_mask;
	spin_unlock_irqrestore(&host->lock, flags);
}

static void pxamci_dma_irq(void *param);

static unsigned int pxamci_get_data_timeout_ms(struct mmc_data *data,
					       unsigned int clock,
					       unsigned int bus_width)
{
	return pxamci_data_timeout_ms(data->timeout_ns, data->timeout_clks,
				      clock, data->blocks * data->blksz,
				      bus_width);
}

static int pxamci_setup_data(struct pxamci_host *host, struct mmc_data *data)
{
	struct dma_async_tx_descriptor *tx;
	struct pxamci_dma *dma;
	enum dma_transfer_direction direction;
	enum dma_data_direction dma_dir;
	struct dma_slave_config	config;
	struct dma_chan *chan;
	dma_cookie_t cookie;
	unsigned int dma_len;
	unsigned int nob = data->blocks;
	unsigned long flags;
	unsigned int bus_width;
	unsigned int clock;
	bool start_dma;
	int ret;

	writel(nob, host->base + MMC_NOB);
	writel(data->blksz, host->base + MMC_BLKLEN);

	writel(pxamci_read_timeout_reg(data->timeout_ns, data->timeout_clks,
				       host->clkrate,
				       host->mmc->actual_clock),
	       host->base + MMC_RDTO);

	memset(&config, 0, sizeof(config));
	config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	config.src_addr = host->res->start + MMC_RXFIFO;
	config.dst_addr = host->res->start + MMC_TXFIFO;
	config.src_maxburst = 32;
	config.dst_maxburst = 32;

	if (data->flags & MMC_DATA_READ) {
		dma_dir = DMA_FROM_DEVICE;
		direction = DMA_DEV_TO_MEM;
		chan = host->dma_chan_rx;
	} else {
		dma_dir = DMA_TO_DEVICE;
		direction = DMA_MEM_TO_DEV;
		chan = host->dma_chan_tx;
	}

	config.direction = direction;

	ret = dmaengine_slave_config(chan, &config);
	if (ret < 0) {
		dev_err(mmc_dev(host->mmc), "dma slave config failed\n");
		return ret;
	}

	dma = kzalloc(sizeof(*dma), GFP_KERNEL);
	if (!dma)
		return -ENOMEM;

	dma->host = host;
	dma->data = data;
	dma->chan = chan;
	dma->dir = dma_dir;
	dma->request_seq = host->request_seq;
	clock = host->mmc->actual_clock ?: host->mmc->ios.clock;
	bus_width = host->mmc->ios.bus_width == MMC_BUS_WIDTH_4 ? 4 : 1;
	dma->timeout_ms = pxamci_get_data_timeout_ms(data, clock, bus_width);

	dma_len = dma_map_sg(chan->device->dev, data->sg, data->sg_len,
			     dma_dir);
	if (!dma_len) {
		dev_err(mmc_dev(host->mmc), "dma_map_sg() failed\n");
		ret = -ENOMEM;
		goto free_dma;
	}

	tx = dmaengine_prep_slave_sg(chan, data->sg, dma_len, direction,
				     DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!tx) {
		dev_err(mmc_dev(host->mmc), "prep_slave_sg() failed\n");
		ret = -ENOMEM;
		goto unmap;
	}

	tx->callback = pxamci_dma_irq;
	tx->callback_param = dma;

	cookie = dmaengine_submit(tx);
	ret = dma_submit_error(cookie);
	if (ret) {
		dev_err(mmc_dev(host->mmc), "dmaengine_submit() failed: %d\n",
			ret);
		goto unmap;
	}
	dma->cookie = cookie;

	start_dma = pxamci_dma_starts_before_command(host->pxa27x,
						     data->flags & MMC_DATA_READ);

	spin_lock_irqsave(&host->lock, flags);
	host->data = data;
	host->dma = dma;
	host->dma_done = false;
	host->data_done_pending = false;
	host->data_finishing = false;
	host->data_abort = false;
	host->recovery_pending = false;

	/*
	 * Work around PXA27x erratum E90:
	 * only start DMA now if we are doing a read,
	 * otherwise we wait until CMD/RESP has finished
	 * before starting DMA.
	 *
	 * Keep publication and issue_pending() atomic to the watchdog.  In
	 * particular, dma->started must never describe a submitted descriptor
	 * which recovery can terminate before it has actually been issued.
	 */
	if (start_dma)
		dma_async_issue_pending(chan);
	dma->started = start_dma;
	spin_unlock_irqrestore(&host->lock, flags);

	return 0;

unmap:
	dma_unmap_sg(chan->device->dev, data->sg, data->sg_len, dma_dir);
free_dma:
	kfree(dma);
	return ret;
}

static void pxamci_start_cmd(struct pxamci_host *host,
			     struct mmc_command *cmd, unsigned int cmdat,
			     unsigned int minimum_timeout_ms)
{
	unsigned long flags;
	unsigned int irq_mask;
	unsigned int timeout_ms;
	bool starts_program;

	timeout_ms = pxamci_command_timeout_ms(cmd->busy_timeout,
					       minimum_timeout_ms);
	starts_program = cmd->flags & MMC_RSP_BUSY;
	if (host->mrq && cmd == host->mrq->cmd && host->mrq->data &&
	    host->mrq->data->flags & MMC_DATA_WRITE)
		starts_program = true;
	irq_mask = pxamci_command_irq_enable_mask(starts_program);

	if (cmd->flags & MMC_RSP_BUSY)
		cmdat |= CMDAT_BUSY;

#define RSP_TYPE(x)	((x) & ~(MMC_RSP_BUSY|MMC_RSP_OPCODE))
	switch (RSP_TYPE(mmc_resp_type(cmd))) {
	case RSP_TYPE(MMC_RSP_R1): /* r1, r1b, r6, r7 */
		cmdat |= CMDAT_RESP_SHORT;
		break;
	case RSP_TYPE(MMC_RSP_R3):
		cmdat |= CMDAT_RESP_R3;
		break;
	case RSP_TYPE(MMC_RSP_R2):
		cmdat |= CMDAT_RESP_R2;
		break;
	default:
		break;
	}

	/*
	 * MMC_STAT is cleared only when START_CLOCK begins the new command.
	 * Hold host->lock from software publication through activation and IRQ
	 * unmask so recovery cannot bind the preceding phase's status to cmd or
	 * finish this command before its completion IRQ has been enabled.
	 */
	spin_lock_irqsave(&host->lock, flags);
	WARN_ON(host->cmd);
	WARN_ON(host->command_done_pending);
	if (starts_program)
		host->program_done = false;
	host->cmd = cmd;
	host->command_activated = false;
	host->command_done_pending = false;
	host->request_deadline = jiffies + msecs_to_jiffies(timeout_ms);

	writel(cmd->opcode, host->base + MMC_CMD);
	writel(cmd->arg >> 16, host->base + MMC_ARGH);
	writel(cmd->arg & 0xffff, host->base + MMC_ARGL);
	writel(cmdat, host->base + MMC_CMDAT);
	writel(host->clkrt, host->base + MMC_CLKRT);

	writel(START_CLOCK, host->base + MMC_STRPCL);
	host->imask &= ~irq_mask;
	writel(host->imask, host->base + MMC_I_MASK);
	host->command_activated = true;
	spin_unlock_irqrestore(&host->lock, flags);

	mod_delayed_work(system_wq, &host->data_watchdog,
			 msecs_to_jiffies(PXAMCI_WATCHDOG_INTERVAL_MS));
}

static void pxamci_finish_request(struct pxamci_host *host, struct mmc_request *mrq)
{
	unsigned long flags;
	unsigned int new_mask;

	cancel_delayed_work(&host->data_watchdog);

	spin_lock_irqsave(&host->lock, flags);
	if (WARN_ON(host->mrq != mrq)) {
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}
	host->mrq = NULL;
	host->cmd = NULL;
	host->command_activated = false;
	host->command_done_pending = false;
	host->data = NULL;
	host->data_done_pending = false;
	host->data_finishing = false;
	host->data_abort = false;
	host->program_needed = false;
	host->program_done = false;
	host->program_wait = false;
	host->program_cmd = NULL;
	new_mask = host->imask | PRG_DONE;
	if (pxamci_irq_mask_write_needed(host->imask, new_mask,
					 host->fatal_error))
		writel(new_mask, host->base + MMC_I_MASK);
	host->imask = new_mask;
	spin_unlock_irqrestore(&host->lock, flags);

	mmc_request_done(host->mmc, mrq);
}

static int pxamci_cmd_done(struct pxamci_host *host, unsigned int stat)
{
	struct mmc_request *mrq;
	struct mmc_command *cmd;
	enum pxamci_lifecycle_action action;
	unsigned long flags;
	bool abort_data = false;
	bool continue_request = false;
	bool data_may_be_active;
	bool finish_request = false;
	bool wait_for_data = false;
	int i;
	u32 v;

	spin_lock_irqsave(&host->lock, flags);
	cmd = host->cmd;
	if (!cmd || !host->command_activated)
		goto out_unlock;

	host->cmd = NULL;
	host->command_activated = false;
	host->command_done_pending = false;

	/*
	 * Did I mention this is Sick.  We always need to
	 * discard the upper 8 bits of the first 16-bit word.
	 */
	v = readl(host->base + MMC_RES) & 0xffff;
	for (i = 0; i < 4; i++) {
		u32 w1 = readl(host->base + MMC_RES) & 0xffff;
		u32 w2 = readl(host->base + MMC_RES) & 0xffff;

		cmd->resp[i] = v << 24 | w1 << 8 | w2 >> 8;
		v = w2;
	}

	if (pxamci_card_unavailable(host->pxa3xx_eject,
				    host->mmc->caps & MMC_CAP_NONREMOVABLE,
				    READ_ONCE(host->card_change_pending),
				    READ_ONCE(host->mmc->trigger_card_event),
				    -EOPNOTSUPP)) {
		cmd->error = -ENOMEDIUM;
	} else if (stat & STAT_TIME_OUT_RESPONSE) {
		cmd->error = -ETIMEDOUT;
	} else if (stat & STAT_RES_CRC_ERR && cmd->flags & MMC_RSP_CRC) {
		/*
		 * Work around PXA27x C0 erratum E41:
		 * A bogus CRC error can appear if the msb of a 136 bit
		 * response is a one.
		 */
		if (pxamci_ignore_r2_crc(host->pxa27x_c0, cmd->flags,
					 cmd->resp[0]))
			pr_debug("ignoring CRC from command %d - *risky*\n", cmd->opcode);
		else
			cmd->error = -EILSEQ;
	}
	WRITE_ONCE(host->sdio_mode,
		   pxamci_sdio_mode_after_command(READ_ONCE(host->sdio_mode),
						  cmd->opcode, cmd->flags,
						  cmd->error));

	host->imask |= END_CMD_RES;
	data_may_be_active = cmd->error && host->dma && host->dma->started &&
		(stat & STAT_RES_CRC_ERR);
	action = pxamci_cmd_done_action(!!host->data, !!cmd->error,
					data_may_be_active,
					host->mrq && cmd == host->mrq->sbc,
					host->program_needed,
					host->program_done);
	if (action == PXAMCI_ACTION_START_DATA) {
		if (WARN_ON(!host->dma)) {
			cmd->error = -EIO;
			host->data_abort = true;
			host->data_finishing = true;
			host->imask |= DATA_TRAN_DONE;
			abort_data = true;
			goto write_mask;
		}
		host->request_deadline = jiffies +
			msecs_to_jiffies(host->dma->timeout_ms);
		host->recovery_pending = false;
		host->imask &= ~DATA_TRAN_DONE;
		/*
		 * Work around PXA27x erratum E90: for writes, enable DMA
		 * only after the command/response sequence has completed.
		 */
		if (pxamci_dma_starts_after_command(host->pxa27x,
						    host->data->flags &
						    MMC_DATA_WRITE)) {
			dma_async_issue_pending(host->dma_chan_tx);
			host->dma->started = true;
		}
	} else if (action == PXAMCI_ACTION_RECOVER) {
		host->data_finishing = true;
		host->data_abort = true;
		host->data_done_pending = false;
		host->imask |= DATA_TRAN_DONE;
		abort_data = true;
	} else if (action == PXAMCI_ACTION_WAIT_FOR_DATA) {
		host->data->error =
			pxamci_preserve_error(host->data->error, cmd->error);
		host->recovery_pending = true;
		host->request_deadline = jiffies +
			msecs_to_jiffies(PXAMCI_RECOVERY_GRACE_MS);
		host->imask &= ~DATA_TRAN_DONE;
		wait_for_data = true;
	} else if (action == PXAMCI_ACTION_START_COMMAND) {
		continue_request = true;
	} else if (pxamci_program_irq_after_response(action)) {
		host->program_wait = true;
		host->program_cmd = cmd;
	} else {
		mrq = host->mrq;
		finish_request = true;
	}

write_mask:
	writel(host->imask, host->base + MMC_I_MASK);
	spin_unlock_irqrestore(&host->lock, flags);

	if (abort_data)
		mod_delayed_work(system_wq, &host->data_watchdog, 0);
	else if (wait_for_data)
		mod_delayed_work(system_wq, &host->data_watchdog, 0);
	else if (continue_request)
		schedule_work(&host->request_work);
	else if (finish_request)
		pxamci_finish_request(host, mrq);

	return 1;

out_unlock:
	spin_unlock_irqrestore(&host->lock, flags);
	return 0;
}

static int pxamci_complete_data(struct pxamci_host *host, unsigned int stat,
				struct pxamci_dma *dma, bool abort_request)
{
	struct mmc_request *mrq;
	struct mmc_data *data = dma->data;
	enum pxamci_lifecycle_action action;
	unsigned long flags;
	unsigned int blocks_remaining = data->blocks;
	unsigned int data_timeout_ms = dma->timeout_ms;
	unsigned int stop_cmdat;
	bool data_failed;
	bool data_write = data->flags & MMC_DATA_WRITE;
	bool card_unavailable;
	bool progress_valid;

	spin_lock_irqsave(&host->lock, flags);
	if (WARN_ON(host->data != data || host->dma != dma)) {
		spin_unlock_irqrestore(&host->lock, flags);
		return 1;
	}
	spin_unlock_irqrestore(&host->lock, flags);

	dma_unmap_sg(dma->chan->device->dev, data->sg, data->sg_len,
		     dma->dir);

	card_unavailable = !abort_request && pxamci_card_unavailable_now(host);
	if (card_unavailable && !data->error)
		data->error = -ENOMEDIUM;
	if (!abort_request && !data->error)
		data->error = pxamci_data_error(stat, !host->pxa25x);
	progress_valid = pxamci_blocks_remaining_valid(host->pxa25x, stat,
						       data->error);
	if (progress_valid)
		blocks_remaining = readl(host->base + MMC_BLKS_REM);
	data_failed = abort_request || data->error;

	/* PXA27x and later only define partial progress for read timeouts. */
	data->bytes_xfered =
		pxamci_bytes_xfered(data->blocks, data->blksz, blocks_remaining,
				     progress_valid, !data_failed);

	pxamci_disable_irq(host, DATA_TRAN_DONE);

	spin_lock_irqsave(&host->lock, flags);
	if (WARN_ON(host->data != data || host->dma != dma)) {
		spin_unlock_irqrestore(&host->lock, flags);
		return 1;
	}
	mrq = host->mrq;
	stop_cmdat = pxamci_stop_cmdat(host->data_cmdat);
	host->data = NULL;
	host->dma = NULL;
	host->data_abort = false;
	host->recovery_pending = false;
	action = pxamci_finish_data_action(abort_request || card_unavailable,
					   !!data->error,
					   !!mrq->sbc, !!mrq->stop, data_write,
					   host->program_done);
	if (action == PXAMCI_ACTION_WAIT_FOR_EVENT) {
		host->program_wait = true;
		host->program_cmd = NULL;
		host->request_deadline = jiffies +
			msecs_to_jiffies(data_timeout_ms);
	}
	spin_unlock_irqrestore(&host->lock, flags);
	kfree(dma);

	if (action == PXAMCI_ACTION_START_STOP) {
		struct mmc_command *stop = mrq->stop;
		int ret = pxamci_stop_clock(host);

		if (WARN_ON(!stop)) {
			mrq->cmd->error = pxamci_preserve_error(mrq->cmd->error,
							   -EIO);
			pxamci_finish_request(host, mrq);
		} else if (ret) {
			stop->error = ret;
			pxamci_finish_request(host, mrq);
		} else {
			pxamci_start_cmd(host, stop, stop_cmdat,
					 data_timeout_ms);
		}
	} else if (action == PXAMCI_ACTION_FINISH_REQUEST) {
		pxamci_finish_request(host, mrq);
	}

	return 1;
}

static void pxamci_data_work(struct work_struct *work)
{
	struct pxamci_host *host = container_of(work, struct pxamci_host,
						 data_work);
	struct pxamci_dma *dma;
	unsigned long flags;
	unsigned int stat;
	bool abort_request;

	spin_lock_irqsave(&host->lock, flags);
	dma = host->dma;
	if (!dma || !host->data_finishing) {
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}
	stat = host->data_done_stat;
	abort_request = host->data_abort;
	spin_unlock_irqrestore(&host->lock, flags);

	pxamci_complete_data(host, stat, dma, abort_request);
}

static enum pxamci_data_completion_context
pxamci_data_completion_context_for_host(struct pxamci_host *host)
{
	bool card_check_may_sleep = host->pxa3xx_eject &&
		!(host->mmc->caps & MMC_CAP_NONREMOVABLE);
	bool has_stop = host->mrq && host->mrq->stop;

	return pxamci_data_completion_context(card_check_may_sleep, has_stop);
}

static void pxamci_complete_data_from_atomic(struct pxamci_host *host,
					     unsigned int stat,
					     struct pxamci_dma *dma,
					     bool abort_request)
{
	if (pxamci_data_completion_context_for_host(host) ==
	    PXAMCI_DATA_COMPLETE_DEDICATED_WORK)
		queue_work(host->data_wq, &host->data_work);
	else
		pxamci_complete_data(host, stat, dma, abort_request);
}

static int pxamci_data_done(struct pxamci_host *host, unsigned int stat)
{
	struct pxamci_dma *dma;
	struct mmc_data *data;
	enum pxamci_lifecycle_action action;
	unsigned long flags;
	bool recover_dma = false;
	bool wait_for_dma = false;

	spin_lock_irqsave(&host->lock, flags);
	dma = host->dma;
	if (!dma || host->data != dma->data || host->data_finishing) {
		spin_unlock_irqrestore(&host->lock, flags);
		return dma ? 1 : 0;
	}
	data = dma->data;

	if (!data->error)
		data->error = pxamci_data_error(stat, !host->pxa25x);

	action = pxamci_data_done_action(!!data->error, host->dma_done);
	if (action == PXAMCI_ACTION_RECOVER) {
		host->data_done_pending = true;
		host->data_done_stat = stat;
		host->imask |= DATA_TRAN_DONE;
		writel(host->imask, host->base + MMC_I_MASK);
		recover_dma = true;
	} else if (action == PXAMCI_ACTION_WAIT_FOR_DMA) {
		host->data_done_pending = true;
		host->data_done_stat = stat;
		host->imask |= DATA_TRAN_DONE;
		writel(host->imask, host->base + MMC_I_MASK);
		wait_for_dma = true;
	}
	if (!wait_for_dma && !recover_dma) {
		host->data_finishing = true;
		host->data_done_pending = false;
		host->data_done_stat = stat;
	}
	spin_unlock_irqrestore(&host->lock, flags);
	if (recover_dma) {
		mod_delayed_work(system_wq, &host->data_watchdog, 0);
		return 1;
	}

	if (wait_for_dma)
		return 1;

	pxamci_complete_data_from_atomic(host, stat, dma, false);
	return 1;
}

static int pxamci_program_done(struct pxamci_host *host, unsigned int stat)
{
	struct mmc_request *mrq = NULL;
	enum pxamci_lifecycle_action action;
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	action = pxamci_program_done_action(host->mrq && host->program_needed,
					    host->program_wait);
	if (action == PXAMCI_ACTION_IGNORE) {
		spin_unlock_irqrestore(&host->lock, flags);
		return 0;
	}

	host->program_done = true;
	if (pxamci_program_error(stat, !host->pxa25x)) {
		if (host->mrq->data &&
		    host->mrq->data->flags & MMC_DATA_WRITE) {
			host->mrq->data->error =
				pxamci_preserve_error(host->mrq->data->error, -EIO);
			host->mrq->data->bytes_xfered = 0;
		} else if (host->program_cmd) {
			host->program_cmd->error =
				pxamci_preserve_error(host->program_cmd->error, -EIO);
		} else {
			host->mrq->cmd->error =
				pxamci_preserve_error(host->mrq->cmd->error, -EIO);
		}
	}
	host->imask |= PRG_DONE;
	writel(host->imask, host->base + MMC_I_MASK);
	if (action == PXAMCI_ACTION_FINISH_REQUEST) {
		host->program_wait = false;
		mrq = host->mrq;
	}
	spin_unlock_irqrestore(&host->lock, flags);

	if (mrq)
		pxamci_finish_request(host, mrq);

	return 1;
}

static int pxamci_defer_cmd_done(struct pxamci_host *host, unsigned int stat)
{
	unsigned long flags;
	bool schedule = false;

	spin_lock_irqsave(&host->lock, flags);
	host->imask |= END_CMD_RES;
	writel(host->imask, host->base + MMC_I_MASK);
	if (host->cmd && host->command_activated &&
	    !host->command_done_pending) {
		host->command_done_stat = stat;
		host->command_done_pending = true;
		schedule = true;
	}
	spin_unlock_irqrestore(&host->lock, flags);

	if (schedule)
		schedule_work(&host->command_work);

	/* The source is masked even if a stale interrupt had no current command. */
	return 1;
}

static void pxamci_command_work(struct work_struct *work)
{
	struct pxamci_host *host = container_of(work, struct pxamci_host,
						 command_work);
	unsigned long flags;
	unsigned int stat;

	/* PXA3xx FEr#44 requires a live card-detect sample before acceptance. */
	pxamci_card_unavailable_now(host);

	spin_lock_irqsave(&host->lock, flags);
	if (!host->command_done_pending) {
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}
	stat = host->command_done_stat;
	host->command_done_pending = false;
	spin_unlock_irqrestore(&host->lock, flags);

	pxamci_cmd_done(host, stat);
}

static irqreturn_t pxamci_irq(int irq, void *devid)
{
	struct pxamci_host *host = devid;
	unsigned int ireg;
	int handled = 0;

	ireg = readl(host->base + MMC_I_REG) & ~readl(host->base + MMC_I_MASK);

	if (ireg) {
		unsigned int stat = readl(host->base + MMC_STAT);

		pr_debug("PXAMCI: irq %08x stat %08x\n", ireg, stat);

		if (ireg & END_CMD_RES) {
			if (pxamci_defer_command_completion(host->pxa3xx_eject,
							    host->mmc->caps &
							    MMC_CAP_NONREMOVABLE))
				handled |= pxamci_defer_cmd_done(host, stat);
			else
				handled |= pxamci_cmd_done(host, stat);
		}
		if (ireg & DATA_TRAN_DONE)
			handled |= pxamci_data_done(host, stat);
		if (ireg & PRG_DONE)
			handled |= pxamci_program_done(host, stat);
		if (ireg & SDIO_INT) {
			mmc_signal_sdio_irq(host->mmc);
			handled = 1;
		}
	}

	return IRQ_RETVAL(handled);
}

static int pxamci_terminate_dma_chan(struct dma_chan *chan)
{
	unsigned int attempt;
	int ret = 0;

	for (attempt = 0; attempt < PXAMCI_DMA_TERMINATE_RETRIES; attempt++) {
		ret = dmaengine_terminate_sync(chan);
		if (!ret)
			break;
	}

	return ret;
}

/*
 * Bound every phase of a request.  The controller has no dependable fallback
 * for a lost command, data, DMA, or stop-command interrupt.  DMA status is
 * only used for terminal COMPLETE/ERROR states; residue from a running DMA
 * channel is not a reliable completion indication.
 */
static void pxamci_data_watchdog(struct work_struct *work)
{
	struct pxamci_host *host = container_of(to_delayed_work(work),
						 struct pxamci_host, data_watchdog);
	struct mmc_request *mrq;
	struct mmc_command *cmd;
	struct mmc_data *data;
	struct pxamci_dma *dma;
	struct dma_chan *chan = NULL;
	struct dma_tx_state state = { };
	struct pxamci_watchdog_state lifecycle = { };
	struct pxamci_watchdog_decision decision;
	enum dma_status status = DMA_IN_PROGRESS;
	dma_cookie_t cookie = 0;
	unsigned long flags;
	unsigned int command_opcode;
	unsigned int request_seq;
	unsigned int stat;
	bool controller_done;
	bool command_activated;
	bool card_unavailable_now;
	bool dma_started = false;
	bool abort_request;
	int ret;

	card_unavailable_now = pxamci_card_unavailable_now(host);

	spin_lock_irqsave(&host->lock, flags);
	mrq = host->mrq;
	cmd = host->cmd;
	command_activated = host->command_activated;
	dma = host->dma;
	data = dma ? dma->data : NULL;
	command_opcode = pxamci_command_opcode_snapshot(cmd);
	request_seq = host->request_seq;
	if (!mrq || (dma && host->data != data) ||
	    (dma && dma->request_seq != request_seq)) {
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}

	if (dma) {
		cookie = dma->cookie;
		chan = dma->chan;
		dma_started = dma->started;
	}
	spin_unlock_irqrestore(&host->lock, flags);

	stat = readl(host->base + MMC_STAT);
	controller_done = stat & STAT_DATA_TRAN_DONE;
	if (dma)
		status = dmaengine_tx_status(chan, cookie, &state);

	spin_lock_irqsave(&host->lock, flags);
	lifecycle.request_current = host->mrq == mrq &&
				    host->request_seq == request_seq;
	lifecycle.card_removed = card_unavailable_now ||
		pxamci_card_unavailable(host->pxa3xx_eject,
					host->mmc->caps & MMC_CAP_NONREMOVABLE,
					host->card_change_pending,
					READ_ONCE(host->mmc->trigger_card_event),
					-EOPNOTSUPP);
	lifecycle.data_active = lifecycle.request_current && dma &&
				 host->data == data && host->dma == dma;
	lifecycle.finishing = host->data_finishing;
	lifecycle.abort_request = host->data_abort;
	lifecycle.dma_failed = status == DMA_ERROR;
	lifecycle.data_failed = lifecycle.data_active && !!data->error;
	lifecycle.dma_complete = status == DMA_COMPLETE;
	lifecycle.controller_done = controller_done;
	lifecycle.data_done_pending = host->data_done_pending;
	lifecycle.deadline_expired =
		time_after_eq(jiffies, host->request_deadline);
	lifecycle.command_active = lifecycle.request_current &&
		pxamci_command_is_current(host->cmd, cmd, host->request_seq,
					  request_seq,
					  command_activated &&
					  host->command_activated);
	lifecycle.command_done = stat & STAT_END_CMD_RES;
	lifecycle.command_deferred = lifecycle.command_active &&
				     host->command_done_pending;
	lifecycle.dma_started =
		pxamci_watchdog_dma_was_started(lifecycle.data_active,
						 dma_started);
	lifecycle.recovery_pending = host->recovery_pending;
	lifecycle.program_active = lifecycle.request_current &&
				   host->program_wait;
	lifecycle.program_done = stat & STAT_PRG_DONE;
	decision = pxamci_watchdog_decide(&lifecycle);
	if (decision.action == PXAMCI_ACTION_IGNORE) {
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}
	if (decision.action == PXAMCI_ACTION_COMPLETE_COMMAND) {
		spin_unlock_irqrestore(&host->lock, flags);
		if (pxamci_cmd_done(host, stat))
			dev_warn(mmc_dev(host->mmc),
				 "recovered lost command completion for opcode %u\n",
				 command_opcode);
		return;
	}
	if (decision.action == PXAMCI_ACTION_COMPLETE_PROGRAM) {
		spin_unlock_irqrestore(&host->lock, flags);
		if (pxamci_program_done(host, stat))
			dev_warn(mmc_dev(host->mmc),
				 "recovered lost programming completion\n");
		return;
	}

	if (decision.reason == PXAMCI_RECOVERY_CARD_REMOVAL) {
		mrq->cmd->error = pxamci_preserve_error(mrq->cmd->error,
							 -ENOMEDIUM);
		if (cmd) {
			cmd->error = pxamci_preserve_error(cmd->error, -ENOMEDIUM);
			host->cmd = NULL;
			host->command_activated = false;
			host->command_done_pending = false;
			host->imask |= END_CMD_RES;
		}
		if (data)
			data->error = pxamci_preserve_error(data->error, -ENOMEDIUM);
	} else if (decision.reason == PXAMCI_RECOVERY_DMA && data) {
		data->error = -EIO;
	} else if (decision.set_command_timeout) {
		cmd->error = -ETIMEDOUT;
		host->cmd = NULL;
		host->command_activated = false;
		host->command_done_pending = false;
		host->imask |= END_CMD_RES;
	} else if (decision.set_data_timeout && data) {
		data->error = pxamci_preserve_error(data->error, -ETIMEDOUT);
	} else if (decision.set_program_timeout) {
		if (host->program_cmd)
			host->program_cmd->error =
				pxamci_preserve_error(host->program_cmd->error,
						      -ETIMEDOUT);
		else if (mrq->data && mrq->data->flags & MMC_DATA_WRITE)
			mrq->data->error =
				pxamci_preserve_error(mrq->data->error, -ETIMEDOUT);
		else
			mrq->cmd->error =
				pxamci_preserve_error(mrq->cmd->error, -ETIMEDOUT);
	}
	if (decision.start_recovery) {
		host->recovery_pending = true;
		host->request_deadline = jiffies +
			msecs_to_jiffies(PXAMCI_RECOVERY_GRACE_MS);
	}
	if (decision.mark_host_dead)
		host->fatal_error = true;

	if (decision.action == PXAMCI_ACTION_WAIT_FOR_EVENT) {
		spin_unlock_irqrestore(&host->lock, flags);
		mod_delayed_work(system_wq, &host->data_watchdog,
				 msecs_to_jiffies(PXAMCI_WATCHDOG_INTERVAL_MS));
		return;
	}
	if (decision.action == PXAMCI_ACTION_FINISH_REQUEST) {
		unsigned int new_mask;

		host->program_wait = false;
		host->program_cmd = NULL;
		new_mask = host->imask | PRG_DONE;
		if (pxamci_irq_mask_write_needed(host->imask, new_mask,
						 host->fatal_error))
			writel(new_mask, host->base + MMC_I_MASK);
		host->imask = new_mask;
		spin_unlock_irqrestore(&host->lock, flags);
		if (decision.mark_host_dead)
			pxamci_quarantine_host(host);
		if (decision.set_command_timeout)
			dev_err(mmc_dev(host->mmc), "command %u timed out\n",
				command_opcode);
		else if (decision.set_program_timeout)
			dev_err(mmc_dev(host->mmc),
				"card programming timed out\n");
		pxamci_finish_request(host, mrq);
		return;
	}

	/* Claim completion while terminate_sync() drops host->lock. */
	abort_request = decision.abort_request || decision.mark_host_dead;
	if (abort_request)
		host->data_abort = true;
	host->data_finishing = true;
	host->imask |= DATA_TRAN_DONE;
	writel(host->imask, host->base + MMC_I_MASK);
	spin_unlock_irqrestore(&host->lock, flags);

	ret = pxamci_terminate_dma_chan(chan);

	spin_lock_irqsave(&host->lock, flags);
	if (host->mrq != mrq || host->request_seq != request_seq ||
	    host->data != data || host->dma != dma) {
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}
	if (!pxamci_dma_safe_to_release(ret)) {
		data->error = pxamci_preserve_error(data->error, -EIO);
		host->data_finishing = false;
		spin_unlock_irqrestore(&host->lock, flags);
		dev_err_ratelimited(mmc_dev(host->mmc),
				    "failed to terminate stalled DMA cookie %d: %d\n",
				    cookie, ret);
		mod_delayed_work(system_wq, &host->data_watchdog,
				 msecs_to_jiffies(PXAMCI_WATCHDOG_INTERVAL_MS));
		return;
	}
	host->dma_done = true;
	host->recovery_pending = false;
	if (!host->data_done_pending)
		host->data_done_stat = stat;
	host->data_done_pending = false;
	spin_unlock_irqrestore(&host->lock, flags);

	if (decision.mark_host_dead) {
		pxamci_quarantine_host(host);
		dev_err(mmc_dev(host->mmc),
			"controller did not reach a terminal state; disabling host until reset\n");
	}

	if (decision.reason == PXAMCI_RECOVERY_CARD_REMOVAL) {
		dev_warn(mmc_dev(host->mmc),
			 "terminated DMA after card-detect event: cookie=%d\n",
			 cookie);
	} else if (decision.reason == PXAMCI_RECOVERY_COMMAND) {
		dev_warn(mmc_dev(host->mmc),
			 "terminated DMA after command error: cookie=%d error=%d residue=%u\n",
			 cookie, mrq->cmd->error, state.residue);
	} else if (decision.reason == PXAMCI_RECOVERY_DMA) {
		dev_err(mmc_dev(host->mmc),
			"DMA error: cookie=%d stat=%08x\n", cookie, stat);
	} else if (decision.reason == PXAMCI_RECOVERY_CONTROLLER) {
		dev_warn(mmc_dev(host->mmc),
			 "terminated DMA after controller error: cookie=%d stat=%08x residue=%u\n",
			 cookie, stat, state.residue);
	} else if (decision.reason == PXAMCI_RECOVERY_TIMEOUT) {
		dev_err(mmc_dev(host->mmc),
			"timed out data request: stat=%08x dma=%d residue=%u\n",
			stat, status, state.residue);
	} else if (decision.reason == PXAMCI_RECOVERY_LOST_COMPLETION) {
		dev_warn(mmc_dev(host->mmc),
			 "recovered lost DMA completion: cookie=%d stat=%08x\n",
			 cookie, stat);
	}

	/* The watchdog already runs in process context. */
	pxamci_complete_data(host, host->data_done_stat, dma, abort_request);
}

static void pxamci_set_request_error(struct mmc_request *mrq, int error)
{
	mrq->cmd->error = error;
	if (mrq->data)
		mrq->data->error = error;
}

static void pxamci_start_request(struct pxamci_host *host,
				 struct mmc_request *mrq)
{
	unsigned int cmdat;
	int ret;

	/* PXA3xx cannot reliably report CRC or stop errors after an eject. */
	if (pxamci_card_unavailable_now(host)) {
		pxamci_set_request_error(mrq, -ENOMEDIUM);
		pxamci_finish_request(host, mrq);
		return;
	}

	ret = pxamci_stop_clock(host);
	if (ret) {
		mrq->cmd->error = ret;
		pxamci_finish_request(host, mrq);
		return;
	}

	cmdat = host->cmdat;
	if (mrq->data) {
		ret = pxamci_setup_data(host, mrq->data);
		if (ret) {
			mrq->cmd->error = ret;
			pxamci_finish_request(host, mrq);
			return;
		}

		cmdat &= ~CMDAT_BUSY;
		cmdat |= CMDAT_DATAEN | CMDAT_DMAEN;
		if (mrq->data->flags & MMC_DATA_WRITE)
			cmdat |= CMDAT_WRITE;

		/* setup_data() may sleep long enough for an eject IRQ to arrive. */
		if (pxamci_card_unavailable_now(host)) {
			mod_delayed_work(system_wq, &host->data_watchdog, 0);
			return;
		}
	}

	host->data_cmdat = cmdat;
	host->cmdat &= ~CMDAT_INIT;
	pxamci_start_cmd(host, mrq->cmd, cmdat, 0);
}

static void pxamci_request_work(struct work_struct *work)
{
	struct pxamci_host *host = container_of(work, struct pxamci_host,
						 request_work);
	struct mmc_request *mrq;
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	mrq = host->mrq;
	if (!mrq || !mrq->sbc || mrq->sbc->error || host->cmd || host->data) {
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}
	spin_unlock_irqrestore(&host->lock, flags);

	pxamci_start_request(host, mrq);
}

static void pxamci_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct pxamci_host *host = mmc_priv(mmc);
	unsigned long flags;
	unsigned int cmdat;
	bool data_read = mrq->data && mrq->data->flags & MMC_DATA_READ;
	bool sd_card = mmc->card && mmc_card_sd(mmc->card);
	int ret;

	ret = pxamci_request_state_error(READ_ONCE(host->fatal_error),
					 READ_ONCE(host->ios_error));
	if (ret) {
		pxamci_set_request_error(mrq, ret);
		mmc_request_done(mmc, mrq);
		return;
	}

	if (!pxamci_command_supported(mrq->cmd->opcode, mrq->cmd->arg,
				      host->pxa27x_c0, sd_card, data_read)) {
		pxamci_set_request_error(mrq, -EOPNOTSUPP);
		mmc_request_done(mmc, mrq);
		return;
	}

	if (pxamci_card_unavailable_now(host)) {
		pxamci_set_request_error(mrq, -ENOMEDIUM);
		mmc_request_done(mmc, mrq);
		return;
	}

	/* E56's readback workaround cannot be issued inside the host request. */
	if (mrq->data &&
	    !pxamci_mmc_write_supported(host->pxa27x_e56,
					mmc->card && mmc_card_mmc(mmc->card),
					mrq->data->flags & MMC_DATA_WRITE,
					mmc->actual_clock)) {
		dev_err_ratelimited(mmc_dev(mmc),
				    "PXA27x E56 forbids writes to MMC below 19.5 MHz\n");
		pxamci_set_request_error(mrq, -EOPNOTSUPP);
		mmc_request_done(mmc, mrq);
		return;
	}

	if (mrq->data &&
	    !pxamci_data_size_supported(mrq->data->blocks,
					mrq->data->blksz, host->pxa27x,
					mrq->data->flags & MMC_DATA_READ,
					mmc->ios.bus_width == MMC_BUS_WIDTH_4)) {
		pxamci_set_request_error(mrq, -EINVAL);
		mmc_request_done(mmc, mrq);
		return;
	}

	spin_lock_irqsave(&host->lock, flags);
	WARN_ON(host->mrq);
	if (!++host->request_seq)
		host->request_seq++;
	host->mrq = mrq;
	host->program_needed =
		(mrq->data && mrq->data->flags & MMC_DATA_WRITE) ||
		(mrq->cmd->flags & MMC_RSP_BUSY) ||
		(mrq->stop && mrq->stop->flags & MMC_RSP_BUSY);
	host->program_done = false;
	host->program_wait = false;
	host->recovery_pending = false;
	spin_unlock_irqrestore(&host->lock, flags);

	if (!mrq->sbc) {
		pxamci_start_request(host, mrq);
		return;
	}

	ret = pxamci_stop_clock(host);
	if (ret) {
		mrq->sbc->error = ret;
		pxamci_finish_request(host, mrq);
		return;
	}
	cmdat = host->cmdat;
	host->cmdat &= ~CMDAT_INIT;
	pxamci_start_cmd(host, mrq->sbc, cmdat, 0);
}

static int pxamci_get_ro(struct mmc_host *mmc)
{
	struct pxamci_host *host = mmc_priv(mmc);

	if (host->use_ro_gpio)
		return mmc_gpio_get_ro(mmc);
	if (host->pdata && host->pdata->get_ro)
		return !!host->pdata->get_ro(mmc_dev(mmc));
	/*
	 * Board doesn't support read only detection; let the mmc core
	 * decide what to do.
	 */
	return -ENOSYS;
}

static void pxamci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct pxamci_host *host = mmc_priv(mmc);
	struct pxamci_clock_config config;
	unsigned int old_actual_clock = mmc->actual_clock;
	unsigned int old_clkrt = host->clkrt;
	unsigned int requested_clock;
	unsigned long flags;
	bool request_active;
	int restore_ret = 0;
	int ret;

	if (READ_ONCE(host->fatal_error)) {
		spin_lock_irqsave(&host->lock, flags);
		request_active = !!host->mrq;
		spin_unlock_irqrestore(&host->lock, flags);

		/* Never restart MMC_STRPCL after a non-terminal controller hang. */
		mmc->actual_clock = 0;
		if (pxamci_fatal_clock_can_disable(request_active))
			pxamci_disable_functional_clock(host);
		if (!request_active &&
		    pxamci_fatal_power_change_allowed(ios->power_mode)) {
			if (host->power_mode != ios->power_mode) {
				ret = pxamci_set_power(host, ios->vdd);
				if (ret)
					dev_err(mmc_dev(mmc),
						"unable to set power on disabled host: %d\n",
						ret);
				else
					host->power_mode = ios->power_mode;
			}
			WRITE_ONCE(host->sdio_mode,
				   pxamci_sdio_mode_after_power(
					   READ_ONCE(host->sdio_mode),
					   ios->power_mode));
		}
		return;
	}

	if (ios->clock) {
		if (host->clkrt == PXAMCI_CLKRT_OFF) {
			ret = pxamci_enable_functional_clock(host);
			if (ret) {
				mmc->actual_clock = 0;
				WRITE_ONCE(host->ios_error, ret);
				dev_err(mmc_dev(mmc),
					"unable to enable clock: %d\n", ret);
				return;
			}
		}

		requested_clock =
			pxamci_limit_sdio_clock(ios->clock, host->pxa27x_c0,
						READ_ONCE(host->sdio_mode) ||
						(mmc->card && mmc_card_sdio(mmc->card)));
		config = pxamci_clock_config(host->clkrate, requested_clock,
					     mmc_has_26MHz(),
					     host->pxa320_b2);
		host->clkrt = config.clkrt;
		mmc->actual_clock = config.actual_clock;

		/*
		 * we write clkrt on the next command
		 */
	} else {
		ret = pxamci_stop_clock(host);
		if (ret) {
			WRITE_ONCE(host->ios_error, ret);
			return;
		}
		pxamci_disable_functional_clock(host);
	}

	if (host->power_mode != ios->power_mode) {
		ret = pxamci_set_power(host, ios->vdd);
		if (ret) {
			dev_err(mmc_dev(mmc), "unable to set power: %d\n", ret);
			if (pxamci_power_failure_disables_clock(old_clkrt,
							   host->clkrt)) {
				clk_disable_unprepare(host->clk);
			} else if (old_clkrt != PXAMCI_CLKRT_OFF &&
				   host->clkrt == PXAMCI_CLKRT_OFF) {
				restore_ret = clk_prepare_enable(host->clk);
				if (restore_ret)
					dev_err(mmc_dev(mmc),
						"unable to restore clock after power failure: %d\n",
						restore_ret);
			}
			if (restore_ret) {
				host->clkrt = PXAMCI_CLKRT_OFF;
				mmc->actual_clock = 0;
			} else {
				host->clkrt = old_clkrt;
				mmc->actual_clock = old_actual_clock;
			}
			WRITE_ONCE(host->ios_error, restore_ret ?: ret);
			/*
			 * The .set_ios() function in the mmc_host_ops
			 * struct return void, and failing to set the
			 * power should be rare so we print an error and
			 * return here.
			 */
			return;
		}
		host->power_mode = ios->power_mode;
		WRITE_ONCE(host->sdio_mode,
			   pxamci_sdio_mode_after_power(
				   READ_ONCE(host->sdio_mode), ios->power_mode));

		if (ios->power_mode == MMC_POWER_ON)
			host->cmdat |= CMDAT_INIT;
	}

	if (ios->bus_width == MMC_BUS_WIDTH_4)
		host->cmdat |= CMDAT_SD_4DAT;
	else
		host->cmdat &= ~CMDAT_SD_4DAT;
	WRITE_ONCE(host->ios_error, 0);

	dev_dbg(mmc_dev(mmc), "PXAMCI: clkrt = %x cmdat = %x\n",
		host->clkrt, host->cmdat);
}

static void pxamci_enable_sdio_irq(struct mmc_host *host, int enable)
{
	struct pxamci_host *pxa_host = mmc_priv(host);

	if (enable)
		pxamci_enable_irq(pxa_host, SDIO_INT);
	else
		pxamci_disable_irq(pxa_host, SDIO_INT);
}

static const struct mmc_host_ops pxamci_ops = {
	.request		= pxamci_request,
	.get_cd			= pxamci_get_cd,
	.get_ro			= pxamci_get_ro,
	.set_ios		= pxamci_set_ios,
	.enable_sdio_irq	= pxamci_enable_sdio_irq,
	.card_event		= pxamci_card_event,
};

static void pxamci_dma_irq(void *param)
{
	struct pxamci_dma *dma = param;
	struct pxamci_host *host = dma->host;
	struct mmc_data *data = dma->data;
	struct dma_tx_state state = { };
	enum pxamci_lifecycle_action action;
	enum pxamci_dma_result result;
	enum dma_status status;
	unsigned long flags;
	unsigned int data_done_stat = 0;
	bool finish_data = false;
	bool recover_dma = false;

	spin_lock_irqsave(&host->lock, flags);

	if (!pxamci_dma_is_current(host->dma, dma, host->data, data,
				   host->request_seq, dma->request_seq,
				   host->data_finishing))
		goto out_unlock;

	status = dmaengine_tx_status(dma->chan, dma->cookie, &state);
	if (status == DMA_COMPLETE)
		result = PXAMCI_DMA_COMPLETE;
	else if (status == DMA_ERROR)
		result = PXAMCI_DMA_FAILED;
	else
		result = PXAMCI_DMA_RUNNING;
	action = pxamci_dma_done_action(result, host->data_done_pending);

	if (action == PXAMCI_ACTION_RECORD_DMA_DONE ||
	    action == PXAMCI_ACTION_FINISH_DATA) {
		if (data->flags & MMC_DATA_WRITE)
			writel(BUF_PART_FULL, host->base + MMC_PRTBUF);
		host->dma_done = true;
		if (action == PXAMCI_ACTION_FINISH_DATA) {
			data_done_stat = host->data_done_stat;
			host->data_done_pending = false;
			host->data_finishing = true;
			host->data_done_stat = data_done_stat;
			finish_data = true;
		}
	} else if (action == PXAMCI_ACTION_RECOVER) {
		pr_err("%s: DMA error on %s channel\n",
		       mmc_hostname(host->mmc),
		       data->flags & MMC_DATA_READ ? "rx" : "tx");
		data->error = -EIO;
		/* Keep waiting unless DATA_TRAN_DONE was already observed. */
		if (host->data_done_pending) {
			host->imask |= DATA_TRAN_DONE;
		} else {
			host->data_done_stat = 0;
			host->imask &= ~DATA_TRAN_DONE;
		}
		writel(host->imask, host->base + MMC_I_MASK);
		recover_dma = true;
	} else {
		dev_warn_ratelimited(mmc_dev(host->mmc),
				     "ignoring premature DMA callback for cookie %d\n",
				     dma->cookie);
	}

out_unlock:
	spin_unlock_irqrestore(&host->lock, flags);
	if (recover_dma)
		mod_delayed_work(system_wq, &host->data_watchdog, 0);
	else if (finish_data)
		pxamci_complete_data_from_atomic(host, data_done_stat, dma, false);
}

static irqreturn_t pxamci_detect_irq(int irq, void *devid)
{
	struct pxamci_host *host = mmc_priv(devid);

	pxamci_latch_card_event(devid);
	WRITE_ONCE(host->mmc->trigger_card_event, true);
	mmc_detect_change(devid, msecs_to_jiffies(host->detect_delay_ms));
	return IRQ_HANDLED;
}

#ifdef CONFIG_OF
static const struct of_device_id pxa_mmc_dt_ids[] = {
	{ .compatible = "marvell,pxa-mmc" },
	{ }
};

MODULE_DEVICE_TABLE(of, pxa_mmc_dt_ids);
#endif

static int pxamci_parse_firmware(struct platform_device *pdev,
				 struct mmc_host *mmc)
{
	struct pxamci_host *host = mmc_priv(mmc);
	bool legacy_detect_delay = false;
	u32 tmp;
	int ret;

	/* pxa-mmc specific */
	if (!device_property_read_u32(&pdev->dev, "marvell,detect-delay-ms",
				      &tmp) ||
	    !device_property_read_u32(&pdev->dev, "pxa-mmc,detect-delay-ms",
				      &tmp)) {
		host->detect_delay_ms = tmp;
		legacy_detect_delay = true;
	}

	ret = mmc_of_parse(mmc);
	if (ret || !legacy_detect_delay)
		return ret;

	ret = mmc_gpiod_set_cd_debounce(
		mmc, pxamci_detect_debounce_us(host->detect_delay_ms));
	return ret == -ENODEV ? 0 : ret;
}

static void pxamci_destroy_data_wq(void *data)
{
	struct pxamci_host *host = data;

	destroy_workqueue(host->data_wq);
}

static int pxamci_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct pxamci_host *host = NULL;
	struct device *dev = &pdev->dev;
	struct resource *r;
	unsigned int cpuid;
	unsigned int debounce_us;
	bool cd_irq;
	bool firmware_bus_width;
	int ret, irq;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	mmc = devm_mmc_alloc_host(dev, sizeof(*host));
	if (!mmc)
		return -ENOMEM;

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->pdata = dev_get_platdata(dev);
	host->irq = irq;
#ifdef CONFIG_ARM
	cpuid = read_cpuid_id();
#else
	cpuid = 0;
#endif
	host->pxa25x = cpu_is_pxa25x();
	host->pxa27x = cpu_is_pxa27x();
	host->pxa3xx_eject = cpu_is_pxa300() || cpu_is_pxa310() ||
				cpu_is_pxa320();
	host->pxa27x_c0 = pxamci_is_pxa27x_c0(cpuid);
	host->pxa27x_e56 = pxamci_is_pxa27x_e56(cpuid);
	host->pxa320_b2 = pxamci_is_pxa320_b2(cpuid);
	host->clkrt = PXAMCI_CLKRT_OFF;
	if (host->pdata)
		host->detect_delay_ms = host->pdata->detect_delay_ms;

	mmc->ops = &pxamci_ops;
	if (host->pxa25x)
		mmc->caps2 |= MMC_CAP2_KEEP_SD_POWER_IN_S2IDLE;
	if (pxamci_use_legacy_ro_active_high(!!host->pdata, !!dev_fwnode(dev),
					     host->pdata ?
					     host->pdata->gpio_card_ro_invert :
					     false))
		mmc->caps2 |= MMC_CAP2_RO_ACTIVE_HIGH;

	/*
	 * PXA DMA can split mapped SG entries into its 8-KiB hardware
	 * descriptors.  Keep the queue bounded while allowing ordinary
	 * multi-page block requests on controllers with a usable block counter.
	 */
	mmc->max_segs = host->pxa25x ? 1 : PXAMCI_MAX_SEGS;

	/*
	 * Our hardware DMA can handle a maximum of one page per SG entry.
	 */
	mmc->max_seg_size = PAGE_SIZE;
	mmc->max_req_size = mmc->max_segs * mmc->max_seg_size;

	/*
	 * Block length register is only 10 bits before PXA27x.
	 */
	mmc->max_blk_size = host->pxa25x ? 1023 : 2048;

	/*
	 * Block count register is 16 bits.
	 */
	mmc->max_blk_count = host->pxa25x ? 1 : 65535;

	host->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(host->clk))
		return dev_err_probe(dev, PTR_ERR(host->clk),
					"Failed to acquire clock\n");

	host->clkrate = clk_get_rate(host->clk);
	if (!host->clkrate)
		return dev_err_probe(dev, -EINVAL, "invalid clock rate\n");

	/*
	 * Calculate minimum clock rate, rounding up.
	 */
	mmc->f_min = (host->clkrate + 63) / 64;
	mmc->f_max = (mmc_has_26MHz()) ? 26000000 : host->clkrate;
	host->cmdat = host->pxa25x ? 0 : CMDAT_SDIO_INT_EN;
	firmware_bus_width = device_property_present(dev, "bus-width");

	ret = pxamci_parse_firmware(pdev, mmc);
	if (ret)
		return ret;
	mmc->caps = pxamci_merge_caps(mmc->caps, host->pxa25x,
				      mmc_has_26MHz(), firmware_bus_width);
	if (!pxamci_bus_width_caps_valid(mmc->caps, host->pxa25x))
		return dev_err_probe(dev, -EINVAL,
				     "unsupported firmware bus-width\n");
	mmc->f_max = min_t(unsigned int, mmc->f_max,
			   mmc_has_26MHz() ? 26000000 : host->clkrate);
	host->use_ro_gpio = mmc_host_can_gpio_ro(mmc);

	ret = pxamci_init_ocr(host);
	if (ret < 0)
		return ret;

	spin_lock_init(&host->lock);
	INIT_DELAYED_WORK(&host->data_watchdog, pxamci_data_watchdog);
	INIT_WORK(&host->command_work, pxamci_command_work);
	INIT_WORK(&host->data_work, pxamci_data_work);
	INIT_WORK(&host->request_work, pxamci_request_work);
	/*
	 * Sleepable data completion must make progress even when the request was
	 * submitted by a worker that is synchronously waiting for the MMC core.
	 * Do not put this work back on a shared system workqueue.
	 */
	host->data_wq = alloc_ordered_workqueue("%s-data", WQ_MEM_RECLAIM,
						dev_name(dev));
	if (!host->data_wq)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, pxamci_destroy_data_wq, host);
	if (ret)
		return ret;
	host->imask = pxamci_irq_mask_all(host->pxa25x);

	host->base = devm_platform_get_and_ioremap_resource(pdev, 0, &r);
	if (IS_ERR(host->base))
		return PTR_ERR(host->base);
	host->res = r;

	/*
	 * The boot loader may leave the card clock running while the common
	 * clock framework still considers the functional clock unused.  Take
	 * explicit ownership before touching MMC_STRPCL so the late unused-clock
	 * pass cannot gate the stop handshake underneath asynchronous probing.
	 */
	ret = pxamci_enable_functional_clock(host);
	if (ret)
		return dev_err_probe(dev, ret,
				     "unable to enable clock for initialization\n");

	ret = pxamci_stop_clock(host);
	if (ret)
		return ret;
	writel(0, host->base + MMC_SPI);
	writel(64, host->base + MMC_RESTO);
	writel(host->imask, host->base + MMC_I_MASK);
	pxamci_disable_functional_clock(host);

	ret = devm_request_irq(dev, irq, pxamci_irq, 0,
			       DRIVER_NAME, host);
	if (ret)
		return ret;
	host->irq_requested = true;

	host->dma_chan_rx = devm_dma_request_chan(dev, "rx");
	if (IS_ERR(host->dma_chan_rx))
		return dev_err_probe(dev, PTR_ERR(host->dma_chan_rx),
				     "unable to request rx dma channel\n");

	host->dma_chan_tx = devm_dma_request_chan(dev, "tx");
	if (IS_ERR(host->dma_chan_tx))
		return dev_err_probe(dev, PTR_ERR(host->dma_chan_tx),
					"unable to request tx dma channel\n");

	if (host->pdata) {
		host->power = devm_gpiod_get_optional(dev, "power", GPIOD_OUT_LOW);
		if (IS_ERR(host->power))
			return dev_err_probe(dev, PTR_ERR(host->power),
						"Failed requesting gpio_power\n");

		if (!dev_fwnode(dev)) {
			debounce_us =
				pxamci_detect_debounce_us(host->detect_delay_ms);
			ret = mmc_gpiod_request_cd(mmc, "cd", 0, false, debounce_us);
			if (ret && ret != -ENOENT)
				return dev_err_probe(dev, ret,
						     "Failed requesting gpio_cd\n");

			ret = mmc_gpiod_request_ro(mmc, "wp", 0, 0);
			if (ret && ret != -ENOENT)
				return dev_err_probe(dev, ret,
						     "Failed requesting gpio_ro\n");
		}

		host->use_ro_gpio = mmc_host_can_gpio_ro(mmc);

		if (host->pdata->init) {
			ret = host->pdata->init(dev, pxamci_detect_irq, mmc);
			if (ret)
				return dev_err_probe(dev, ret,
						     "platform initialization failed\n");
		}

		if (host->power && host->pdata->setpower)
			dev_warn(dev, "gpio_power and setpower() both defined\n");
		if (host->use_ro_gpio && host->pdata->get_ro)
			dev_warn(dev, "gpio_ro and get_ro() both defined\n");
	}

	cd_irq = false;
	if (host->pxa3xx_eject &&
	    !(mmc->caps & MMC_CAP_NONREMOVABLE)) {
		mmc_gpiod_request_cd_irq(mmc);
		cd_irq = mmc->slot.cd_irq >= 0;
	}
	if (!pxamci_eject_detection_supported(
		    host->pxa3xx_eject,
		    mmc->caps & MMC_CAP_NONREMOVABLE,
		    mmc_host_can_gpio_cd(mmc), cd_irq)) {
		ret = dev_err_probe(dev, -EINVAL,
			"PXA3xx removable cards require readable IRQ card-detect\n");
		goto exit_platform;
	}

	platform_set_drvdata(pdev, mmc);
	ret = mmc_add_host(mmc);
	if (ret) {
		platform_set_drvdata(pdev, NULL);
		goto exit_platform;
	}

	return 0;

exit_platform:
	if (host->pdata && host->pdata->exit)
		host->pdata->exit(dev, mmc);
	return ret;
}

static void pxamci_cancel_watchdog(void *data)
{
	struct pxamci_host *host = data;

	cancel_delayed_work_sync(&host->data_watchdog);
	cancel_work_sync(&host->command_work);
	cancel_work_sync(&host->data_work);
	cancel_work_sync(&host->request_work);
}

static int pxamci_terminate_dma(void *data, bool tx)
{
	struct pxamci_host *host = data;
	struct dma_chan *chan = tx ? host->dma_chan_tx : host->dma_chan_rx;

	return dmaengine_terminate_sync(chan);
}

static const struct pxamci_quiesce_ops pxamci_quiesce_ops = {
	.cancel_work = pxamci_cancel_watchdog,
	.terminate_dma = pxamci_terminate_dma,
};

static int pxamci_quiesce_dma(struct pxamci_host *host)
{
	/* Stop recovery first, then drain anything a racing callback queued. */
	return pxamci_quiesce_sequence(&pxamci_quiesce_ops, host);
}

static void pxamci_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);

	if (mmc) {
		struct pxamci_host *host = mmc_priv(mmc);
		int ret;

		mmc_remove_host(mmc);

		if (host->pdata && host->pdata->exit)
			host->pdata->exit(&pdev->dev, mmc);

		host->imask = pxamci_irq_mask_all(host->pxa25x);
		if (!READ_ONCE(host->quarantined))
			writel(host->imask, host->base + MMC_I_MASK);
		synchronize_irq(host->irq);

		do {
			ret = pxamci_quiesce_dma(host);
			if (!pxamci_teardown_can_continue(ret)) {
				dev_err_ratelimited(&pdev->dev,
						    "waiting for DMA quiesce during removal: %d\n",
						    ret);
				msleep(PXAMCI_WATCHDOG_INTERVAL_MS);
			}
		} while (!pxamci_teardown_can_continue(ret));
		if (!READ_ONCE(host->fatal_error))
			pxamci_stop_clock(host);
		pxamci_disable_functional_clock(host);
		platform_set_drvdata(pdev, NULL);
	}
}

static int pxamci_suspend(struct device *dev)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct pxamci_host *host;
	unsigned long flags;
	bool active;
	int ret;

	if (!mmc)
		return 0;

	host = mmc_priv(mmc);
	spin_lock_irqsave(&host->lock, flags);
	active = !!host->mrq;
	spin_unlock_irqrestore(&host->lock, flags);
	if (active) {
		dev_err(dev, "refusing suspend with an active request\n");
		return -EBUSY;
	}

	ret = pxamci_quiesce_dma(host);
	if (ret)
		return ret;

	if (READ_ONCE(host->fatal_error)) {
		pxamci_quarantine_host(host);
		return 0;
	}

	return pxamci_stop_clock(host);
}

static int pxamci_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(pxamci_pm_ops, pxamci_suspend, pxamci_resume);

static struct platform_driver pxamci_driver = {
	.probe		= pxamci_probe,
	.remove		= pxamci_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.pm	= &pxamci_pm_ops,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = of_match_ptr(pxa_mmc_dt_ids),
	},
};

module_platform_driver(pxamci_driver);

MODULE_DESCRIPTION("PXA Multimedia Card Interface Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pxa2xx-mci");
