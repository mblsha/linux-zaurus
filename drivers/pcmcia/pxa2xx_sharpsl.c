// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sharp SL-C7xx Series PCMCIA routines
 *
 * Copyright (c) 2004-2005 Richard Purdie
 *
 * Based on Sharp's 2.4 kernel patches and pxa2xx_mainstone.c
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include <asm/mach-types.h>
#include <asm/irq.h>
#include <asm/hardware/scoop.h>

#include <pcmcia/soc_common.h>

#define	NO_KEEP_VS 0x0001
#define SCOOP_DEV platform_scoop_config->devs

struct sharpsl_pcmcia_dt {
	struct platform_device *scoop;
	struct platform_device *socket;
	struct gpio_desc *cd_gpio;
	struct scoop_pcmcia_dev scoopdev;
	struct pcmcia_low_level ops;
};

static struct sharpsl_pcmcia_dt *sharpsl_pcmcia_dt_data(
		struct soc_pcmcia_socket *skt)
{
	struct device *socket = skt->socket.dev.parent;

	if (!socket->parent ||
	    !of_device_is_compatible(socket->parent->of_node,
				     "sharp,sl-c860-pcmcia"))
		return NULL;

	return dev_get_drvdata(socket->parent);
}

static struct scoop_pcmcia_dev *sharpsl_pcmcia_scoopdev(
		struct soc_pcmcia_socket *skt)
{
	struct sharpsl_pcmcia_dt *dt = sharpsl_pcmcia_dt_data(skt);

	if (dt)
		return &dt->scoopdev;

	return &SCOOP_DEV[skt->nr];
}

static void sharpsl_pcmcia_set_power(struct soc_pcmcia_socket *skt,
				     struct device *scoop, unsigned short cpr)
{
	if (!sharpsl_pcmcia_dt_data(skt) && platform_scoop_config &&
	    platform_scoop_config->power_ctrl)
		platform_scoop_config->power_ctrl(scoop, cpr, skt->nr);
	else
		write_scoop_reg(scoop, SCOOP_CPR, cpr);
}

static void sharpsl_pcmcia_init_reset(struct soc_pcmcia_socket *skt)
{
	struct scoop_pcmcia_dev *scoopdev = sharpsl_pcmcia_scoopdev(skt);

	reset_scoop(scoopdev->dev);

	/* Shared power controls need to be handled carefully */
	sharpsl_pcmcia_set_power(skt, scoopdev->dev, 0x0000);

	scoopdev->keep_vs = NO_KEEP_VS;
	scoopdev->keep_rd = 0;
}

static int sharpsl_pcmcia_hw_init(struct soc_pcmcia_socket *skt)
{
	struct sharpsl_pcmcia_dt *dt = sharpsl_pcmcia_dt_data(skt);
	struct scoop_pcmcia_dev *scoopdev = sharpsl_pcmcia_scoopdev(skt);

	if (dt) {
		skt->stat[SOC_STAT_CD].desc = dt->cd_gpio;
		skt->stat[SOC_STAT_CD].name = "PCMCIA0 CD";
	} else if (scoopdev->cd_irq >= 0) {
		skt->stat[SOC_STAT_CD].irq = scoopdev->cd_irq;
		skt->stat[SOC_STAT_CD].name = scoopdev->cd_irq_str;
	}

	skt->socket.pci_irq = scoopdev->irq;

	return 0;
}

static void sharpsl_pcmcia_socket_state(struct soc_pcmcia_socket *skt,
				    struct pcmcia_state *state)
{
	unsigned short cpr, csr;
	struct scoop_pcmcia_dev *scoopdev = sharpsl_pcmcia_scoopdev(skt);
	struct device *scoop = scoopdev->dev;

	cpr = read_scoop_reg(scoop, SCOOP_CPR);

	write_scoop_reg(scoop, SCOOP_IRM, 0x00FF);
	write_scoop_reg(scoop, SCOOP_ISR, 0x0000);
	write_scoop_reg(scoop, SCOOP_IRM, 0x0000);
	csr = read_scoop_reg(scoop, SCOOP_CSR);
	if (csr & 0x0004) {
		/* card eject */
		write_scoop_reg(scoop, SCOOP_CDR, 0x0000);
		scoopdev->keep_vs = NO_KEEP_VS;
	}
	else if (!(scoopdev->keep_vs & NO_KEEP_VS)) {
		/* keep vs1,vs2 */
		write_scoop_reg(scoop, SCOOP_CDR, 0x0000);
		csr |= scoopdev->keep_vs;
	}
	else if (cpr & 0x0003) {
		/* power on */
		write_scoop_reg(scoop, SCOOP_CDR, 0x0000);
		scoopdev->keep_vs = (csr & 0x00C0);
	}
	else {
		/* card detect */
	        if ((machine_is_spitz() || machine_is_borzoi()) && skt->nr == 1) {
	                write_scoop_reg(scoop, SCOOP_CDR, 0x0000);
	        } else {
		        write_scoop_reg(scoop, SCOOP_CDR, 0x0002);
	        }
	}

	state->detect = (csr & 0x0004) ? 0 : 1;
	state->ready  = (csr & 0x0002) ? 1 : 0;
	state->bvd1   = (csr & 0x0010) ? 1 : 0;
	state->bvd2   = (csr & 0x0020) ? 1 : 0;
	state->wrprot = (csr & 0x0008) ? 1 : 0;
	state->vs_3v  = (csr & 0x0040) ? 0 : 1;
	state->vs_Xv  = (csr & 0x0080) ? 0 : 1;

	if ((cpr & 0x0080) && ((cpr & 0x8040) != 0x8040)) {
		printk(KERN_ERR "sharpsl_pcmcia_socket_state(): CPR=%04X, Low voltage!\n", cpr);
	}
}


static int sharpsl_pcmcia_configure_socket(struct soc_pcmcia_socket *skt,
				       const socket_state_t *state)
{
	unsigned long flags;
	struct scoop_pcmcia_dev *scoopdev = sharpsl_pcmcia_scoopdev(skt);
	struct device *scoop = scoopdev->dev;

	unsigned short cpr, ncpr, ccr, nccr, mcr, nmcr, imr, nimr;

	switch (state->Vcc) {
	case	0:  	break;
	case 	33: 	break;
	case	50: 	break;
	default:
		 printk(KERN_ERR "sharpsl_pcmcia_configure_socket(): bad Vcc %u\n", state->Vcc);
		 return -1;
	}

	if ((state->Vpp!=state->Vcc) && (state->Vpp!=0)) {
		printk(KERN_ERR "CF slot cannot support Vpp %u\n", state->Vpp);
		return -1;
	}

	local_irq_save(flags);

	nmcr = (mcr = read_scoop_reg(scoop, SCOOP_MCR)) & ~0x0010;
	ncpr = (cpr = read_scoop_reg(scoop, SCOOP_CPR)) & ~0x0083;
	nccr = (ccr = read_scoop_reg(scoop, SCOOP_CCR)) & ~0x0080;
	nimr = (imr = read_scoop_reg(scoop, SCOOP_IMR)) & ~0x003E;

	if ((machine_is_spitz() || machine_is_borzoi() || machine_is_akita()) && skt->nr == 0) {
	        ncpr |= (state->Vcc == 33) ? 0x0002 :
		        (state->Vcc == 50) ? 0x0002 : 0;
	} else {
	        ncpr |= (state->Vcc == 33) ? 0x0001 :
		        (state->Vcc == 50) ? 0x0002 : 0;
	}
	nmcr |= (state->flags&SS_IOCARD) ? 0x0010 : 0;
	ncpr |= (state->flags&SS_OUTPUT_ENA) ? 0x0080 : 0;
	nccr |= (state->flags&SS_RESET)? 0x0080: 0;
	nimr |=	((skt->status&SS_DETECT) ? 0x0004 : 0)|
			((skt->status&SS_READY)  ? 0x0002 : 0)|
			((skt->status&SS_BATDEAD)? 0x0010 : 0)|
			((skt->status&SS_BATWARN)? 0x0020 : 0)|
			((skt->status&SS_STSCHG) ? 0x0010 : 0)|
			((skt->status&SS_WRPROT) ? 0x0008 : 0);

	if (!(ncpr & 0x0003)) {
		scoopdev->keep_rd = 0;
	} else if (!scoopdev->keep_rd) {
		if (nccr & 0x0080)
			scoopdev->keep_rd = 1;
		else
			nccr |= 0x0080;
	}

	if (mcr != nmcr)
		write_scoop_reg(scoop, SCOOP_MCR, nmcr);
	if (cpr != ncpr) {
		sharpsl_pcmcia_set_power(skt, scoop, ncpr);
	}
	if (ccr != nccr)
		write_scoop_reg(scoop, SCOOP_CCR, nccr);
	if (imr != nimr)
		write_scoop_reg(scoop, SCOOP_IMR, nimr);

	local_irq_restore(flags);

	return 0;
}

static void sharpsl_pcmcia_socket_init(struct soc_pcmcia_socket *skt)
{
	sharpsl_pcmcia_init_reset(skt);

	/* Enable interrupt */
	write_scoop_reg(sharpsl_pcmcia_scoopdev(skt)->dev,
			SCOOP_IMR, 0x00C0);
	write_scoop_reg(sharpsl_pcmcia_scoopdev(skt)->dev,
			SCOOP_MCR, 0x0101);
	sharpsl_pcmcia_scoopdev(skt)->keep_vs = NO_KEEP_VS;
}

static void sharpsl_pcmcia_socket_suspend(struct soc_pcmcia_socket *skt)
{
	sharpsl_pcmcia_init_reset(skt);
}

static struct pcmcia_low_level sharpsl_pcmcia_ops = {
	.owner                  = THIS_MODULE,
	.hw_init                = sharpsl_pcmcia_hw_init,
	.socket_state           = sharpsl_pcmcia_socket_state,
	.configure_socket       = sharpsl_pcmcia_configure_socket,
	.socket_init            = sharpsl_pcmcia_socket_init,
	.socket_suspend         = sharpsl_pcmcia_socket_suspend,
	.first                  = 0,
	.nr                     = 0,
};

#ifdef CONFIG_SA1100_COLLIE
#include "sa11xx_base.h"

int pcmcia_collie_init(struct device *dev)
{
       int ret = -ENODEV;

       if (machine_is_collie())
               ret = sa11xx_drv_pcmcia_probe(dev, &sharpsl_pcmcia_ops, 0, 1);

       return ret;
}

#else

static struct platform_device *sharpsl_pcmcia_device;

static int sharpsl_pcmcia_dt_probe(struct platform_device *pdev)
{
	struct sharpsl_pcmcia_dt *dt;
	struct device_node *scoop_node;
	int ret;

	dt = devm_kzalloc(&pdev->dev, sizeof(*dt), GFP_KERNEL);
	if (!dt)
		return -ENOMEM;

	scoop_node = of_parse_phandle(pdev->dev.of_node, "sharp,scoop", 0);
	if (!scoop_node)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "missing sharp,scoop phandle\n");

	dt->scoop = of_find_device_by_node(scoop_node);
	of_node_put(scoop_node);
	if (!dt->scoop)
		return dev_err_probe(&pdev->dev, -EPROBE_DEFER,
				     "SCOOP device is not registered\n");
	if (!platform_get_drvdata(dt->scoop)) {
		put_device(&dt->scoop->dev);
		return dev_err_probe(&pdev->dev, -EPROBE_DEFER,
				     "SCOOP driver is not ready\n");
	}

	dt->cd_gpio = devm_gpiod_get(&pdev->dev, "cd", GPIOD_IN);
	if (IS_ERR(dt->cd_gpio)) {
		ret = dev_err_probe(&pdev->dev, PTR_ERR(dt->cd_gpio),
				    "failed to acquire card-detect GPIO\n");
		goto err_put_scoop;
	}

	dt->scoopdev.dev = &dt->scoop->dev;
	dt->scoopdev.irq = platform_get_irq(pdev, 0);
	if (dt->scoopdev.irq < 0) {
		ret = dt->scoopdev.irq;
		goto err_put_scoop;
	}
	dt->scoopdev.cd_irq = -1;
	dt->scoopdev.keep_vs = NO_KEEP_VS;

	dt->ops = sharpsl_pcmcia_ops;
	dt->ops.first = 0;
	dt->ops.nr = 1;
	platform_set_drvdata(pdev, dt);

	dt->socket = platform_device_alloc("pxa2xx-pcmcia", -1);
	if (!dt->socket) {
		ret = -ENOMEM;
		goto err_clear_drvdata;
	}

	ret = platform_device_add_data(dt->socket, &dt->ops, sizeof(dt->ops));
	if (ret)
		goto err_put_socket;

	dt->socket->dev.parent = &pdev->dev;
	ret = platform_device_add(dt->socket);
	if (ret)
		goto err_put_socket;

	return 0;

err_put_socket:
	platform_device_put(dt->socket);
err_clear_drvdata:
	platform_set_drvdata(pdev, NULL);
err_put_scoop:
	put_device(&dt->scoop->dev);
	return ret;
}

static void sharpsl_pcmcia_dt_remove(struct platform_device *pdev)
{
	struct sharpsl_pcmcia_dt *dt = platform_get_drvdata(pdev);

	platform_device_unregister(dt->socket);
	put_device(&dt->scoop->dev);
}

static const struct of_device_id sharpsl_pcmcia_dt_of_match[] = {
	{ .compatible = "sharp,sl-c860-pcmcia" },
	{ }
};
MODULE_DEVICE_TABLE(of, sharpsl_pcmcia_dt_of_match);

static struct platform_driver sharpsl_pcmcia_dt_driver = {
	.probe = sharpsl_pcmcia_dt_probe,
	.remove = sharpsl_pcmcia_dt_remove,
	.driver = {
		.name = "sharp-sl-c860-pcmcia",
		.of_match_table = sharpsl_pcmcia_dt_of_match,
	},
};

static int __init sharpsl_pcmcia_init(void)
{
	int ret;

	ret = platform_driver_register(&sharpsl_pcmcia_dt_driver);
	if (ret)
		return ret;

	if (!platform_scoop_config)
		return 0;

	sharpsl_pcmcia_ops.nr = platform_scoop_config->num_devs;
	sharpsl_pcmcia_device = platform_device_alloc("pxa2xx-pcmcia", -1);

	if (!sharpsl_pcmcia_device) {
		ret = -ENOMEM;
		goto err_unregister_dt;
	}

	ret = platform_device_add_data(sharpsl_pcmcia_device,
			&sharpsl_pcmcia_ops, sizeof(sharpsl_pcmcia_ops));
	if (ret == 0) {
		sharpsl_pcmcia_device->dev.parent = platform_scoop_config->devs[0].dev;
		ret = platform_device_add(sharpsl_pcmcia_device);
	}

	if (ret) {
		platform_device_put(sharpsl_pcmcia_device);
		goto err_unregister_dt;
	}

	return 0;

err_unregister_dt:
	platform_driver_unregister(&sharpsl_pcmcia_dt_driver);
	return ret;
}

static void __exit sharpsl_pcmcia_exit(void)
{
	if (sharpsl_pcmcia_device)
		platform_device_unregister(sharpsl_pcmcia_device);
	platform_driver_unregister(&sharpsl_pcmcia_dt_driver);
}

fs_initcall(sharpsl_pcmcia_init);
module_exit(sharpsl_pcmcia_exit);
#endif

MODULE_DESCRIPTION("Sharp SL Series PCMCIA Support");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pxa2xx-pcmcia");
