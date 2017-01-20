/*
 * linux/arch/arm/mach-omap2/gpmc-fpga.c
 *
 * Copyright (C) 2013 Guanghong Xu, rwen
 *
 * Modified from linux/arch/arm/mach-omap2/gpmc-smsc911x.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>

#include <plat/board.h>
#include <plat/gpmc.h>


#define GPMC_EXT_CONFIG1	0x00000000
#define GPMC_EXT_CONFIG2	0x001E1E82
#define GPMC_EXT_CONFIG3	0x001E1E00
#define GPMC_EXT_CONFIG4	0x12841004
#define GPMC_EXT_CONFIG5	0x0F1A1E1E
#define GPMC_EXT_CONFIG6	0x1C000F00
//#define GPMC_EXT_CONFIG7	0x00000f6c
#define GPMC_EXT_CONFIG7	0x00000f40

static void GPMC_EXT_setime(int cs)
{
   gpmc_cs_write_reg(cs, GPMC_CS_CONFIG1, GPMC_EXT_CONFIG1);
   gpmc_cs_write_reg(cs, GPMC_CS_CONFIG2, GPMC_EXT_CONFIG2);
   gpmc_cs_write_reg(cs, GPMC_CS_CONFIG3, GPMC_EXT_CONFIG3);
   gpmc_cs_write_reg(cs, GPMC_CS_CONFIG4, GPMC_EXT_CONFIG4);
   gpmc_cs_write_reg(cs, GPMC_CS_CONFIG5, GPMC_EXT_CONFIG5);
   gpmc_cs_write_reg(cs, GPMC_CS_CONFIG6, GPMC_EXT_CONFIG6);
   /*gpmc_cs_write_reg(cs, GPMC_CS_CONFIG7, GPMC_EXT_CONFIG7);*/
}


static struct resource gpmc_ext_resources[] = {
	{
		.name	= "gpmc_ext_mem",
		.flags	= IORESOURCE_MEM,
	},
	{   /* irq0*/
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
	},
	{   /* irq1*/
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
};


static struct platform_device gpmc_ext_device = {
	.name   = "gpmc_ext",
	.id	    = -1,
	.num_resources	= ARRAY_SIZE(gpmc_ext_resources),
	.resource	= gpmc_ext_resources,
	.dev		= {
		.platform_data = NULL,
	},
};

void gpmc_ext_init(int cs, int irq0, int irq1)
{
	unsigned long cs_mem_base;
	int ret;

	pr_info("Registering gpmc_ext on CS%d\n", cs);

	gpmc_ext_device.id = cs;

	cs_mem_base = 0;
	if (gpmc_cs_request(cs, SZ_16M, &cs_mem_base) < 0) {
		pr_err("Cannot request CS%d of gpmc_ext\n",  cs);
		return;
	}
	gpmc_ext_resources[0].start = cs_mem_base + 0x0;
	gpmc_ext_resources[0].end   = cs_mem_base + SZ_16M - 1;

	printk("gpmc_ext_resources[0].start=%08x\n", gpmc_ext_resources[0].start);

	if (irq0 > 0)
	{
		ret = gpio_request_one(irq0, GPIOF_IN, "gpmc_ext irq0");
		if (ret == 0)
			gpio_export(irq0, 0);
		else {
			pr_err("could not obtain gpio %d, for gpmc ext[%d] irq\n", irq0, cs);
			return;
		}

		gpmc_ext_resources[1].start = gpio_to_irq(irq0);
		gpmc_ext_resources[1].end = 0;
	}

	if (irq1 > 0)
	{
		ret = gpio_request_one(irq1, GPIOF_IN, "gpmc_ext irq1");
		if (ret == 0)
			gpio_export(irq0, 0);
		else {
			pr_err("could not obtain gpio %d, for gpmc ext[%d] irq\n", irq1, cs);
			return;
		}

		gpmc_ext_resources[2].start = gpio_to_irq(irq1);
		gpmc_ext_resources[2].end = 0;
	}

	ret = platform_device_register(&gpmc_ext_device);

	GPMC_EXT_setime(cs);
	return;
}


