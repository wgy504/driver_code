#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <mach/hardware.h>
#include <mach/board-am335xevm.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <plat/omap_device.h>
#include <plat/irqs.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/emif.h>
#include <linux/leds.h>
#include <plat/i2c.h>
#include <plat/usb.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>

#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <mach/board-am335xevm.h>
#include <plat/omap-spi.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/poll.h>
#include <linux/sched.h>

#include "mux.h"

/* module pin mux structure */
struct pinmux_config {
	const char *string_name; /* signal name format */
	int val; /* Options for the mux register value */
	int gpio_num; //number
    int init_level;
    const char *my_name;
};


/*
* @pin_mux - single module pin-mux structure which defines pin-mux
*			details for all its pins.
*/
static void setup_pin_mux(struct pinmux_config *pin_mux)
{
	int i, ret;

	for (i = 0; pin_mux->string_name != NULL; pin_mux++)
    {
		omap_mux_init_signal(pin_mux->string_name, pin_mux->val);
        if(pin_mux->gpio_num < 0)
        {
            continue;
        }
        ret = gpio_request_one(pin_mux->gpio_num, pin_mux->init_level, pin_mux->my_name);
        if(ret != 0)
        {
            pr_err("Error requesting gpio %s (gpio%d_%d): %d\n", pin_mux->my_name,
                    pin_mux->gpio_num / 32, pin_mux->gpio_num % 32, ret);
        }
        else
        {
            gpio_export(pin_mux->gpio_num, 1);
        }
    }
}

/* Convert GPIO signal to GPIO pin number */
#define NOT_GPIO (-1)
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

#include <plat/omap-spi.h>

/* Module pin mux for SPI fash */
static struct pinmux_config spi0_pin_mux[] = {
    {"spi0_sclk.spi0_sclk", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL | AM33XX_PULL_UP | AM33XX_INPUT_EN, NOT_GPIO, 0, "spi0_clk"},
    {"spi0_d0.spi0_d0", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL | AM33XX_PULL_UP | AM33XX_INPUT_EN, NOT_GPIO, 0, "spi0_miso"},
    {"spi0_d1.spi0_d1", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL | AM33XX_PULL_UP | AM33XX_INPUT_EN, NOT_GPIO, 0, "spi0_mosi"},
    {"spi0_cs0.spi0_cs0", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL | AM33XX_INPUT_EN, NOT_GPIO, 0, "spi0_cs0"},
    //{"spi0_cs0.gpio0_5", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP, GPIO_TO_PIN(0, 5), GPIOF_OUT_INIT_HIGH, "spi0_cs0"},
    {NULL, 0},
};


static const struct omap_spi_platform_data spidev0_platform_data = {
    .gpio_cs  = GPIO_TO_PIN(0, 5),
};


static struct spi_board_info __initdata am335x_spi0_slave_info[] = {

    {
        .modalias   = "spidev",
        //.irq        = GPIO_TO_PIN(3, 7),
        .irq = GPIO_TO_PIN(1, 20),
        .max_speed_hz   = 48 * 1000 * 1000,
        .bus_num    = 1,
        .chip_select    = 0,
        .platform_data = NULL, //&spidev0_platform_data,
    },
};


/* Module pin mux for SPI fash */
static struct pinmux_config spi1_pin_mux[] = {
    {"mcasp0_aclkx.spi1_sclk", OMAP_MUX_MODE3 | AM33XX_PULL_ENBL |AM33XX_PULL_UP | AM33XX_INPUT_EN, NOT_GPIO, 0, "spi1_clk"},
    {"mcasp0_fsx.spi1_d0", OMAP_MUX_MODE3 | AM33XX_PULL_ENBL | AM33XX_PULL_UP | AM33XX_INPUT_EN, NOT_GPIO, 0, "spi1_miso"},
    {"mcasp0_axr0.spi1_d1", OMAP_MUX_MODE3 | AM33XX_PULL_ENBL | AM33XX_PULL_UP | AM33XX_INPUT_EN, NOT_GPIO, 0, "spi1_mosi"},
    {"mcasp0_ahclkr.spi1_cs0", OMAP_MUX_MODE3 | AM33XX_PULL_ENBL | AM33XX_INPUT_EN, NOT_GPIO, 0, "spi1_cs0"},
    //{"mcasp0_ahclkr.gpio3_17", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP, GPIO_TO_PIN(3, 17), GPIOF_OUT_INIT_HIGH, "spi1_cs0"},
    {NULL, 0},
};

static const struct omap_spi_platform_data spidev1_platform_data = {
    .gpio_cs  = GPIO_TO_PIN(3, 17),
};

static struct spi_board_info __initdata am335x_spi1_slave_info[] = {

    {
        .modalias   = "spidev",
        .irq        = GPIO_TO_PIN(1, 15),
        .max_speed_hz   = 48 * 1000 * 1000,
        .bus_num    = 2,
        .chip_select = 0,
        .platform_data = NULL, //&spidev1_platform_data,
    },
};

static void am335x_spi_init(void)
{
    setup_pin_mux(spi0_pin_mux);
    spi_register_board_info(am335x_spi0_slave_info,ARRAY_SIZE(am335x_spi0_slave_info));
    setup_pin_mux(spi1_pin_mux);
    spi_register_board_info(am335x_spi1_slave_info,ARRAY_SIZE(am335x_spi1_slave_info));
}

/* Module pin mux for led */
static struct pinmux_config gpio_led_mux[] = {
    //{"lcd_hsync.gpio2_23", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP,GPIO_TO_PIN(2, 23),
     //                       GPIOF_OUT_INIT_LOW, "heartbeet_led"},/*led2 */
    {"lcd_pclk.gpio2_24", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP,GPIO_TO_PIN(2, 24),
                            GPIOF_OUT_INIT_LOW, "user_led"},/*led3 */
	{NULL, 0},
};

static struct gpio_led gpio_leds[] = {
	{
		.name = "led_green",
		.gpio = GPIO_TO_PIN(2, 23),
		.default_trigger = "heartbeat",
    },
    {
        .name = "led_green",
        .gpio = GPIO_TO_PIN(2, 24),
        .default_trigger = NULL,
    },
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds_gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

static void gpio_led_init(void)
{
	int err;

	setup_pin_mux(gpio_led_mux);
	err = platform_device_register(&leds_gpio);
	if (err)
    {
		pr_err("failed to register gpio led device\n");
    }
}


/* Module pin mux for SPI fash */
static struct pinmux_config beep_pin_mux[] = {
	{"lcd_data8.gpio2_14", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP, GPIO_TO_PIN(2, 14),
                            GPIOF_OUT_INIT_LOW, "beep"},

    //{"lcd_data0.gpio2_6", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP, GPIO_TO_PIN(2, 6),
                            //GPIOF_OUT_INIT_LOW, "beep_old"},
	{NULL, 0},
};

static void am335x_beep_init(void)
{
	setup_pin_mux(beep_pin_mux);
}

/* Module pin mux for common gpio */
static struct pinmux_config common_gpio_pin_mux[] = {
	{"gpmc_ad14.gpio1_14", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT, GPIO_TO_PIN(1, 14),
                            GPIOF_OUT_INIT_LOW, "rfid_power_en"},
    {"gpmc_ad13.gpio1_13", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT, GPIO_TO_PIN(1, 13),
                            GPIOF_OUT_INIT_LOW, "rfid_reset"},
    {"mii1_crs.gpio3_1",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT, GPIO_TO_PIN(3, 1),
                            GPIOF_OUT_INIT_LOW, "secure_power_en"},
    //{"emu0.gpio3_7",OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP, GPIO_TO_PIN(3, 7), GPIOF_IN, "secure_read_irq"},
    {"emu0.gpio3_7",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP, GPIO_TO_PIN(1, 24), GPIOF_OUT_INIT_LOW, "sync_mode"},

    {"emu1.gpio3_8",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP, GPIO_TO_PIN(3, 8),
                            GPIOF_OUT_INIT_HIGH, "secure_write_irq"},
    {"gpmc_ad15.gpio1_15",OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP, GPIO_TO_PIN(1, 15),
                            GPIOF_IN, "rfid_read_irq"},
    {"mcasp0_fsr.gpio3_19",OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP, GPIO_TO_PIN(3, 19),
                            GPIOF_IN, "secure_error_status"},
    {"gpmc_ad12.gpio1_12",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP, GPIO_TO_PIN(1, 12),
                            GPIOF_OUT_INIT_HIGH, "secure_reset"},

    {"lcd_ac_bias.gpio2_25",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP, GPIO_TO_PIN(2, 25),
                            GPIOF_OUT_INIT_LOW, "uart5_mode"},

	/*for test v2 board*/
	{"gpmc_a4.gpio1_20",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP,GPIO_TO_PIN(1, 20),
						 GPIOF_OUT_INIT_LOW, "gpio_out1"},

    //{"gpmc_a0.gpio1_16",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP,GPIO_TO_PIN(1, 16),
    //                        GPIOF_OUT_INIT_LOW, "gpio_out1"},
	{"gpmc_a1.gpio1_17",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP,GPIO_TO_PIN(1, 17),
                            GPIOF_OUT_INIT_LOW, "gpio_out2"},
    {"gpmc_a2.gpio1_18",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP,GPIO_TO_PIN(1, 18),
                            GPIOF_OUT_INIT_LOW, "gpio_out3"},

    //{"gpmc_a4.gpio1_20",OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP, GPIO_TO_PIN(1, 20), GPIOF_IN, "secure_read_irq"},

    //{"gpmc_a8.gpio1_24", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP, GPIO_TO_PIN(1, 24), GPIOF_OUT_INIT_HIGH, "sync_out"},

    {"xdma_event_intr0.gpio0_19", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP, GPIO_TO_PIN(0, 19), GPIOF_OUT_INIT_HIGH, "sync_out"},

    {"gpmc_a5.gpio1_21",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP,GPIO_TO_PIN(1, 21),
                            GPIOF_OUT_INIT_LOW, "fpga_power_or_wl_reg_on"},
    {"lcd_data6.gpio2_12",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP,GPIO_TO_PIN(2, 12),
                            GPIOF_OUT_INIT_LOW, "uart1_sel1"},
    {"lcd_data7.gpio2_13",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP,GPIO_TO_PIN(2, 13),
                            GPIOF_OUT_INIT_LOW, "uart2_sel2"},

    {"gpmc_a11.gpio1_27",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP,GPIO_TO_PIN(1, 27),
                            GPIOF_OUT_INIT_LOW, "psam_power_en"},

    {"lcd_data3.gpio2_9",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP,GPIO_TO_PIN(2, 9),
                            GPIOF_OUT_INIT_LOW, "psam_reset"},

    {"mcasp0_axr1.gpio3_20",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP,GPIO_TO_PIN(3, 20),
                            GPIOF_OUT_INIT_LOW, "pa_en"},
    {NULL, 0},
};

static struct pinmux_config i2c3_pin_mux[] = {
    {"uart1_ctsn.i2c2_sda", OMAP_MUX_MODE3|AM33XX_INPUT_EN|AM33XX_PIN_OUTPUT, NOT_GPIO, 0, "i2c2_sda"},
    {"uart1_rtsn.i2c2_scl", OMAP_MUX_MODE3|AM33XX_INPUT_EN|AM33XX_PIN_OUTPUT, NOT_GPIO, 0, "i2c2_scl"},
    {NULL, 0},
};

static struct i2c_board_info __initdata am335x_i2c2_boardinfo[] = {
    {
        I2C_BOARD_INFO("lm75", 0x90 >> 1),
    },
};

static void am335x_evm_i2c_init(void)
{
    setup_pin_mux(i2c3_pin_mux);
    omap_register_i2c_bus(3, 400, am335x_i2c2_boardinfo,
                ARRAY_SIZE(am335x_i2c2_boardinfo));
}

static void am335x_common_gpio_init(void)
{
    setup_pin_mux(common_gpio_pin_mux);
}

static struct pinmux_config wifi_pin_mux[] =
{//mmc1
    {"gpmc_a6.gpio1_22",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP,GPIO_TO_PIN(1, 22),
                            GPIOF_OUT_INIT_LOW, "wifi_host_wk"},
    {"mcasp0_aclkr.gpio3_18",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP,GPIO_TO_PIN(3, 18),
                            GPIOF_OUT_INIT_HIGH, "wifi_en"},
    {"gpmc_csn1.mmc1_clk",OMAP_MUX_MODE2 | AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT,NOT_GPIO, 0, "wifi_clk"},
    {"gpmc_csn2.mmc1_cmd",OMAP_MUX_MODE2 | AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT,NOT_GPIO, 0, "wifi_cmd"},
    {"gpmc_ad8.mmc1_dat0",OMAP_MUX_MODE2 | AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT,NOT_GPIO, 0, "wifi_d0"},
    {"gpmc_ad9.mmc1_dat1",OMAP_MUX_MODE2 | AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT,NOT_GPIO, 0, "wifi_d1"},
    {"gpmc_ad10.mmc1_dat2",OMAP_MUX_MODE2 | AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT,NOT_GPIO, 0, "wifi_d2"},
    {"gpmc_ad11.mmc1_dat3",OMAP_MUX_MODE2 | AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT,NOT_GPIO, 0, "wifi_d3"},
    {NULL, 0},
};

static void am335x_wifi_gpio_init(void)
{
    setup_pin_mux(wifi_pin_mux);
}

static struct pinmux_config gsm_pin_mux[] =
{
    {"gpmc_a7.gpio1_23",OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP, GPIO_TO_PIN(1, 23),
                            GPIOF_IN, "gsm_wake"}, //
    //{"gpmc_a8.gpio1_24",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP,GPIO_TO_PIN(1, 24),
     //                       GPIOF_OUT_INIT_HIGH, "gsm_reset"},
    //{"gpmc_a9.gpio1_25",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP,GPIO_TO_PIN(1, 25),
                            //GPIOF_OUT_INIT_LOW, "gsm_disable"},
    {"gpmc_a10.gpio1_26",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP,GPIO_TO_PIN(1, 26),
                            GPIOF_OUT_INIT_HIGH, "gsm_power_en"},
    {NULL, 0},
};

static struct omap_musb_board_data musb_board_data =
{
    .interface_type = MUSB_INTERFACE_ULPI,
    .mode = (MUSB_HOST << 4) | MUSB_HOST,
    .power = 500,
    .instances = 1,
};

static void am335x_gsm_gpio_init(void)
{
    setup_pin_mux(gsm_pin_mux);
    //usb_musb_init(&musb_board_data);
}

static int  g90b_iomux_init(void)
{
    int ret;
    printk("G90b iomux init, driver version = %s.\n", "1.0.4");
    am335x_beep_init();
    am335x_common_gpio_init();
    gpio_led_init();

    am335x_wifi_gpio_init();
    am335x_gsm_gpio_init();

    ret = gpio_request_one(GPIO_TO_PIN(0, 15), GPIOF_IN, "uart1_txd");
    printk("gpio_request_one for uart1_txd ret = %d\n", ret);
    gpio_export(15, 1);

    am335x_evm_i2c_init();
    am335x_spi_init();

    return 0;
}

static void  g90b_iomux_exit(void)
{
    printk("g90b_ext rmmod not support, you need reboot the system.\n");
}


struct gpio_irq_desc
{
	const char *muxname;
	int muxflags;
    int pin;
	unsigned long id_mask;
    unsigned long flags;
    const char *name;
};

struct gpio_dev
{
	int opened;
    dev_t major;
    struct device *pdev;
    struct class *class;
    struct cdev cdev;
    struct spinlock lock;
    unsigned long sync_event;
	unsigned long gpio_event;
	wait_queue_head_t sync_wait;
    wait_queue_head_t gpio_wait;
};

struct gpio_irq_desc gs_irq_array[] =
{
	/*for test v2 board*/
    {"gpmc_a0.gpio1_16"    , OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP, GPIO_TO_PIN(1, 16), 1 <<  0, IRQF_SHARED | IRQF_TRIGGER_FALLING, "gpio_irq_0"},
    //{"gpmc_ben1.gpio1_28"    , OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP, GPIO_TO_PIN(1, 28), 1 <<  0, IRQF_SHARED | IRQF_TRIGGER_FALLING, "gpio_irq_0"},
    {"mcasp0_ahclkx.gpio3_21", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP, GPIO_TO_PIN(3, 21), 1 <<  1, IRQF_SHARED | IRQF_TRIGGER_FALLING, "gpio_irq_1"},
    {"gpmc_csn3.gpio2_0"     , OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP, GPIO_TO_PIN(2,  0), 1 <<  2, IRQF_SHARED | IRQF_TRIGGER_FALLING, "gpio_irq_2"},
	{"gpmc_a3.gpio1_19"      , OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP, GPIO_TO_PIN(1, 19), 1 << 31, IRQF_SHARED | IRQF_TRIGGER_FALLING, "sync_data_in"},
};

static struct gpio_dev gs_dev;

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	unsigned long flags;
	struct gpio_irq_desc *pirq = dev_id;
	spin_lock_irqsave(&gs_dev.lock, flags);
#if 0
    if(pirq->id_mask != (1 << 31))
    {
        gs_dev.gpio_event |= pirq->id_mask;
        wake_up_interruptible(&gs_dev.gpio_wait);
    }
    else
    {
        gs_dev.sync_event = 1;
        wake_up_interruptible(&gs_dev.sync_wait);
    }
#else
    if(pirq->id_mask == (1 << 31))
    {
        gs_dev.sync_event = 1;
        wake_up_interruptible(&gs_dev.sync_wait);
    }
    gs_dev.gpio_event |= pirq->id_mask;
	
    wake_up_interruptible(&gs_dev.gpio_wait);
	//printk("pirq->id_mask = %d\n", pirq->id_mask);
#endif
	spin_unlock_irqrestore(&gs_dev.lock, flags);
	return IRQ_HANDLED;
}


static int setup_gpio_irq(void)
{
	int i, ret, cnt;
	unsigned int irq;
	struct gpio_irq_desc *p;
	for(i = cnt = 0; i < ARRAY_SIZE(gs_irq_array); i ++)
	{
		p = &gs_irq_array[i];
		if(p->muxname == NULL)
		{
			break;
		}
		ret = omap_mux_init_signal(p->muxname, p->muxflags);
		if(ret != 0)
		{
            printk("init signal %s error\n", p->muxname);
			continue;
		}

		ret = gpio_request_one(p->pin, GPIOF_IN, p->name);
		if(ret != 0)
		{
            printk("gpio request %s error, pin = %d\n", p->muxname, p->pin);
			continue;
		}

		gpio_export(p->pin, 1);

		irq = gpio_to_irq(p->pin);
		ret = request_irq(irq, gpio_irq_handler, p->flags, p->name, (void *)p);
		if(ret != 0)
		{
            printk("request irq %s error, pin = %d, irq = %d\n", p->name, p->pin, irq);
			continue;
		}
		printk("init io for pin %d irq %s(%d) success\n", p->pin, p->name, irq);
		cnt ++;
	}
	return cnt > 0 ? 0 : -1;
}


static void unsetup_gpio_irq(void)
{
	int i;
	struct gpio_irq_desc *p;
	for(i = 0; i < ARRAY_SIZE(gs_irq_array); i ++)
	{
		p = &gs_irq_array[i];
		if(p->muxname == NULL)
		{
			break;
		}
		gpio_free(p->pin);
		free_irq(gpio_to_irq(p->pin), (void *)p);
	}
}


static int gpio_control_open(struct inode *inode, struct file *filp)
{
	unsigned long flags;
	spin_lock_irqsave(&gs_dev.lock, flags);
	if(gs_dev.opened > 0)
	{
		spin_unlock_irqrestore(&gs_dev.lock, flags);
		printk("%s file already open.\n", __FUNCTION__);
		return -EEXIST;
	}
	gs_dev.opened = 1;
	spin_unlock_irqrestore(&gs_dev.lock, flags);
    return 0;
}

static int gpio_control_release(struct inode *inode, struct file *filp)
{
	unsigned long flags;
	spin_lock_irqsave(&gs_dev.lock, flags);
	gs_dev.opened = 0;
	spin_unlock_irqrestore(&gs_dev.lock, flags);
    return 0;
}


#define GPIO_IRQ_SET_EDGE			_IOW('I', 0, int)
#define GPIO_IRQ_WAIT_SIGNAL		_IOW('I', 1, int)
#define GPIO_IRQ_GET_EVENT  		_IOR('I', 2, __u32)
#define GPIO_SET_UART1_TXD_INPUT   	_IOW('I', 3, int)
#define GPIO_SET_UART1_TXD_FUNCT	_IOW('I', 4, int)
#define GPIO_WAIT_SYNC              _IOR('I', 5, __u32)

#define IRQ_TRIGGER_RISING_EDGE    1
#define IRQ_TRIGGER_FALLING_EDGE   2
#define IRQ_TRIGGER_BOTH_EDGE      3

static long gpio_control_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
    switch(cmd)
    {
		case GPIO_IRQ_SET_EDGE:
		{
			int irq_num, irq_edge;
			ret |=  __get_user(irq_num, (int __user *)arg);
			ret |=  __get_user(irq_edge, (int __user *)arg + 1);
			if(ret != 0)
			{
				return -ENOTTY;
			}
			irq_num = gpio_to_irq(gs_irq_array[irq_num].pin);
		    if (irq_edge == IRQ_TRIGGER_RISING_EDGE)
			{
		        ret = irq_set_irq_type(irq_num, IRQF_TRIGGER_RISING);
		    }
			else if(irq_edge == IRQ_TRIGGER_FALLING_EDGE)
		    {
		       	ret = irq_set_irq_type(irq_num, IRQF_TRIGGER_FALLING);
		    }
			else if(irq_edge == IRQ_TRIGGER_BOTH_EDGE)
		    {
		        ret = irq_set_irq_type(irq_num, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
		    }
			else
			{
				return -ENOTTY;
			}
			return ret;
		}
		break;
		case GPIO_IRQ_WAIT_SIGNAL:
		{
		    arg = msecs_to_jiffies(arg);
            wait_event_interruptible_timeout(gs_dev.gpio_wait, gs_dev.gpio_event, arg);
			return gs_dev.gpio_event;
		}
		break;
		case GPIO_IRQ_GET_EVENT:
		{
			unsigned long flags;
			spin_lock_irqsave(&gs_dev.lock, flags);
			__put_user(gs_dev.gpio_event, (__u32 __user *)arg);
			printk("gs_dev.gpio_event = %d\n", gs_dev.gpio_event);
			gs_dev.gpio_event = 0;
			spin_unlock_irqrestore(&gs_dev.lock, flags);
			return 0;
		}
		break;
		case GPIO_SET_UART1_TXD_INPUT:
		{
            return omap_mux_init_signal("uart1_txd.gpio0_15", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP);
		}
		break;
		case GPIO_SET_UART1_TXD_FUNCT:
        {
            return omap_mux_init_signal("uart1_txd.uart1_txd", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL);
        }
		break;
        case GPIO_WAIT_SYNC:
        {
            gs_dev.sync_event = 0;
            arg = msecs_to_jiffies(arg);
            wait_event_interruptible_timeout(gs_dev.sync_wait, gs_dev.sync_event, arg);
            return !gs_dev.sync_event;
        }
        break;
		default:
        {
            return -ENOTTY;
        }
    }
    return 0;
}

static unsigned int gpio_control_poll (struct file *file, struct poll_table_struct *table)
{
	unsigned int msk = 0;
	unsigned long flags;
	poll_wait(file, &gs_dev.gpio_wait, table);
	spin_lock_irqsave(&gs_dev.lock, flags);
	if(gs_dev.gpio_event)
	{
		msk = POLLIN | POLLRDNORM;
	}
	spin_unlock_irqrestore(&gs_dev.lock, flags);
	return msk;
}

static struct file_operations gpio_control_fops =
{
    .owner = THIS_MODULE,
    .open = gpio_control_open,
    .release = gpio_control_release,
    .unlocked_ioctl = gpio_control_ioctl,
    .llseek = no_llseek,
    .poll = gpio_control_poll,
};



int __init gpio_control_init(void)
{
    int result,devno;
	struct gpio_dev *pdev = &gs_dev;
    memset(pdev, 0, sizeof(struct gpio_dev));

    printk("\n\n\n\n\n%s start\n",__FUNCTION__);
	g90b_iomux_init();

    result = setup_gpio_irq();
    if(result != 0)
    {
        printk("%s setup_gpio_irq error\n", __FUNCTION__);
        return result;
    }

	result = alloc_chrdev_region(&pdev->major, 0, 1, "gpio_control");
	if(result != 0)
	{
		printk("%s: alloc_chrdev_region failed.\n", __FUNCTION__);
    	goto error1;
	}

	pdev->major = MAJOR(pdev->major);

	printk("pdev->major = %d\n", pdev->major);
	spin_lock_init(&pdev->lock);
	init_waitqueue_head(&pdev->gpio_wait);
    init_waitqueue_head(&pdev->sync_wait);

    cdev_init(&pdev->cdev, &gpio_control_fops);
    pdev->cdev.owner = THIS_MODULE;
    pdev->cdev.ops = &gpio_control_fops;

    devno = MKDEV(pdev->major, 0);

    result = cdev_add(&pdev->cdev, devno, 1);
    if(result != 0)
    {
        printk("%s cdev_add error\n", __FUNCTION__);
        goto error2;
    }

    pdev->class = class_create(THIS_MODULE, "gpio_control");
    if(IS_ERR(pdev->class))
    {
        printk("%s class_create error\n", __FUNCTION__);
        goto error3;
    }

    pdev->pdev = device_create(pdev->class, NULL, devno, NULL, "gpio_control");
    if (IS_ERR(pdev->pdev))
    {
        printk("%s device_create error\n", __FUNCTION__);
        goto error4;
    }
    printk("%s init success\n", __FUNCTION__);
    return 0;

error4:
    class_destroy(pdev->class);
error3:
    cdev_del(&pdev->cdev);
error2:
    unregister_chrdev_region(devno, 1);
error1:
    unsetup_gpio_irq();
    return -ENOTTY;
}

void __exit gpio_control_cleanup(void)
{
    struct gpio_dev *dev = &gs_dev;
    int devno = MKDEV(dev->major,0);

	g90b_iomux_exit();

    device_destroy(dev->class, devno);
    class_destroy(dev->class);
    cdev_del(&dev->cdev);
    unregister_chrdev_region(devno, 1);
    unsetup_gpio_irq();
}


module_init(gpio_control_init);
module_exit(gpio_control_cleanup);

MODULE_AUTHOR("yumingl");
MODULE_LICENSE("Dual BSD/GPL");

