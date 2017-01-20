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
    {"spi0_sclk.spi0_sclk", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL | AM33XX_PULL_UP | AM33XX_INPUT_EN,
                            NOT_GPIO, 0, "spi0_clk"},
    {"spi0_d0.spi0_d0", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL | AM33XX_PULL_UP | AM33XX_INPUT_EN,
                            NOT_GPIO, 0, "spi0_miso"},
    {"spi0_d1.spi0_d1", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL | AM33XX_PULL_UP | AM33XX_INPUT_EN,
                            NOT_GPIO, 0, "spi0_mosi"},
    {"spi0_cs0.spi0_cs0", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL | AM33XX_INPUT_EN,
                            NOT_GPIO, 0, "spi0_cs0"},
    //{"spi0_cs0.gpio0_5", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP, GPIO_TO_PIN(0, 5), GPIOF_OUT_INIT_HIGH, "spi0_cs0"},
    {NULL, 0},
};


static const struct omap_spi_platform_data spidev0_platform_data = {
    .gpio_cs  = GPIO_TO_PIN(0, 5),
};


static struct spi_board_info __initdata am335x_spi0_slave_info[] = {

    {
        .modalias   = "spidev",
        .irq        = GPIO_TO_PIN(3,7),
        .max_speed_hz   = 48*1000*1000,
        .bus_num    = 1,
        .chip_select    = 0,
        //.platform_data = &spidev0_platform_data,
    },
};


/* Module pin mux for SPI fash */
static struct pinmux_config spi1_pin_mux[] = {
    {"mcasp0_aclkx.spi1_sclk", OMAP_MUX_MODE3 | AM33XX_PULL_ENBL |AM33XX_PULL_UP | AM33XX_INPUT_EN,
                            NOT_GPIO, 0, "spi1_clk"},
    {"mcasp0_fsx.spi1_d0", OMAP_MUX_MODE3 | AM33XX_PULL_ENBL | AM33XX_PULL_UP | AM33XX_INPUT_EN,
                            NOT_GPIO, 0, "spi1_miso"},
    {"mcasp0_axr0.spi1_d1", OMAP_MUX_MODE3 | AM33XX_PULL_ENBL | AM33XX_PULL_UP | AM33XX_INPUT_EN,
                            NOT_GPIO, 0, "spi1_mosi"},
    {"mcasp0_ahclkr.spi1_cs0", OMAP_MUX_MODE3 | AM33XX_PULL_ENBL | AM33XX_INPUT_EN,
                            NOT_GPIO, 0, "spi1_cs0"},
    //{"mcasp0_ahclkr.gpio3_17", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP},
    {NULL, 0},
};

static const struct omap_spi_platform_data spidev1_platform_data = {
    .gpio_cs  = GPIO_TO_PIN(3, 17),
};

static struct spi_board_info __initdata am335x_spi1_slave_info[] = {

    {
        .modalias   = "spidev",
        .irq        = GPIO_TO_PIN(1,15),
        .max_speed_hz   = 48*1000*1000,
        .bus_num    = 2,
        .chip_select = 0,
        //.platform_data = &spidev1_platform_data,
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
    {"lcd_hsync.gpio2_23", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP,GPIO_TO_PIN(2, 23),
                            GPIOF_OUT_INIT_LOW, "heartbeet_led"},/*led2 */
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

    {"lcd_data0.gpio2_6", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP, GPIO_TO_PIN(2, 6),
                            GPIOF_OUT_INIT_LOW, "beep_old"},
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
    {"emu0.gpio3_7",OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP, GPIO_TO_PIN(3, 7),
                            GPIOF_IN, "secure_read_irq"},
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

    {"gpmc_a7.gpio1_23",OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP, GPIO_TO_PIN(1, 23),
                            GPIOF_IN, "gsm_wake"}, //
    {"gpmc_a8.gpio1_24",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP,GPIO_TO_PIN(1, 24),
                            GPIOF_OUT_INIT_HIGH, "gsm_reset"},
    {"gpmc_a9.gpio1_25",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP,GPIO_TO_PIN(1, 25),
                            GPIOF_OUT_INIT_LOW, "gsm_disable"},
    {"gpmc_a10.gpio1_26",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP,GPIO_TO_PIN(1, 26),
                            GPIOF_OUT_INIT_HIGH, "gsm_power_en"},

    {"gpmc_ben1.gpio1_28",OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP,GPIO_TO_PIN(1, 28),
                            GPIOF_IN, "gpio_int1"},
    {"mcasp0_ahclkx.gpio3_21",OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP,GPIO_TO_PIN(3, 21),
                            GPIOF_IN, "gpio_int2"},
    {"gpmc_csn3.gpio2_0",OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP,GPIO_TO_PIN(2, 0),
                            GPIOF_IN, "gpio_int3"},

    {"gpmc_a0.gpio1_16",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP,GPIO_TO_PIN(1, 16),
                            GPIOF_OUT_INIT_LOW, "gpio_out1"},
    {"gpmc_a1.gpio1_17",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP,GPIO_TO_PIN(1, 17),
                            GPIOF_OUT_INIT_LOW, "gpio_out2"},
    {"gpmc_a2.gpio1_18",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP,GPIO_TO_PIN(1, 18),
                            GPIOF_OUT_INIT_LOW, "gpio_out3"},

    {"gpmc_a3.gpio1_19",OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP,GPIO_TO_PIN(1, 19),
                            GPIOF_IN, "sync_data_in"},
    {"gpmc_a4.gpio1_20",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP,GPIO_TO_PIN(1, 20),
                            GPIOF_OUT_INIT_LOW, "sync_mode"},
    {"xdma_event_intr0.clkout1",OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT_PULLUP, NOT_GPIO, 0, "sync_out"},

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
};

static void am335x_wifi_gpio_init(void)
{
    setup_pin_mux(wifi_pin_mux);
}

static struct pinmux_config i2c3_pin_mux[] = {
    {"uart1_ctsn.i2c2_sda", OMAP_MUX_MODE3|AM33XX_INPUT_EN|AM33XX_PIN_OUTPUT, NOT_GPIO, 0, "i2c2_sda"},

    {"uart1_rtsn.i2c2_scl", OMAP_MUX_MODE3|AM33XX_INPUT_EN|AM33XX_PIN_OUTPUT, NOT_GPIO, 0, "i2c2_scl"},

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

static int __init g90b_iomux_init(void)
{
    printk("G90b iomux init, driver version = %s.\n", "1.0.2");

	am335x_beep_init();
	am335x_common_gpio_init();
	gpio_led_init();

    am335x_wifi_gpio_init();
    am335x_evm_i2c_init();
    am335x_spi_init();

    return 0;
}

static void __exit g90b_iomux_exit(void)
{
    printk("g90b_ext rmmod not support, you need reboot the system.\n");
}

module_init(g90b_iomux_init);
module_exit(g90b_iomux_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("R.wen");

