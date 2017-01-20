#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
 .name = KBUILD_MODNAME,
 .init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
 .exit = cleanup_module,
#endif
 .arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x819eaac0, "module_layout" },
	{ 0x403f9529, "gpio_request_one" },
	{ 0x729a72b2, "omap_mux_init_signal" },
	{ 0x27e1a049, "printk" },
	{ 0xfaef0ed, "__tasklet_schedule" },
	{ 0x82f776b7, "gpio_export" },
	{ 0x9545af6d, "tasklet_init" },
	{ 0xd6b8e852, "request_threaded_irq" },
	{ 0x82072614, "tasklet_kill" },
	{ 0x11f447ce, "__gpio_to_irq" },
	{ 0xfe990052, "gpio_free" },
	{ 0xf20dabd8, "free_irq" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "C51F588880D219C3AE5815E");
