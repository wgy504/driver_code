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
	{ 0x45a55ec8, "__iounmap" },
	{ 0x47e0ffa5, "misc_deregister" },
	{ 0xb27e1dca, "platform_driver_unregister" },
	{ 0xb80f10f3, "platform_device_unregister" },
	{ 0x1b8a0f05, "platform_driver_register" },
	{ 0xc63280f2, "platform_device_register" },
	{ 0x5688fc65, "misc_register" },
	{ 0xbb72d4fe, "__put_user_1" },
	{ 0x28118cb6, "__get_user_1" },
	{ 0x27e1a049, "printk" },
	{ 0x40a6f522, "__arm_ioremap" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "DAB53CF805A56CE2C438163");
