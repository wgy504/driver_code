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
	{ 0x3e4c6428, "bus_register" },
	{ 0x97255bdf, "strlen" },
	{ 0xc059d39c, "device_register" },
	{ 0x27e1a049, "printk" },
	{ 0x84b183ae, "strncmp" },
	{ 0x8686d1c3, "bus_unregister" },
	{ 0xde46da08, "bus_create_file" },
	{ 0xf25f1fe, "device_unregister" },
	{ 0xb81960ca, "snprintf" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "FB84D3F5279EC3AF34713CC");
