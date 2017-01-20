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
	{ 0xf930dc8c, "my_bus_type" },
	{ 0x64d67bac, "my_bus_dev" },
	{ 0xf25f1fe, "device_unregister" },
	{ 0xd13e5b7a, "device_create_file" },
	{ 0xc059d39c, "device_register" },
	{ 0xf5ad8568, "dev_set_name" },
	{ 0x91715312, "sprintf" },
	{ 0x27e1a049, "printk" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=bus";


MODULE_INFO(srcversion, "C20C7603C9F98E93656B8FE");
