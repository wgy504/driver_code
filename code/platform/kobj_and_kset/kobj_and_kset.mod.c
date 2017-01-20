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
	{ 0x5179dd50, "kset_unregister" },
	{ 0xd1e02867, "kobject_put" },
	{ 0x7258431b, "kobject_del" },
	{ 0xc5ae0182, "malloc_sizes" },
	{ 0xb6eecd34, "kset_create_and_add" },
	{ 0xc7a15a24, "kobject_init_and_add" },
	{ 0x8836fca7, "kmem_cache_alloc" },
	{ 0x91715312, "sprintf" },
	{ 0x42224298, "sscanf" },
	{ 0xe2d5255a, "strcmp" },
	{ 0x37a0cba, "kfree" },
	{ 0xe914e41e, "strcpy" },
	{ 0x27e1a049, "printk" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "82DD61AF40E0572408DFA37");
