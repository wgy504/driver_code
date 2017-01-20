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
	{ 0xb431a21b, "tty_unregister_driver" },
	{ 0xc6ad0bd4, "tty_unregister_device" },
	{ 0x67b27ec1, "tty_std_termios" },
	{ 0x2861648, "put_tty_driver" },
	{ 0x70ad0658, "tty_register_driver" },
	{ 0xc834e714, "tty_set_operations" },
	{ 0x99c665b6, "alloc_tty_driver" },
	{ 0x6c5b2360, "tty_flip_buffer_push" },
	{ 0x4ddc3f6b, "tty_insert_flip_string_fixed_flag" },
	{ 0x27e1a049, "printk" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "43AAAFFD7603B023B1ACE94");
