obj-m += g90b_iomux.o
g90b_iomux-objs := g90b_iomux_config.o gpmc_ext.o
#usb-musb.o


#模块代码的当前目录，SUBDIRS需要
PWD := $(shell pwd)
#内核源码的位置变量
KPATH := $(PWD)/../../../linux-3.2-g90b_mini
CROSS_ARCH := ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
CC = $(CROSS_COMPILE)gcc

#这是真正的编译命令
default:
	$(MAKE) $(CROSS_ARCH) -C $(KPATH) SUBDIRS=$(PWD) modules

clean:
	rm *.ko *.mod.c *.bak .tmp_versions modules.order Module.symvers *.c.bak* *.o testmymodule \.*.cmd -rf


cp:
	cp g90b_iomux.ko ~/g90/rootfs-g90/genvict/lib

send:
	zftp g90b_iomux.ko /genvict/lib 192.168.5.245
#	tiny_send IDTmodule.ko 192.168.0.158:/lib

send0:
	zftp g90b_iomux.ko /genvict/lib 192.168.0.245

