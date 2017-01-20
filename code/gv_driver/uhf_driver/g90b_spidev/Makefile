obj-m += gv_spidev.o
gv_spidev-objs := spidev.o
#模块代码的当前目录，SUBDIRS需要
PWD := $(shell pwd)
#内核源码的位置变量
KPATH := $(PWD)/../../linux-3.2-g90b_mini
CROSS_ARCH := ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
CC = $(CROSS_COMPILE)gcc

#这是真正的编译命令
default:
	$(MAKE) $(CROSS_ARCH) -C $(KPATH) SUBDIRS=$(PWD) modules

clean:
	rm *.ko *.mod.c *.bak .tmp_versions modules.order Module.symvers *.c.bak* *.o testmymodule \.*.cmd -rf


cp:
	cp gv_spidev.ko ~/g90/rootfs-g90/genvict/lib

send:
	zftp gv_spidev.ko /genvict/lib 192.168.5.245
#	tiny_send IDTmodule.ko 192.168.0.158:/lib
send0:
	zftp gv_spidev.ko /genvict/lib 192.168.0.245
