#
# name : linux driver makefile
#	user : wuw
#	date : 2016年1月5日 
#	
#
#

KBUILD_EXTRA_SYMBOLS += /mnt/hgfs/drv_code/platform/bus_device_driver/bus/Module.symvers
export KBUILD_EXTRA_SYMBOLS

ifeq ($(MODULE) , )

	MODULE := watchdog
	
endif

#DEBUG := YES 	
#ifeq ($(DEBUG), YES)

#	DEBFLAGS = -O -g -DDEBUG 		# "-O" is needed to expand inlines
#	ccflags-y += -DDEBUG
	
#else

#	DEBFLAGS = -O2
#	ccflags-y += -DAAAAAA
	
#endif

#CFLAGS += $(DEBFLAGS)

ifneq ($(KERNELRELEASE),)

	obj-m := $(MODULE).o
	
#$(MODULE)-objs := OTHER_FILE.o

else

#generate the path
CURRENT_PATH := $(shell pwd)

#the absolute path	
#LINUX_KERNEL_PATH := $(CURRENT_PATH)/../../system/linux-3.2-g90b_mini
LINUX_KERNEL_PATH := /mnt/hgfs/system/linux-3.2-g90b_mini

#complie object

default:
	make -C $(LINUX_KERNEL_PATH) M=$(CURRENT_PATH) modules
#	echo $(ccflags-y)
#	echo $(DEBUG)
	

#clean:
#	make -C $(LINUX_KERNEL_PATH) M=$(CURRENT_PATH) clean
endif

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c *.tmp_versions *.order *.symvers

send245:
	zftp $(MODULE).ko /home/root 192.168.5.245




depend .depend dep:
	$(CC) $(CFLAGS) -M *.c > .depend


ifeq (.depend,$(wildcard .depend))
include .depend
endif

#**************************************************************************************************#
#		在模块的源代码目录下执行make，此时，宏“KERNELRELEASE”没有定义，因此进入else。
#		由于make 后面没有目标，所以make会在Makefile中的第一个不是以.开头的目标作为默认的目标执行。
#		于是default成为make的目标。
#		make会执行 $(MAKE) -C $(KERNELDIR) M=$(PWD) modules ,假设当前内核版本是2.6.13-study,
#		所以$(shell uname -r)的结果是 2.6.13-study ，这里实际运行的是
#		make -C /lib/modules/2.6.13-study/build M=/home/study/prog/mod/hello/ modules
#		-C 表示到存放内核的目录执行其makefile，在执行过程中会定义KERNELRELEASE，kbuild也被启动去 解析kbuild语法的语句，
#		然后M=$(CURDIR)表示返回到当前目录，再次执行makefile，modules表示编译成模块的意思。
#		而此时KERNELRELEASE已定义，则会执行obj-m += hello.o，表示会将hello_world.o目标编译成.ko模块。
#		若有多个源文件，则采用如下方法：
#		obj-m := hello.o
#		hello-objs := file1.o file2.o file3.o
#		关于make modules的更详细的过程可以在内核源码目录下的scripts/Makefile.modpost文件的注释 中找到。
#
#









