
DIR = $(shell "pwd")

TARGET = user_thread

all:
	arm-linux-gnueabihf-gcc $(TARGET).c -o $(TARGET) -lpthread




send:
	zftp $(TARGET) /home/root 192.168.${ip}
	
gv_send:
	zftp_gv $(TARGET) /home/root root gv020 192.168.${ip}
