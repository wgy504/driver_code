#project : 
#version : 
#author :
#date : 


APP_NAME = wait_queue
SRC_FILE = wait_queue_user.c

CC = arm-linux-gnueabihf-gcc
AR = arm-linux-gnueabihf-ar
STRIP = arm-linux-gnueabihf-strip



CFLAGS = -Wall $(INCFLAGS) -std=gnu99 $(DEFINE)
LDFLAGS = -lpthread 




 



all : 

	$(CC) -o $(APP_NAME) $(CFLAGS) $(LDFLAGS) $(SRC_FILE)

clean :
	rm  -rf  *.O $(APP_NAME)


send245:
	
	zftp $(APP_NAME) /home/root 192.168.5.245


