//也可通过 #cp main.c /dev/globalmem 写进去
//通过 #cat /dev/globalmem 读出来


/*write.c*/
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define MAXSIZE
int main(void)
{
	int i,fd,size,len;
	char c;
	if((fd = open("/dev/globalmem",   O_TRUNC | O_RDWR,0666 ))<0)
	{
		perror("open:");
		exit(1);
	}
	else
		printf("open file:/dev/globalmem %d\n",fd);
	while(read( fd, &c, 1)>0)
	{ 
		sleep(3);
		printf("%c",c);
	}
	
	if( close(fd) < 0 )
	{
		perror("close:");
		exit(1);
	}
	else
		printf("Close /dev/globalmem\n");
	
	exit(0);
}

