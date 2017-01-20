#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/select.h>
#include <unistd.h>
#include <signal.h>

unsigned int flag;

void sig_handler(int sig)
{
	printf("<app>%s\n", __FUNCTION__);
	flag = 1;
}

int main(void)
{
	char buf[20];
	int fd;	
	int f_flags;
	flag = 0;

	fd = open("/dev/my_dev", O_RDWR);
	if(fd < 0)
	{
		perror("open");
		return -1;
	}
	
	signal(SIGIO, sig_handler);//注册信号处理函数
	fcntl(fd, F_SETOWN, getpid());//设置将要在文件描述词fd上接收SIGIO 或 SIGURG事件信号的进程或进程组标识
	f_flags = fcntl(fd, F_GETFL);//F_GETFL，先获取原来的状态标志
	fcntl(fd, F_SETFL, FASYNC | f_flags);//F_SETFL设置文件状态标志，在原来的状态基础上添加FASYNC状态标志

	while(1)
	{
		printf("waiting \n");
		sleep(4);
		if(flag)
			break;
	}

	read(fd, buf, 10);
	printf("finish: read[%s]\n", buf);

	close(fd);
	return 0;
}
