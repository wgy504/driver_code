#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
int main(void)
{
	char buf[20];
	int fd;	
	fd = open("/dev/test", O_RDWR);
	if(fd < 0)
	{
		perror("open");
		return -1;
	}
	printf("<app_read> pid[%d]\n", getpid());
	read(fd, buf, 10);

	return 0;
}
