#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
int main(void)
{
	char buf[20];
	int fd;	
	fd = open("/dev/my_dev", O_RDWR);
	if(fd < 0)
	{
		perror("open");
		return -1;
	}

	write(fd, "xiao bai", 10);

	return 0;
}
