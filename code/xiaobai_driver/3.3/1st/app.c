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

	read(fd, buf, 10);
	printf("<app>buf is [%s]\n", buf);

	write(fd, "xiao bai", 10);

	read(fd, buf, 10);
	printf("<app>buf is [%s]\n", buf);

	close(fd);
	return 0;
}
