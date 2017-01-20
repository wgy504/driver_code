#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "test_cmd.h"

int main(void)
{
	char buf[20];
	int fd;
	int ret;

	fd = open("/dev/test", O_RDWR);
	if(fd < 0)
	{
		perror("open");
		return -1;
	}

	write(fd, "xiao bai", 10);
	
	ioctl(fd, TEST_CLEAR);

	ret = read(fd, buf, 10);
	if(ret < 0)
	{
		perror("read");
	}

	close(fd);
	return 0;
}
