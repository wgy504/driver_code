#include <stdio.h>
#include <unistd.h> 
#include <fcntl.h>  
#define MISC_DEV  "/dev/my_misc"




int main(int argc, char** argv)
{
	int ret = 0;
	int fd = open(MISC_DEV, O_RDWR|O_NDELAY);
	int val = 0;
	ret = read(fd, &val, 4);
	printf("read val = %d\n", val);
	int cnt = 10;
	while(cnt-- > 0)
	{
		ret = cnt % 2;
		write(fd, &ret, 4);
		
		sleep(2);
	}
	
	
	
}

