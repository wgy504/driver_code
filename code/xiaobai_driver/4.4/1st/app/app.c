#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>

int main(void)
{
	for(;;)
	{
		printf("<app> runing\n");	
		sleep(2);
	}

	return 0;
}
