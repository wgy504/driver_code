#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <pthread.h>
#include <libgen.h> //need for basename()
#include <signal.h>
#include <arpa/inet.h>
#include <fcntl.h>




#include<sys/ioctl.h>

#include <sched.h>
#include <sys/syscall.h> 



#define SET_SLEEP 1
#define SET_WAKE_UP 2


typedef void* (*thread_func_t)(void* arg);

int my_dev_fd;

int monitor_thread_create(pthread_t *pthread, thread_func_t start, void* arg)
{
    pthread_attr_t attr;

    pthread_attr_init(&attr); //init pthread
    pthread_attr_setdetachstate(&attr, (int)PTHREAD_CREATE_DETACHED);

    return pthread_create(pthread, &attr, start, arg);
}
int set_scheduling(int schedpol, int prio)
{
    struct sched_param schedparm = { 0 };
    int prio_max;
    int prio_min;

    prio_min = sched_get_priority_min(schedpol);
    prio_max = sched_get_priority_max(schedpol);
    //printf("min priority: %i, max priority: %i\n",prio_min,prio_max);

    if ((prio > prio_max) || (prio < prio_min))
    {
        printf("priority should be between %d, %d\n", sched_get_priority_min(schedpol), sched_get_priority_max(schedpol));
        return -1;
    }

    schedparm.sched_priority = prio;
    //printf("schedpol = %d, priority = %d\n", schedpol, schedparm.sched_priority);
    if (sched_setscheduler(0, schedpol, &schedparm))
    {
        perror("sched_setscheduler()");
        return -1;
    }
    return 0;
}


void *wake_up_thread(void *arg)
{
	printf("%s--start---\n", __FUNCTION__);
	for(int i = 0; i < 5; i++){
		printf("sleep %d\n", i);
		sleep(1);
	}
	ioctl(my_dev_fd, SET_WAKE_UP, NULL);
	printf("%s--end---\n", __FUNCTION__);
	return NULL;
}


int main(int argc, char **argv)
{
	int ret = 0;
	pthread_t tid;
	//set_scheduling(SCHED_FIFO, 62);

	
	my_dev_fd = open("/dev/my_dev", O_RDWR);
	if(my_dev_fd <= 0)	{
		printf("fd error\n");
		return -1;
	}
	monitor_thread_create(&tid, wake_up_thread, NULL);
	ret = ioctl(my_dev_fd, SET_SLEEP, NULL);
	printf("ret = %d\n", ret);	
	
	
	

	return 0;
}

