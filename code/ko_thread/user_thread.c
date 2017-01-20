#include <stdio.h>
#include <signal.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <fcntl.h>
 #include<pthread.h>
#include <unistd.h>
#include <stdarg.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <string.h>
#include <sys/ioctl.h> 
#include <stdlib.h>
#include <unistd.h>




#define TEST_KO
//#define TEST_USER


#define CMD_1			_IOW('I', 0, int)
#define CMD_2			_IOW('I', 1, int)
#define CMD_3			_IOW('I', 2, int)
#define CMD_4			_IOW('I', 3, int)
typedef void* (*thread_func_t)(void* arg);

int fd;

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




int monitor_thread_create(pthread_t *pthread, thread_func_t start, void* arg)
{
    pthread_attr_t attr;

    pthread_attr_init(&attr); //init pthread
    pthread_attr_setdetachstate(&attr, (int)PTHREAD_CREATE_DETACHED);

    return pthread_create(pthread, &attr, start, arg);
}


void sigroutine(int signal)
{
		ioctl(fd, CMD_4, NULL);
		exit(0);
}

static void *func(void *arg)
{
	signal(SIGINT, sigroutine);
  signal(SIGPIPE, sigroutine);
  //signal(EBADF, sigroutine);
  signal(SIGKILL, sigroutine);
	set_scheduling(SCHED_RR, 62);
#ifdef TEST_KO	
	ioctl(fd, CMD_3, NULL);	
	while(1)
	{
		sleep(1);
		ioctl(fd, CMD_2, NULL);
		printf("thread func-----------------\n");	
	}
#endif	
	
#ifdef TEST_USER	
	
	while(1)
	{
		sleep(1);
		printf("thread func-----------------\n");	
		
	}
	
#endif	
	
	
}


void main()
{
		
    signal(SIGINT, sigroutine);
    signal(SIGPIPE, sigroutine);
   // signal(EBADF, sigroutine);
    signal(SIGKILL, sigroutine);
		set_scheduling(SCHED_RR, 62);
		pthread_t tid;
		if(monitor_thread_create(&tid, func, NULL) == 0)
		{
				printf("func create success!!!!\n");	
		}
		else
		{
				printf("func create error!!!!\n");	
			
		}
		
#ifdef TEST_KO			
		fd = open("/dev/my_dev", O_RDWR);
		if(fd > 0)
		{
			printf("fd open success!\n");
			
		}
		else
		{
			printf("fd open error!\n");
			return;
		}
		
		ioctl(fd, CMD_3, NULL);
		while(1)
		{
			printf("main func\n");	
			sleep(1);
			ioctl(fd, CMD_1, NULL);
			
		}
#endif


	


#ifdef TEST_USER
		while(1)
		{
			sleep(1);
			printf("main func\n");	
			
		}


#endif		
		
		
		
		
	
}

/*
	用户空间线程与内核空间的线程是一一对应的关系，并且线程类型也应该是一样的.
	现在的系统线程虽然是实时线程，但是其也是优先级+时间片的方式：
		优先级高的比优先级低的先运行，且低优先级的线程不能抢占高优先级的线程。除非高优先级的线程主动放弃CPU
		当优先级一样时，机会相等，调度的参考为时间片。
		













*/





















