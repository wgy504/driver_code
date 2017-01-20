#ifndef _TEST_CMD_H
#define _TEST_CMD_H

struct ioctl_data{
	unsigned int size;
	char buf[100];
};

#define DEV_SIZE	100

#define TEST_MAGIC		'x'		//定义幻数
#define TEST_MAX_NR		3		//定义命令的最大序数

#define TEST_CLEAR		_IO(TEST_MAGIC, 1)
#define TEST_OFFSET		_IO(TEST_MAGIC, 2)
#define TEST_KBUF		_IOW(TEST_MAGIC, 3, struct ioctl_data)

#endif /*_TEST_CMD_H*/
