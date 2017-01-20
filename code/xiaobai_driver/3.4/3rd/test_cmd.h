#ifndef _TEST_CMD_H
#define _TEST_CMD_H

#define TEST_MAGIC		'x'		//定义幻数
#define TEST_MAX_NR		2		//定义命令的最大序数

#define TEST_CLEAR		_IO(TEST_MAGIC, 1)
#define TEST_OFFSET		_IO(TEST_MAGIC, 2)

#endif /*_TEST_CMD_H*/
