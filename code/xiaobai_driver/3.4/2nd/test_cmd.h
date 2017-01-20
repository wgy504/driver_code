#ifndef _TEST_CMD_H
#define _TEST_CMD_H

#define TEST_MAGIC		'x'		//定义幻数
#define TEST_MAX_NR		1		//定义命令的最大序数，只有一个命令当然是1


#define TEST_CLEAR		_IO(TEST_MAGIC, 0)

#endif /*_TEST_CMD_H*/
