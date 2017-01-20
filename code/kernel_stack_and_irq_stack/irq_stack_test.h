/*
 * irq_stack.h -- definitions for the kernel stack test module
 *
 * Copyright (C) 2011 Tekkamanninja
 *
 * The source code in this file can be freely used, adapted,
 * and redistributed in source or binary form, so long as an
 * acknowledgment appears in derived source files.  The citation
 * should list that the code comes from the book "
 * " by Tekkamanninja, published
 * by  .   No warranty is attached;
 * we cannot take responsibility for errors or fitness for use.
 *
 * $Id: irq_stack.h,v 1.00 28/10/2011 15:14:24  $
 */

#ifndef _IRQ_STACK_H_
#define _IRQ_STACK_H_

#ifndef IRQ_STACK_MAJOR
#define IRQ_STACK_MAJOR 0   /* dynamic major by default */
#endif


#define		TEST_IRQ_NUM	OMAP_GPIO_IRQ(56) /* GPIO? INT */

#endif /* _IRQ_STACK_H_ */
