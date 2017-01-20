/*
 *  linux/drivers/char/watchdog/omap_wdt.h
 *
 *  BRIEF MODULE DESCRIPTION
 *      OMAP Watchdog timer register definitions
 *
 *  Copyright (C) 2004 Texas Instruments.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef _OMAP_WATCHDOG_H
#define _OMAP_WATCHDOG_H

#define OMAP_WATCHDOG_REV		(0x00)
#define OMAP_WATCHDOG_SYS_CONFIG	(0x10)
#define OMAP_WATCHDOG_STATUS		(0x14)
#define OMAP_WATCHDOG_CNTRL		(0x24)
#define OMAP_WATCHDOG_CRR		(0x28)
#define OMAP_WATCHDOG_LDR		(0x2c)

/*
 *写一个新的值进去后使得wdt计数器重载
 *
*/
#define OMAP_WATCHDOG_TGR		(0x30)
/*
 *	OMAP_WATCHDOG_WPS:表示wdt中6个寄存器是否可写
 *	OMAP_WATCHDOG_CNTRL、OMAP_WATCHDOG_CRR、OMAP_WATCHDOG_LDR
 *	OMAP_WATCHDOG_TGR、  OMAP_WATCHDOG_SPR、WDT_WDLY
*/
#define OMAP_WATCHDOG_WPS		(0x34)

/*
 *	WDT使能/禁止寄存器
 *
*/
#define OMAP_WATCHDOG_SPR		(0x48)

/* Using the prescaler, the OMAP watchdog could go for many
 * months before firing.  These limits work without scaling,
 * with the 60 second default assumed by most tools and docs.
 */
#define TIMER_MARGIN_MAX	(24 * 60 * 60)	/* 1 day */
#define TIMER_MARGIN_DEFAULT	60	/* 60 secs */
#define TIMER_MARGIN_MIN	1




#define PTV			0	/* prescale */
#define GET_WLDR_VAL(secs)	(0xffffffff - ((secs) * (32768/(1<<PTV))) + 1)

/*	GET_WLDR_VAL的计算 参考TRM P4453
	OVF_Rate = (FFFF FFFFh – WDT_WLDR + 1) × (wd-functional clock period) × PS
	
	OVF_Rate = 5s(设置的值)
	(wd-functional clock period) = 1/32768
	PS = 1<<PTV
	
	WDT_WLDR = FFFF FFFFh - (OVF_Rate /((wd-functional clock period) * PS)) +1
			 = FFFF FFFFh - ( 5/(1/32768) *(1<<PTV)) + 1
			 = FFFF FFFFh - ( 5*(32768) /(1<<PTV)) + 1
*/





#endif				/* _OMAP_WATCHDOG_H */
