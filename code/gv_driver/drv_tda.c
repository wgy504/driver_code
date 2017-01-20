/***********************************************************
 * 版权信息: 金溢科技版权所有，保留一切权利
 * 文件名称: drv_tda.c
 * 文件作者: linsl
 * 完成日期: 2015-09-22
 * 当前版本: 1.0.0
 * 主要功能: SPAM卡DS8007读写芯片驱动
 * 版本历史: 
 ***********************************************************/
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/irq.h>
#include <linux/swab.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <plat/gpmc.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <asm/delay.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/sched.h>


#define G_VER           "v1.00"
#define DEV_NAME        "gv_tda"
#define DEV_FILE_NAME   "TDAmodule"

#define DEV_MAJOR       242

//定义8007时钟频率
#define CRYSTAL_FREQUENCY_8007 14745600L  

#define CARD_SLOTS  2

/** 8007 Register definitions */
#define CSR     0x00	 /** Offset address of CSR */
#define CCR     0x01	 /** Offset address of CCR */
#define PDR     0x02	 /** Offset address of PDR */
#define UCR2    0x03	 /** Offset address of UCR2*/
#define GTR     0x05	 /** Offset address of GTR */
#define UCR1    0x06	 /** Offset address of UCR1*/
#define PCR     0x07	 /** Offset address of PCR */
#define TOC     0x08	 /** Offset address of TOC */
#define TOR1    0x09	 /** Offset address of TOR1*/
#define TOR2    0x0A	 /** Offset address of TOR2*/
#define TOR3    0x0B	 /** Offset address of TOR3*/
#define MSR     0x0C	 /** Offset address of MSR */
#define FCR     0x0C	 /** Offset address of FCR */
#define UTR     0x0D	 /** Offset address of UTR */
#define URR     0x0D	 /** Offset address of URR */
#define USR     0x0E	 /** Offset address of USR */
#define HSR     0x0F	 /** Offset address of HSR */

/** Bit definitions of CSR register */
#define CSR_ID_MASK         0xF0  /** Device Identification Register Bits */
#define CSR_nRIU_MASK       0x08  /** Reset ISO UART bit */
#define CSR_SC3_MASK        0x04  /** Select Card Configuration Bits */
#define CSR_SC2_MASK        0x02	  
#define CSR_SC1_MASK        0x01      

/** Bit definitions of HSR register */
#define HSR_PRTL2_MASK      0x40  /** Protection Card Interface B Status Bit */
#define HSR_PRTL1_MASK      0x20  /** Protection Card Interface A Status Bit */
#define HSR_SUPL_MASK       0x10  /** Supervisor Latch bit */
#define HSR_PRL2_MASK       0x08  /** Presence Latch B bit */
#define HSR_PRL1_MASK       0x04  /** Presence Latch A bit */
#define HSR_INTAUXL_MASK    0x02  /** INTAUX Latch bit */
#define HSR_PTL_MASK        0x01  /** Protection Thermal Latch bit */

/** Bit definitions of MSR register */
#define MSR_CLKSW_MASK      0x80  /** Clock Switch */
#define MSR_FE_MASK         0x40  /** FIFO Empty Status Bit */
#define MSR_BGT_MASK        0x20  /** Block Guard Time Status Bit */
#define MSR_CRED_MASK       0x10  /** Control Ready */
#define MSR_PR2_MASK        0x08  /** Presence Card B */
#define MSR_PR1_MASK        0x04  /** Presence Card A */
#define MSR_INTAUX_MASK     0x02  /** INTAUX bit */
#define MSR_TBE_RBF_MASK    0x01  /** Transmit Buffer Empty/Receive Buffer Full bit */

/** Bit definitions of FCR register */
#define FCR_PEC_MASK        0x70  /** Parity Error Count bits */
#define FCR_FL_MASK         0x07  /** FIFO Length bits */

/** Bit definitions of USR register */
#define USR_TOL3_MASK       0x80  /** Time-Out Counter 3 Status bit */
#define USR_TOL2_MASK       0x40  /** Time-Out Counter 2 Status bit */
#define USR_TOL1_MASK       0x20  /** Time-Out Counter 1 Status bit */
#define USR_EA_MASK         0x10  /** Early Answer Detected bit */
#define USR_PE_MASK         0x08  /** Parity Error bit */
#define USR_OVR_MASK        0x04  /** Overrun FIFO bit */
#define USR_FER_MASK        0x02  /** Framing Error bit */
#define USR_TBE_RBF_MASK    0x01  /** Transmit Buffer Empty/Receive Buffer Full bit */

/** Bit definitions of UCR1 register */
#define UCR1_FTE0_MASK      0x80  /** */
#define UCR1_FIP_MASK       0x40  /** Force Inverse Parity bit */
#define UCR1_PROT_MASK      0x10  /** Protocol Select bit */
#define UCR1_T_R_MASK       0x08  /** Transmit/Receive bit */
#define UCR1_LCT_MASK       0x04  /** Last Character to Transmit bit */
#define UCR1_SS_MASK        0x02  /** Software Convention Setting bit */
#define UCR1_CONV_MASK      0x01  /** Convention bit */

/** Bit definitions of UCR2 register */
#define UCR2_DISTBE_RBF_MASK 0x40 /** Disable TBE/RBF Interrupt bit */
#define UCR2_DISAUX_MASK     0x20 /** Disable Auxiliary Interrupt bit */
#define UCR2_PDWN_MASK       0x10 /** Power Down Mode Enable bit */
#define UCR2_SAN_MASK        0x08 /** Synchronous/asynchronous Card Select bit */
#define UCR2_nAUTOCONV_MASK  0x04 /** Auto Convention Disable bit */
#define UCR2_CKU_MASK        0x02 /** Clock UART Doubler Enable bit */
#define UCR2_PSC_MASK        0x01 /** Prescaler Select bit */

/** Bit definitions of CCR register */
#define CCR_SHL_MASK        0x20  /** Stop High Low Select bit */
#define CCR_CST_MASK        0x10  /** Clock Stop Enable bit */
#define CCR_SC_MASK         0x08  /** Synchronous Clock bit */
#define CCR_AC_MASK         0x07  /** Alternating Clock Select bit */

/** Bit definitions of PCR register */
#define PCR_C8_MASK         0x20  /** Contact 8 bit */
#define PCR_C4_MASK         0x10  /** Contact 4 bit */
#define PCR_1V8_MASK        0x08  /** 1.8V Card Select bit */
#define PCR_RSTIN_MASK      0x04  /** Reset Bit */
#define PCR_3V_5V_MASK      0x02  /** 3V/5V Card Select bit */
#define PCR_START_MASK      0x01  /** Start bit, It initiates activation sequence */


#define POWERUP_5V      	0
#define POWERUP_3V      	1
#define POWERUP_1V8     	2

#define PROTOCOL_T0   		0
#define PROTOCOL_T1   		1    

#define LOW    				0
#define HIGH   				1 

//用于ioctl接口函数
#define IOCTL_PSAM_APDU_DISPOSE 		0			//发送APDU并接收响应处理
#define IOCTL_ONE_PSAM_RESET_DISPOSE 	1			//复位单个PSAM卡处理
#define IOCTL_ALL_PSAM_RESET_DISPOSE 	2			//复位所有PSAM卡处理
#define IOCTL_TDA_POWERUP_DISPOSE		3			//TDA上电操作
#define IOCTL_TDA_POWERDOWN_DISPOSE		4			//TDA下电操作
#define IOCTL_ReadPsamInfo_DISPOSE		5			//读取PSAM卡相关信息操作 

#define GET_PSAM_TERMID					0			//取PSAM卡终端机编号
#define Enter_PSAM_DF01					1			//进PSAM卡DF01目录
#define Enter_PSAM_3F01					2			//进PSAM卡3F01目录
#define Enter_PSAM_1001					3			//进PSAM卡1001目录
#define CAL_PSAM_DESINIT				4			//DES初始化
#define CAL_PSAM_DES					5			//DES计算
#define CAL_PSAM_DESINIT_GB				6			//DES计算
#define GET_PSAM_RAND					7			//取随机数
#define Enter_PSAM_3F00					8			//进PSAM卡3F00目录
 

//======= 错误代码定义 ========================================
#define NO_ERROR					0 			//成功

#define RESET_PSAM_NOT_3B3F_ERROR		-1 			//复位PSAM卡时,复位信息的第一个字节不是3B或3F
#define RESET_PSAM_TIMEOUT				-2			//复位PSAM卡超时
#define SELECT_CARDVOLTAGE_ERROR		-3			//卡电压选择错误
#define TDA_POWERUP_ERROR				-4			//TDA激活PSAM卡失败
#define SELECT_PSAM_NUMBER_ERROR		-5			//选择PSAM卡错误
#define TIMECOS_CMD_LEN_ERROR			-6			//Timecos指令长度超长
#define DISPOSE_PSAM_T1_TIMEOUT_ERROR	-7			//处理T=1的PSAM卡超时
#define PSAM_T1_RETURN_LEN_ERROR		-8			//T=1的PSAM卡返回数据长度超长
#define PSAM_RESPONSE_BCC_ERROR			-9			//T=1的PSAM卡返回数据的bcc校验错误
#define DISPOSE_PSAM_PPS_TIMEOUT_ERROR	-10			//PPS失败
#define DISPOSE_PSAM_T0_TIMEOUT_ERROR	-11			//处理T=0的PSAM卡超时
#define PSAM_T0_RESPONSE_NOT_C0			-12			//T=0卡处理返回的第3步时没有0xc0字头
#define PSAM_T0_RESPONSE_NOT_CMD_HEADER	-13			//T=0卡，当命令小于6字节是，返回的数据没有命令头
#define KERNEL_TIMEOUT					-14			//内核定时器计时超时




u8 PSAMInfoBuf[500];				//保存PSAM卡复位信息	

u8 g_ATR_buf[500];				//ATR数据缓冲区,将复位缓冲区生命为全局变量，在PPS时会用到复位数据2012-2-20
u8 g_ATRlen;						//ATR数据长度					
 
u8 g_cardtype;				//存放公共卡类型
u8 g_cardtype_buf[500];  		//卡协议类型  (T=0 or T=1)     注 : g_cardtype_buf[0]  	未用
u8 g_cmdtype;				//存放公共命令类型
u8 g_cmdtype_buf[500]; 		//卡命令类型  (对于T=1卡有用)  注 : g_cmdtype_buf[0]	未用

u8 g_resetspeed_buf[500];		//复位速度标志缓冲区 注：g_resetspeed_buf[0]未用 0--低速 1--高速



//用于IOCTL函数，作为接收与发送数据 
typedef struct 
{
	u8   psamnum;				//PSAM号
	u8	datalen;	     		//数据长度 (存放命令长度或响应数据长度)
	u8	databuf[130];   		//数据缓冲区
	u8   resetbuf[6];			//存放复位信息 (1--存在 0--不存在) 
}IoctlInfo;

static IoctlInfo g_SendInfo; 	//发送命令结构体
static IoctlInfo g_RecvInfo; 	//接收响应结构体


struct gv_tda_io
{
	int io_wr_out;
	int io_rd_out;

	int io_psam_select1;
	int io_psam_select2;
	int io_psam_select3;

	int io_psam_led1;
	int io_psam_led2;
	int io_psam_led3;
	int io_psam_led4;
	int io_psam_led5;
	int io_psam_led6;
};

//设备类
static struct class *tda_class;

//设备私有数据
struct gv_dev 
{
    char *name;
    int id;

    void __iomem *ioaddr;

    struct platform_device *pdev;
    struct cdev cdev;
    dev_t devno; //device node num
	int irq;

    wait_queue_head_t recv_wait;
    u32 recv_ready;

    spinlock_t recv_lock;

    struct mutex open_lock;

	struct gv_tda_io *dio;

	u8 recv_buf[1024];
	int rlen;
	
	u8 atr_buf[1024]; //reset buf
};

static  void __iomem *tda_addr = NULL;
static struct gv_tda_io *tda_io = NULL;

/*****************************************************************
 函数名称：cal_bcc
 函数描述：计算数据的BCC校验
 输入参数：buf，数据
		   len，数据大小
 输出参数：无
 返回说明：BCC值
 其它说明：
 *****************************************************************/
static u8 cal_bcc(u8 *buf, int len)
{
	u8 bcc = 0;	
	int i = 0;

	for (i=0; i<len; i++)
	{
		bcc ^= buf[i];
	}
	
	return bcc;
}

/*****************************************************************
 函数名称：gv_io_out
 函数描述：GPIO输出电平
 输入参数：ionum：管脚标号
		   val电平0,1
 输出参数：无
 返回说明：无
 其它说明：
 *****************************************************************/
static inline void gv_io_out(int ionum, int val)
{   
    gpio_set_value(ionum, val);
}

/*****************************************************************
 函数名称：gv_io_in
 函数描述：读GPIO的电平
 输入参数：无
 输出参数：无
 返回说明：电平或错误状态
 其它说明：
 *****************************************************************/
static inline int gv_io_in(int ionum)
{   
    return gpio_get_value(ionum);
}

/*****************************************************************
 函数名称：psam_led_control
 函数描述：控制PSAM卡的指示灯
 输入参数：psam_idx，卡槽编号
		   onoff, 开关
 输出参数：无
 返回说明：无
 其它说明：
 *****************************************************************/
static void psam_led_control(u8 psam_idx , u8 onoff )
{
	switch(psam_idx)
	{
		case 1:						//PSAM1指示灯模式
			if(onoff)
			{
				gv_io_out(tda_io->io_psam_led1, 1);
			}	
			else
			{
				gv_io_out(tda_io->io_psam_led1, 0);
			}	
			break;	
		
		case 2:						//PSAM2指示灯模式
			if(onoff)
			{
				gv_io_out(tda_io->io_psam_led2, 1);
			}	
			else
			{
				gv_io_out(tda_io->io_psam_led2, 0);
			}	
			break;	
		
		case 3:						//PSAM3指示灯模式
			if(onoff)
			{
				gv_io_out(tda_io->io_psam_led3, 1);
			}	
			else
			{
				gv_io_out(tda_io->io_psam_led3, 0);
			}	
			break;	
		
		case 4:						//PSAM4指示灯模式
			if(onoff)
			{
				gv_io_out(tda_io->io_psam_led4, 1);
			}	
			else
			{
				gv_io_out(tda_io->io_psam_led4, 0);
			}	
			break;				
		
		case 5:						//PSAM5指示灯模式
			if(onoff)
			{
				gv_io_out(tda_io->io_psam_led5, 1);
			}	
			else
			{
				gv_io_out(tda_io->io_psam_led5, 0);
			}	
			break;	
		
		case 6:						//PSAM6指示灯模式
			if(onoff)
			{
				gv_io_out(tda_io->io_psam_led6, 1);
			}	
			else
			{
				gv_io_out(tda_io->io_psam_led6, 0);
			}	
			break;	
		
		default:
			break;			
	}
}

/*****************************************************************
 函数名称：tda_write
 函数描述：向芯片寄存器写入数据
 输入参数：reg，寄存器编号
		   val，数据
 输出参数：无
 返回说明：无
 其它说明：
 *****************************************************************/
static void tda_write(u8 reg, u8 val)
{ 
   volatile unsigned char *offset  = ((volatile unsigned char *)tda_addr) + reg;
	
	gv_io_out(tda_io->io_rd_out, 0);
	offset[0] = val;	
	
	udelay(5);
}

/*****************************************************************
 函数名称：tda_read
 函数描述：读取芯片寄存器数据
 输入参数：reg，寄存器编号
 输出参数：无
 返回说明：寄存器数据
 其它说明：
 *****************************************************************/
static u8 tda_read(u8 reg) 
{
	volatile unsigned char *offset  = ((volatile unsigned char *)tda_addr) + reg;
	
	gv_io_out(tda_io->io_rd_out, 1);
	return offset[0];
}


volatile int g_count1;
volatile int g_count2;
/*****************************************************************
 函数名称：iso7816_sendbyte
 函数描述：向SPAM卡写入一着急数据
 输入参数：content，数据内容
		   lct，是否最后一个字节数据
 输出参数：无
 返回说明：0，成功；其他设备
 其它说明：
 *****************************************************************/
static int iso7816_sendbyte(u8 content, u8 lct)
{
		int timeout_us = 100 * 1000;
	  u8 val = tda_read(UCR1);
	  
	  if(lct)
	  		tda_write(UCR1, val | (UCR1_T_R_MASK  |UCR1_LCT_MASK));	//T/R=1 and LCT=1
	  else
	  		tda_write(UCR1, (val | UCR1_T_R_MASK) & ~UCR1_LCT_MASK);	//T/R=1 and LCT=0

	  tda_write(UTR,content); 

      //如果发送的是最后一个字符就不在判断发送缓冲区是否为空，直接退出，因为LCT=1时
      //发送完最后一个字符后USR_TBE_RBF_MASK自动被清0，再去检测可能导致一直检测不到而卡在下面的循环中
      //重庆PSAM卡出现过此问题 2012-2-20 
      if(lct)
      {
	       	 return 0;   
      }  

	  while (!(tda_read(USR) & USR_TBE_RBF_MASK))					//因为置了LCT,发送最后一个字节时不产生中断	
	  {																//所以在此轮循USR
	  	   //之前是10000,后来在杭州测试时发现容易超时(当时的表现是校验MAC2失败)
	  	   //修改后正常了 林树亮 2009-5-19 于杭州测试	  	    
	  	    udelay(1);
	  	    if (timeout_us-- < 0)
	  	    {
	  	   		return KERNEL_TIMEOUT;		  	    	
	  	    }	  	    
	  }

	  return 0;	  
}

/*****************************************************************
 函数名称：tda_timer_start
 函数描述：设置芯片内定时器，控制PSAM ETU
 输入参数：无
 输出参数：无
 返回说明：无
 其它说明：
 *****************************************************************/
static void tda_timer_start(u8 mode, u8 tor3, u8 tor2, u8 tor1)
{
	  u8 i,val;
	  
	  tda_write(TOC ,0x00);
	  for(i=0;i<200;i++);
	  tda_write(TOR3,tor3);
	  for(i=0;i<200;i++);
	  tda_write(TOR2,tor2);
	  for(i=0;i<200;i++);
	  tda_write(TOR1,tor1); 
	  for(i=0;i<200;i++);
	  tda_write(TOC ,mode);      		//启动timer 
	  
//	  ds8007_int=0;
//	  atomic_set(&ds8007_int,0);
	  
  
 	  g_count1=0; 
	  g_count2=0;
	  
	  val=tda_read(USR);					//清标志位
	  val=tda_read(URR);
}

/*****************************************************************
 函数名称：tda_timer_stop
 函数描述：停止定时器
 输入参数：无
 输出参数：无
 返回说明：无
 其它说明：
 *****************************************************************/
void tda_timer_stop(void)
{
	  tda_write(TOC ,0x00);
}

/*****************************************************************
 函数名称：iso7816_sendcmd
 函数描述：向SPAM卡发送命令APDU
 输入参数：buff，命令
		   len，命令长度
 输出参数：无
 返回说明：成功标识
 其它说明：
 *****************************************************************/
static int iso7816_sendcmd( u8 *buff, int len )      				                 
{                                                           
	int i;
	int ret;
	
	for(i=0;i<len;i++) 
	{
		if(i==(len-1))
			ret=iso7816_sendbyte(buff[i],1);		// LCT=1	
		else
			ret=iso7816_sendbyte(buff[i],0);	
			
		if(ret!=0)
		{
			return ret; 				
		}			
	}

	return 0;	
} 

/*****************************************************************
 函数名称：send_pps
 函数描述：向PSAM卡发送提速命令
 输入参数：protocoltype, T0, 或者T1
 输出参数：无
 返回说明：成功标识
 其它说明：
 *****************************************************************/
static int send_pps(u8 protocoltype)
{
	int i,ret,cnt;
    u8 val,USRval,MSRval;
    u8 CMDBuf[500];
	u8 PPS_RecvBuf[500];
	u8 PPS_Recvlen=0;
	
	if( protocoltype==PROTOCOL_T0 )		//T=0;
	{
		CMDBuf[0]=0XFF;
		CMDBuf[1]=0X10;
		CMDBuf[2]=0X13;					//提4倍速
		CMDBuf[3]=0XFC;
	}
	else 								//T=1
	{
		CMDBuf[0]=0XFF; 
		CMDBuf[1]=0X11;
		CMDBuf[2]=0X13;					//提4倍速
		CMDBuf[3]=0XFD;
	}
	
	tda_timer_start(0x61,0x0a,0xc3,0x00);		//设置超时时间
	
	local_irq_disable();					//关中断
	ret=iso7816_sendcmd( CMDBuf, 4 );				//发送命令
	if(ret!=0)
    {
        local_irq_enable();					//开中断
		return ret; 
	}
	
	cnt=0;
	while(1)       
	{			
		udelay(50);						//增加延时2009-12-17
		USRval = tda_read(USR);
		if (USRval & USR_TBE_RBF_MASK)		//判断串口是否有数据		
		{	
			for(i=0;i<4001;i++)  
			{
			     udelay(50);				//增加延时2009-12-17
			     USRval=tda_read(USR);
			     MSRval=tda_read(MSR);
			     if( (USRval&USR_TBE_RBF_MASK) || (!(MSRval&MSR_FE_MASK)) )
			     {
					 val = tda_read(URR);					
					 PPS_RecvBuf[PPS_Recvlen++] = val; 
				     if( PPS_Recvlen>2 ) 				//判断PPS是否返回数据(返回的数据与PPS命令相同)
				     {
				    	tda_write(PDR,3);				//设置分频  高速
						break;
					 }	 
				 } 
				 else if ( (USRval&USR_TOL3_MASK) || (i==4000) )
				 {
					  local_irq_enable();			//开中断
					  return DISPOSE_PSAM_PPS_TIMEOUT_ERROR;   	      
				 }     
			}    
			break;     
	    } 
	//  else  if (USRval & USR_TOL3_MASK)   
		else
		{	
			cnt++;   
			if(cnt==2000)  				//将超时修改成次数限制的方式,因为这个超时寄存器容易不起作用导致卡在驱动里 2009-12-17
			{
				//判断是否是重庆MF1卡的PSAM卡，如果是就强行提速2012-2-20
				if(g_ATR_buf[0]==0x3b && g_ATR_buf[1]==0x7e && g_ATR_buf[2]==0x13 && g_ATR_buf[3]==0x00 && g_ATR_buf[4]==0x02 && g_ATR_buf[5]==0x15)
				{
				    printk ("PSAM支持提速但PPS无响应,强行提速!\n");   
				    tda_write(PDR,3);				//设置分频  高速 
				    local_irq_enable();				//开中断 
				    return 0;  
				}    
				
				local_irq_enable();						//开中断
				return DISPOSE_PSAM_PPS_TIMEOUT_ERROR;   	      
			}
		} 	 
	}	
	
	local_irq_enable();				//开中断 
	
//	printk ("PPS_Recvlen = 0x%02x\n", PPS_Recvlen);
//	 for(i=0;i<PPS_Recvlen;i++) 
//	  		printk ("PPS_RecvBuf = 0x%02x\n", PPS_RecvBuf[i]);

	return 0;  

}

/*****************************************************************
 函数名称：tda_init
 函数描述：初始化芯片
 输入参数：无
 输出参数：无
 返回说明：无
 其它说明：
 *****************************************************************/
static void tda_init(void)
{
	  u8 val = 0;
	  
	  //---------  不选中任何卡  ---------
	  val = tda_read(CSR);
	  tda_write(CSR,val & ~(CSR_SC1_MASK | CSR_SC2_MASK | CSR_SC3_MASK));	//都不选中

	  udelay(5);  

	  //------  卡A寄存器初始化 ---------
	  val = tda_read(CSR);
	  tda_write(CSR,(val | CSR_SC1_MASK) & ~(CSR_SC2_MASK | CSR_SC3_MASK));	//选择卡A
	  udelay(5);  
	  val = tda_read(CSR);
	  tda_write(CSR,val & ~CSR_nRIU_MASK);	
	  udelay(5);  
	  tda_write(CSR,val | CSR_nRIU_MASK);									//RIU位拉低几微妙,复位寄存器
	  udelay(5);  
	  //------  卡B寄存器初始化 ---------
	  val = tda_read(CSR);
	  tda_write(CSR,(val | CSR_SC2_MASK) & ~(CSR_SC1_MASK | CSR_SC3_MASK));	//选择卡B
	  udelay(5);  
	  val = tda_read(CSR);
	  tda_write(CSR,val & ~CSR_nRIU_MASK);	
	  udelay(5);  
	  tda_write(CSR,val | CSR_nRIU_MASK);									//RIU位拉低几微妙,复位寄存器
	  udelay(5);  
	  //-----------  关闭A卡  -----------
	  val = tda_read(CSR);
	  tda_write(CSR,(val | CSR_SC1_MASK) & ~(CSR_SC2_MASK | CSR_SC3_MASK));	//选择卡A
	  udelay(5);  
	  val = tda_read(PCR);
	  tda_write(PCR,val & ~PCR_START_MASK);									//关闭卡A
	  udelay(5);  
	  //-----------  关闭B卡  -----------
	  val = tda_read(CSR);
	  tda_write(CSR,(val | CSR_SC2_MASK) & ~(CSR_SC1_MASK | CSR_SC3_MASK));	//选择卡B
	  udelay(5);  
	  val = tda_read(PCR); 
	  tda_write(PCR,val & ~PCR_START_MASK);									//关闭卡B
	udelay(5);  
	  //---------  不选中任何卡  ---------
	  val = tda_read(CSR);
	  tda_write(CSR,0);														//都不选中
	  udelay(5);  
	  
	  printk("tda init ok\n");
}

/*****************************************************************
 函数名称：tda_select_channel
 函数描述：
 输入参数：无
 输出参数：无
 返回说明：成功标识
 其它说明：
 *****************************************************************/
static int tda_select_channel(u8 chan)
{
	  u8 val = tda_read(CSR);
	  
	  switch (chan) 
	  {
	    case 1:
	      	tda_write(CSR, (val & ~(CSR_SC3_MASK | CSR_SC2_MASK))|CSR_SC1_MASK);		//选择卡A	      
	      	break;
	    case 2:
	      	tda_write(CSR, (val & ~(CSR_SC3_MASK|CSR_SC1_MASK))|CSR_SC2_MASK);		//选择卡B
	     		break;
	     		
	    default:
	    		tda_write(CSR,val & ~(CSR_SC1_MASK | CSR_SC2_MASK | CSR_SC3_MASK));		//都不选中
	      	return 1;
	  }	
	  
	  return 0;
}

/*****************************************************************
 函数名称：psam_select_card
 函数描述：现在SPAM卡槽
 输入参数：psam_num，卡槽号
 输出参数：无
 返回说明：无
 其它说明：
 *****************************************************************/
//only connect on the tda channel 1
static void psam_select_card(int psam_num)
{
	switch(psam_num)
	{
		case 1:					//选择卡1
			gv_io_out(tda_io->io_psam_select1, 0);
			gv_io_out(tda_io->io_psam_select2, 0);
			gv_io_out(tda_io->io_psam_select3, 0);						
			break;
			
		case 2:					//选择卡2
			gv_io_out(tda_io->io_psam_select1, 1);
			gv_io_out(tda_io->io_psam_select2, 0);
			gv_io_out(tda_io->io_psam_select3, 0);						
			break;
		case 3:					//选择卡3
			gv_io_out(tda_io->io_psam_select1, 0);
			gv_io_out(tda_io->io_psam_select2, 1);
			gv_io_out(tda_io->io_psam_select3, 0);						
			break;
			
		case 4:					//选择卡4
			gv_io_out(tda_io->io_psam_select1, 1);
			gv_io_out(tda_io->io_psam_select2, 1);
			gv_io_out(tda_io->io_psam_select3, 0);						
			break;
			
		case 5:					//选择卡5
			gv_io_out(tda_io->io_psam_select1, 0);
			gv_io_out(tda_io->io_psam_select2, 0);
			gv_io_out(tda_io->io_psam_select3, 1);						
			break;
			
		case 6:					//选择卡6
			gv_io_out(tda_io->io_psam_select1, 1);
			gv_io_out(tda_io->io_psam_select2, 0);
			gv_io_out(tda_io->io_psam_select3, 1);						
			break;
			
		default:				//	
			gv_io_out(tda_io->io_psam_select1, 0);
			gv_io_out(tda_io->io_psam_select2, 0);
			gv_io_out(tda_io->io_psam_select3, 0);								
			break;
	}
}

/*****************************************************************
 函数名称：tda_powerdown
 函数描述：芯片掉电，进入低功耗模式
 输入参数：无
 输出参数：无
 返回说明：电平或错误状态
 其它说明：
 *****************************************************************/
static void tda_powerdown(void)
{
  	u8 val = tda_read(PCR);		
  	tda_write(PCR,val & ~PCR_START_MASK);		//关闭卡
}

/*****************************************************************
 函数名称：tda_powerup
 函数描述：芯片退出低功耗模式
 输入参数：voltage
 输出参数：无
 返回说明：成功标识
 其它说明：
 *****************************************************************/
static int tda_powerup(u8 voltage)
{
	u8 val;	
		
	tda_write(CCR,0x01);			//外部晶振2分频  

	//----- 设置卡供电电压 ------
	val = tda_read(PCR);	  
	tda_write(PCR,val & ~(PCR_3V_5V_MASK|PCR_1V8_MASK));		//清1.8V和3V-5V位
	
	switch(voltage)
	{
		case POWERUP_5V:					//什么都不用做，已经是5V
			break;
			
		case POWERUP_3V:					//设置成3V
		    val = tda_read(PCR);
		    tda_write(PCR,val | PCR_3V_5V_MASK);
		    break;
		    
		case POWERUP_1V8:					//设置成1.8V
		  	val = tda_read(PCR);
		   	tda_write(PCR,val | PCR_1V8_MASK);
		   	break;
		   
		default:
		  	return SELECT_CARDVOLTAGE_ERROR;

	}
		
	//--------- TDA激活PSAM卡 ---------
	val = tda_read(PCR);
	tda_write(PCR,val & ~PCR_RSTIN_MASK);	//复位脚置低 

	val = tda_read(HSR);						//读HSR寄存器是为了清HSR寄存器	  
	do
	{ 
		val = tda_read(PCR);
		tda_write(PCR,val | (PCR_C8_MASK|PCR_C4_MASK|PCR_START_MASK));	//C4,C8置高 STRAT位置1，启动激活
		  
		val = tda_read(HSR);
		if (val & (HSR_PRTL2_MASK|HSR_PRTL1_MASK|HSR_PRL2_MASK|HSR_PRL1_MASK|HSR_PTL_MASK))
		{	 	
			 	printk("Power problem detected (HSR=)0x%2x\n",val);
		    tda_powerdown();
		    return TDA_POWERUP_ERROR;
		}
		val = tda_read(PCR);
	}while (!(val & PCR_START_MASK));	

	val = tda_read(PCR);							//释放复位引脚 2011-5-6 修改，测试新加坡PSAM卡时发现此问题
	tda_write(PCR,val | PCR_RSTIN_MASK);

	return 0;
}

/*****************************************************************
 函数名称：iso7816_readbyte
 函数描述：等待SPAM卡返回一个字节数据
 输入参数：timeout_ms，等待时间
 输出参数：无
 返回说明：>=0 数据，其他超时失败
 其它说明：
 *****************************************************************/
static int iso7816_readbyte(int timeout_ms)
{
	u8 val = 0;
	u8 usr = 0;
	u8 msr = 0;
	int timeout = timeout_ms * 1000 / 2;
	
	do
	{
   	  usr = tda_read(USR); 
   	  msr = tda_read(MSR);
   	  if( (usr & USR_TBE_RBF_MASK) || (!(msr & MSR_FE_MASK)) )
   	  {
	       val = tda_read(URR);		
	       return val;
	    }
	    
	    udelay(1);
	    timeout--;
	}	while (timeout > 0);
	
	return -1;
}

/*****************************************************************
 函数名称：TDA_ATRsequence
 函数描述：
 输入参数：无
 输出参数：无
 返回说明：
 其它说明：
 *****************************************************************/
static int TDA_ATRsequence(int PSAMnum, u8 speed)
{
	  u8 val;
	  int i;
	  int ret;
	  	  
	  //接收数据前初始化相关寄存器
	  val = tda_read(CSR);
	  tda_write(CSR,val & ~CSR_nRIU_MASK);			//复位串口
	  
	  val = tda_read(CSR);
	  tda_write(CSR,val | CSR_nRIU_MASK);			//使能串口
	  
	  tda_write(FCR,0x3a);  
	  tda_write(FCR,FCR_PEC_MASK);					//设置1字节FIFO深度,7次校验错误重试
	  
	  if(speed)
	  {
	  	  tda_write(PDR,3);							//设置分频  高速
	  	  g_resetspeed_buf[PSAMnum]=1;				//设置高速复位标志	
	  }	  
	  else
	  {
	  	  tda_write(PDR,12);							//设置分频 	低速	
	  	  g_resetspeed_buf[PSAMnum]=0;				//设置低速复位标志
	  }	  
	 
 	  tda_write(UCR2,0x20);							//禁止串口中断，使能自动约定,禁止INTAUX引脚中断

	  val = tda_read(UCR1);	
	  tda_write(UCR1,val|UCR1_FTE0_MASK|UCR1_SS_MASK);	//SS=1 FTE0=1
		
	  //============  复位PSAM卡 ================
	  val = tda_read(PCR);
	  tda_write(PCR,val & ~PCR_RSTIN_MASK);			//复位脚置低 
	  mdelay(10);
	  
	  tda_timer_start(0x61,0x1a,0xc3,0x00); 			//设置超时时间
			
	  val = tda_read(PCR);							//释放复位引脚 
	  tda_write(PCR,val | PCR_RSTIN_MASK);
	  
	  g_ATRlen = 0;
	  local_irq_disable();					//关中断
	  
	  for (;;)
	  {
	  	int val = iso7816_readbyte(200);
	  	if (val >= 0)
	  	{
	  			g_ATR_buf[g_ATRlen++] = val; 
	   	  //   if( g_ATRlen > 3 )
	   		   if( g_ATRlen > 6 )       //此处多接收接个字节，PPS中要判断是否是重庆PSAM卡2012-2-20 
		           break;    	  			
	  	}
	  	else
	  	{
						local_irq_enable();		//开中断
				 		return DISPOSE_PSAM_T0_TIMEOUT_ERROR;     	  			
	  	}
	  }

	 local_irq_enable();			//开中断 
	 
	 mdelay(20);
	 printk ("g_ATRlen=0x%02x\n", g_ATRlen); 
	 for(i=0;i<g_ATRlen;i++)  
	  	printk ("g_ATR_buf = 0x%02x\n", g_ATR_buf[i]);

	  //判断是否存在3B或3F
	  if( (g_ATR_buf[0] != 0x3f) && (g_ATR_buf[0] != 0x3b) ) 	
	  {
	  		return RESET_PSAM_NOT_3B3F_ERROR; 
	  }	
	  
	  //判断是T=0卡还是T=1卡 
	  if( g_ATR_buf[1] & 0x80 ) 					
	  		g_cardtype_buf[PSAMnum]=1;	
	  else
		    g_cardtype_buf[PSAMnum]=0;	
	  		
	  mdelay(10); 									//等待10ms等待复位信息结束
	  val=tda_read(USR); 	
      val=tda_read(URR); 							//清相关标志	
      
      tda_write(GTR, 00);			//设置保护时间
//    tda_write(FCR,0x40);  			//当T=0时奇偶校验出错后的重试次数
      
     //如果是低速卡再判断是否支持PPS (2010-9-26修改)
	 //之前出现过不能兼容广西PSAM卡的情况，是因为该PSAM卡虽然是低速卡，但不支持PPS，在此处加以判断
	 if( (g_ATR_buf[1]&0x10) && (speed==LOW) ) 			
	 {
      	  ret=send_pps( g_cardtype_buf[PSAMnum] ); 	  
      	  if(ret==0)
      	  {
      	  		printk ("提速成功!\n"); 
      	  		g_resetspeed_buf[PSAMnum]=1;		//设置高速复位标志	
      	  }	  	
  		  return ret;				
     } 
     
     return 0;     
} 


/*****************************************************************
 函数名称：tda_psam_reset
 函数描述：复位SPAM卡
 输入参数：psam_idx，SPAM卡编号
 输出参数：无
 返回说明：成功标识
 其它说明：
 *****************************************************************/
static int tda_psam_reset( int psam_idx )			
{	
	int ret; 

	switch(psam_idx)
	{
		case 1:
			psam_select_card(1);
			break;						//选择卡1
			
		case 2:
			psam_select_card(2);
			break;						//选择卡2	
			
		case 3:
			psam_select_card(3);
			break;						//选择卡3	
			
		case 4:
			psam_select_card(4);
			break;						//选择卡4	
			
		case 5:
			psam_select_card(5);
			break;						//选择卡5
			
		case 6:
			psam_select_card(6);
			break;						//选择卡6	
			
		default:
			return SELECT_PSAM_NUMBER_ERROR;		//选择出错	
	}
	
	//取复位响应数据
	ret = TDA_ATRsequence(psam_idx , LOW );	 
	if(ret==0)
	{
		mdelay(10);
		return 0;  
	}	
		
	ret = TDA_ATRsequence(psam_idx , HIGH );	
	mdelay(10);
	return ret;  
} 

/*****************************************************************
 函数名称：Check_SW1SW2
 函数描述：枚举SPAM卡返回的状态是否正常
 输入参数：无
 输出参数：无
 返回说明：正常标识
 其它说明：
 *****************************************************************/
static int Check_SW1SW2( u8 *data_buf, u8 len, u8 *SW_buf )
{
	if(len<2)
		return 1;				//不包含卡状态	
	
	if( ((data_buf[len-2]==0x62) && (data_buf[len-1]==0x81)) || 		//回送的数据可能有误
	    ((data_buf[len-2]==0x6a) && (data_buf[len-1]==0x80)) ||			//
	    ((data_buf[len-2]==0x6a) && (data_buf[len-1]==0x81)) ||			//
	    ((data_buf[len-2]==0x6a) && (data_buf[len-1]==0x82)) ||			//
	    ((data_buf[len-2]==0x6a) && (data_buf[len-1]==0x83)) ||			//
	    ((data_buf[len-2]==0x6a) && (data_buf[len-1]==0x84)) ||			//
	    ((data_buf[len-2]==0x6a) && (data_buf[len-1]==0x86)) ||			//
	    ((data_buf[len-2]==0x93) && (data_buf[len-1]==0x02)) ||			//
	    ((data_buf[len-2]==0x93) && (data_buf[len-1]==0x03)) ||			//
	    ((data_buf[len-2]==0x94) && (data_buf[len-1]==0x01)) ||			//
	    ((data_buf[len-2]==0x94) && (data_buf[len-1]==0x03)) ||			//
	    ((data_buf[len-2]==0x62) && (data_buf[len-1]==0x83)) ||			//选择文件无效，文件或密钥校验错误
	    ((data_buf[len-2]==0x63) && ((data_buf[len-1]&0xf0)==0xc0)) ||	//还可以再尝试X次
	    ((data_buf[len-2]==0x64) && (data_buf[len-1]==0x00)) ||			//状态标志未改写
	    ((data_buf[len-2]==0x65) && (data_buf[len-1]==0x81)) ||			//写EEROM不成功
	    ((data_buf[len-2]==0x67) && (data_buf[len-1]==0x00)) ||			//
	    ((data_buf[len-2]==0x69) && (data_buf[len-1]==0x00)) ||			//
	    ((data_buf[len-2]==0x69) && (data_buf[len-1]==0x00)) ||			//
	    ((data_buf[len-2]==0x69) && (data_buf[len-1]==0x01)) ||			//
	    ((data_buf[len-2]==0x69) && (data_buf[len-1]==0x81)) ||			//
	    ((data_buf[len-2]==0x69) && (data_buf[len-1]==0x82)) ||			//
	    ((data_buf[len-2]==0x69) && (data_buf[len-1]==0x83)) ||			//
	    ((data_buf[len-2]==0x69) && (data_buf[len-1]==0x85)) ||			//
	    ((data_buf[len-2]==0x69) && (data_buf[len-1]==0x87)) ||			//
	    ((data_buf[len-2]==0x69) && (data_buf[len-1]==0x88)) ||			//
	    
	    ((data_buf[len-2]==0x6b) && (data_buf[len-1]==0x00)) ||			//
	    ((data_buf[len-2]==0x6e) && (data_buf[len-1]==0x00)) ||			//
	    ((data_buf[len-2]==0x6f) && (data_buf[len-1]==0x00)) ||			//
	    
	    ((data_buf[len-2]==0x94) && (data_buf[len-1]==0x06))			//
	  )
	{
		SW_buf[0]=data_buf[len-2];
		SW_buf[1]=data_buf[len-1];		//设置SW1,SW2
		return 0;						//包含状态信息
	}
	
	return 1;
}

/*****************************************************************
 函数名称：Delay_us
 函数描述：原地等待多时US
 输入参数：usnum，US
 输出参数：无
 返回说明：无
 其它说明：
 *****************************************************************/
static void Delay_us(uint usnum)
{
   volatile int i,j;
   for(j=0;j<usnum;j++)
       for(i = 0; i < 20; i++);
}

//===========================================================================
//函数名称：TDA_Send_Recv_APDU
//函数说明：TDA发送命令给PSAM卡，并接收PSAM卡返回的数据，兼容T=0和T=1协议
//参    数：PSAMnum--PSAM号,*req_buf--发送命令缓冲区, req_len发送命令长度  
//			*resp_buf--响应数据缓冲区 *resp_len--响应数据长度指针
//返回值  ：0--成功 非0--失败
//===========================================================================
static int TDA_Send_Recv_APDU(u8 PSAMnum, u8 *req_buf, u8 req_len, u8 *resp_buf, u8 *resp_len)
{
  	int i,j,ret,cnt;
    u8 val,USRval,MSRval,temp; 
    u8 CMDBuf[500];
	u8 TmpBuf[500],tmplen;
	u8 SWbuf[500];
	u8 dlength = 0; 

//	printk ("g_resetspeed_buf[PSAMnum]=0x%02x\n",g_resetspeed_buf[PSAMnum]);

	//判断是否允许操作PSAM卡2011-4-19增加此条件
	if( (PSAMnum==0) || (PSAMnum>6) )
	{
		printk ("PSAM卡槽号超出范围#\n"); 
		return 1;
	}	
	if( g_RecvInfo.resetbuf[PSAMnum-1]==0 )
	{
		printk ("PSAM卡复位失败禁止执行该TimeCos指令#\n"); 
		return 1; 
	}
	
	tmplen=0;
	*resp_len=0;
	
	//----------  判断TimeCos内容是否超长  -------------
	if(req_len>100 || req_len<5)
		return TIMECOS_CMD_LEN_ERROR;	 
				 
	//-------------  选择卡片  -------------------------
	switch(PSAMnum)
	{
		case 1:psam_select_card(1);break;						//选择卡1		
		case 2:psam_select_card(2);break;						//选择卡2	
		case 3:psam_select_card(3);break;						//选择卡3	
		case 4:psam_select_card(4);break;						//选择卡4	
		case 5:psam_select_card(5);break;						//选择卡5
		case 6:psam_select_card(6);break;						//选择卡6	
		default:return SELECT_PSAM_NUMBER_ERROR;break;		//选择出错	
	}
	
	for(i=0;i<100;i++); 
	
	if(g_resetspeed_buf[PSAMnum])
	  	 tda_write(PDR,3);							//设置分频  高速 
	else
	  	 tda_write(PDR,12);							//设置分频 	低速	
	
	//-----------  设置通讯相关寄存器  ------------------
	g_cardtype = g_cardtype_buf[PSAMnum];					//设置卡协议类型
	g_cmdtype  = g_cmdtype_buf[PSAMnum];					//设置卡命令类型(对T=1卡有用)
	
	val = tda_read(UCR1);
	if( g_cardtype == 0 )
	    	tda_write(UCR1,val & ~UCR1_PROT_MASK);			//设置T=0协议
	  else
	    	tda_write(UCR1,val | UCR1_PROT_MASK);			//设置T=1协议

	//===========================================================
	//===============  T=1卡处理流程  ===========================
	j=0;
	if( g_cardtype == PROTOCOL_T1 )
	{		
		CMDBuf[j++] = 0x00; 
		CMDBuf[j++] = g_cmdtype;				//命令00/40
		CMDBuf[j++] = req_len; 					//命令长度          
		for(i=0;i<req_len;i++)
			CMDBuf[j++] = req_buf[i];			//TimeCos命令内容
		temp=cal_bcc(CMDBuf,j);
		CMDBuf[j++]=temp;						//计算BCC  
		
		tda_timer_start(0x61,0x0a,0xc3,0x00);		//设置超时时间
 		
 	 	local_irq_disable();					//关中断
 	 	ret=iso7816_sendcmd( CMDBuf, j );				//发送命令
        if(ret!=0)
        {
        	local_irq_enable();					//开中断
			return ret; 
		}
	    
	    cnt=0;
	    while(1)       
	    {			
			Delay_us(50);						//增加延时2009-12-17
			USRval = tda_read(USR);
			if (USRval & USR_TBE_RBF_MASK)		//判断串口是否有数据		
			{	
			     for(i=0;i<58001;i++) 
			     {
			     	  Delay_us(50);			//增加延时2009-12-17
			     	  USRval=tda_read(USR);
			     	  MSRval=tda_read(MSR);
			     	  if( (USRval&USR_TBE_RBF_MASK) || (!(MSRval&MSR_FE_MASK)) )
			     	  {
					       val = tda_read(URR);					
					   	   TmpBuf[tmplen++] = val; 
					   	   if( (tmplen==1) && (TmpBuf[0]!=0) )
					   	   {
					   	   		tmplen=0;
					   	   }
					   	   else if( (tmplen==2) && (TmpBuf[1]!=g_cmdtype) )
					   	   {
					   	   		if(TmpBuf[1]==0)
					   	   			tmplen=1; 
					   	   		else 
					  	   			tmplen=0;	
					   	   	} 
						   if( (tmplen>4) && (tmplen==TmpBuf[2]+4) )	//接收完成(在此判断长度，不判断9000)
						   		break;	
					   }
					   else if ( (USRval&USR_TOL3_MASK) || (i==58000) ) 
					   {
					   //		printk ("USRval=0x%02x,i=%d\n",USRval,i);	
					   //		for(i=0;i<tmplen;i++) 
					   //			printk ("TmpBuf=0x%02x\n",TmpBuf[i]);
					   		
					   		local_irq_enable();						//开中断
					   		//切换命令
							if( g_cmdtype==0 )
								g_cmdtype=0x40;
							else	
								g_cmdtype=0x00;
							g_cmdtype_buf[PSAMnum]=g_cmdtype;		//保存命令类型
					   		
					   		return DISPOSE_PSAM_T1_TIMEOUT_ERROR;   	      
					   	}  
				 } 
				 break;    
			}     
		//	else  if (USRval & USR_TOL3_MASK) 
			else  
			{	
				cnt++;  
				if(cnt==42000)  				//将超时修改成次数限制的方式,因为这个超时寄存器容易不起作用导致卡在驱动里 2009-12-17
				{
					local_irq_enable();	//开中断
					return DISPOSE_PSAM_T1_TIMEOUT_ERROR;   	      
				}	
			} 
	    } 	      
		
		local_irq_enable();				//开中断
		 
//		printk ("i=%d\n",i); 		 
//   	for(i=0;i<tmplen;i++) 
//			printk ("22TmpBuf=0x%02x\n",TmpBuf[i]); 

	    //校验bcc是否正确
		temp=0;
		for(i=0;i<tmplen;i++)		
			temp ^= TmpBuf[i]; 
		if( temp != 0)	
		{
			return PSAM_RESPONSE_BCC_ERROR;	
		}
		
		//切换命令
		if( g_cmdtype==0 )
			g_cmdtype=0x40;
		else	
			g_cmdtype=0x00;
		g_cmdtype_buf[PSAMnum]=g_cmdtype;		//保存命令类型
			
		//判断PSAM卡返回的数据是否超长
		if( TmpBuf[2]>100 )
			return PSAM_T1_RETURN_LEN_ERROR;	 
		
		//保存数据
		*resp_len=TmpBuf[2];
		for(i=0;i<TmpBuf[2];i++)	 			
			resp_buf[i]=TmpBuf[3+i]; 
		
		return NO_ERROR; 
	}	
	  
	  
	//========================================================================
	//===============  T=0卡处理流程  ========================================
	//========================================================================

	//===========  命令长度小于6时  ================= 将命令的所有字节1次发送至卡
	if(req_len < 6)
	{		
	//	tda_timer_start(0x61,0x0a,0xc3,0x00); 
		tda_timer_start(0x61,0x1a,0xc3,0x00);			//在北京测试发现取随机数指令超时，延长该时间后正常2009-7-7 林树亮于北京			
		
		local_irq_disable();						//关中断
		ret=iso7816_sendcmd( req_buf, 5 );
		if(ret!=0)
		{  
			local_irq_enable();						//开中断
			return ret;
		}
		
		cnt=0;					
	    while(1)    
	    {			
			Delay_us(50);							//增加延时2009-12-17
			USRval = tda_read(USR);
			if (USRval & USR_TBE_RBF_MASK)			//判断串口是否有数据		
			{	
			     for(i=0;i<58001;i++)
			     {
			     	  Delay_us(50);					//增加延时2009-12-17
			     	  USRval=tda_read(USR);
			     	  MSRval=tda_read(MSR);
			     	  if( (USRval&USR_TBE_RBF_MASK) || (!(MSRval&MSR_FE_MASK)) )
			     	  {
					      val = tda_read(URR);					
					   	  TmpBuf[tmplen++] = val; 
					   	  
					   	  if(tmplen>2)    
					   	  {
						 	  if( (tmplen==req_buf[4]+3) && (TmpBuf[tmplen-2]==0x90) && (TmpBuf[tmplen-1]==0x00) )		//在此增加长度判断 防止数据中有9000 林树亮 2009-8-5 于公司 
							   		break; 	
						  }
						  else   
						  {
						 	  if( (tmplen==req_buf[4]+2) && (TmpBuf[tmplen-2]==0x90) && (TmpBuf[tmplen-1]==0x00) )		//广西测试DES初始化时只有5个字节，导致此处长度判断出错（只返回2个字节的9000），现已改正2010-10-20
						 	  {
							   		//保存数据
									*resp_len=2;		 				 
									resp_buf[0]=0x90;
									resp_buf[1]=0x00; 	
									return NO_ERROR;  	 		 
							   }  		 
						  }	 	   	 	     
					   }
					   else if ( (USRval&USR_TOL3_MASK) || (i==58000) ) 
					   {
					   	//	printk ("USRval=0x%02x,i=%d\n",USRval,i);	
					   		local_irq_enable();			//开中断
					   		//检查是否存在状态信息 
							ret=Check_SW1SW2( TmpBuf, tmplen, SWbuf );		
							if(ret==0)
							{
								*resp_len=2;		 			
								for(j=0;j<2;j++)			 
									resp_buf[j]=SWbuf[j];
								return NO_ERROR; 	
							} 
							else
							{
					   			return DISPOSE_PSAM_T0_TIMEOUT_ERROR;   	      
					   		}	
					   	}  
				 } 
				 break; 
			}  
		//	else if (USRval & USR_TOL3_MASK)
			else
			{	
				cnt++;  
				if(cnt==42000)  				//将超时修改成次数限制的方式,因为这个超时寄存器容易不起作用导致卡在驱动里 2009-12-17
				{	
					local_irq_enable();		//开中断
			 		return DISPOSE_PSAM_T0_TIMEOUT_ERROR;   
			 	}	     	
			 } 
	     } 	      
	    
	    local_irq_enable();					//开中断 
		//检验是否存在命令头
		for(i=0;i<8;i++)						//在这里最多判断8次是否存在命令头				
		{										//如果8次判断都没有命令头就认为命令错误，并丢掉
			if(TmpBuf[i]==req_buf[1])		
				break;
			if(i==7)
			{
				return PSAM_T0_RESPONSE_NOT_CMD_HEADER;	
			}	
		}
			
//		for(j=0;j<tmplen;j++)	
//			printk ("TmpBuf=0x%02x\n",TmpBuf[j]);	

		//保存数据
		temp = tmplen-i-1;				//根据命令头来判断数据长度，并且保存数据
		*resp_len=temp;					//这样可以避免因为串口错误接收而丢掉有效数据
		for(j=0;j<temp;j++)			
			resp_buf[j]=TmpBuf[i+1+j];	

		return NO_ERROR; 	
	}	 
	
	//==================================================
	//===========  命令长度>=6时 =================>>>>>> 第1步 先发送命令的前5字节

		tda_timer_start(0x61,0x1a,0xc3,0x00);			
	 
		local_irq_disable();						//关中断
		ret=iso7816_sendcmd( req_buf, 5 );						
		if(ret!=0)
		{
			local_irq_enable();						//开中断
			return ret;
		}
		
		cnt=0;
		while(1)    
	    {			
			Delay_us(50);							//增加延时2009-12-17
			USRval = tda_read(USR);
			if (USRval & USR_TBE_RBF_MASK)			//判断串口是否有数据		
			{	
				for(i=0;i<58001;i++)
			     {
			     	  Delay_us(50);					//增加延时2009-12-17
			     	  USRval=tda_read(USR); 
			     	  MSRval=tda_read(MSR);
			     	  if( (USRval&USR_TBE_RBF_MASK) || (!(MSRval&MSR_FE_MASK)) )
			     	  {
					       val = tda_read(URR);					
					   	   TmpBuf[tmplen++] = val; 
					   	   if( tmplen>0 )
					   //  if( (tmplen>0) && (TmpBuf[tmplen-1]==req_buf[1]) )
						   		break;	
					   }
					   else if ( (USRval&USR_TOL3_MASK) || (i==58000) )
					   {
					   		local_irq_enable();			//开中断 
					   		//检查是否存在状态信息 
							ret=Check_SW1SW2( TmpBuf, tmplen, SWbuf );		
							if(ret==0)
							{
								*resp_len=2;		 	 		
								for(j=0;j<2;j++)			 
									resp_buf[j]=SWbuf[j];
								return NO_ERROR; 	
							} 
							else
							{
									printk("apdu timeout1...\n");
					   			return DISPOSE_PSAM_T0_TIMEOUT_ERROR;   	      
					   		}	
					   	}  
				 } 
				 break;  
			}  
		//	else  if (USRval & USR_TOL3_MASK) 
			else 
			{
				cnt++;  
				if(cnt==42000)  				//将超时修改成次数限制的方式,因为这个超时寄存器容易不起作用导致卡在驱动里 2009-12-17
				{	
					printk("apdu timeout2...\n");
					local_irq_enable();		//开中断  	
			 		return DISPOSE_PSAM_T0_TIMEOUT_ERROR;  
			 	}	  
			 }
	     } 	     

		//===========================================>>>>>> 第2步 将命令的剩余字节发送至卡
		local_irq_enable();						//开中断
		tmplen=0;
		
//		while( (tda_read(MSR)&MSR_BGT_MASK)!=MSR_BGT_MASK );		//判断是否满足块保护时间
//		for(i=0;i<400;i++);  			//杭州测试，测试完要改回来
	 
		for( i=0;i<req_len-5;i++ )
			CMDBuf[i] = req_buf[5+i];
			 
		tda_timer_start(0x61,0x0a,0xc3,0x00);			
	
		local_irq_disable();						//关中断
		ret=iso7816_sendcmd( CMDBuf, req_len-5 );						
		if(ret!=0)
		{
			local_irq_enable();						//开中断
			return ret;
		}
		
		cnt=0;
		while(1)    
	    {			
			Delay_us(50);							//增加延时2009-12-17
			USRval = tda_read(USR);
			if (USRval & USR_TBE_RBF_MASK)			//判断串口是否有数据		
			{	
				 for(i=0;i<58001;i++)
			     {
			     	  Delay_us(50);					//增加延时2009-12-17
			     	  USRval=tda_read(USR);
			     	  MSRval=tda_read(MSR);
			     	  if( (USRval&USR_TBE_RBF_MASK) || (!(MSRval&MSR_FE_MASK)) )
			     	  {
					       val = tda_read(URR);					
					   	   TmpBuf[tmplen++] = val; 
					   	   if( (tmplen>1) && (TmpBuf[tmplen-2]==0x61) )
						   {
								 dlength = TmpBuf[tmplen-1];		//此处收2个字节，保存0x61后面一个字节,就是将要返回的数据长度	
								 break;	
							}

					   	   if( (tmplen>1) && (TmpBuf[tmplen-2] ==0x90) && (TmpBuf[tmplen-1]==0x00) )
						   { 
								*resp_len=tmplen;					//保存数据长度
								for(i=0;i<tmplen;i++)				//保存数据	
						  			resp_buf[i]=TmpBuf[i];	
						  		local_irq_enable();					//开中断	
								return NO_ERROR;
							}	 	
					   }
					   else if ( (USRval&USR_TOL3_MASK) || (i==58000) )
					   {	
						   		local_irq_enable();						//开中断
						   		//检查是否存在状态信息 
								ret=Check_SW1SW2( TmpBuf, tmplen, SWbuf );		
								if(ret==0)
								{ 
									*resp_len=2;		 			
									for(j=0;j<2;j++)			 
										resp_buf[j]=SWbuf[j];
									return NO_ERROR; 	 
								} 
								else
								{
										printk("apdu timeout...\n");
						   			return DISPOSE_PSAM_T0_TIMEOUT_ERROR;   	      
						   		}	
					   	}  
				 } 
				 break;    
			}  
		//	else if (USRval & USR_TOL3_MASK)
			else
			{
				cnt++;  
				if(cnt==48000)  					//将超时修改成次数限制的方式,因为这个超时寄存器容易不起作用导致卡在驱动里 2009-12-17
				{	
					printk("apdu timeout4...\n");
					local_irq_enable();			//开中断
					return DISPOSE_PSAM_T0_TIMEOUT_ERROR; 	 
				}	
			}
	     } 	     

		//===========================================>>>>>> 第3步 取数据
		local_irq_enable();				//开中断
		tmplen=0;

//		while( (tda_read(MSR)&MSR_BGT_MASK)!=MSR_BGT_MASK );		//判断是否满足块保护时间	
//		for(i=0;i<1000;i++);		//杭州测试，测试完要改回来
		
		tmplen=0;
		CMDBuf[0] = 0x00; 
		CMDBuf[1] = 0xc0;
		CMDBuf[2] = 0x00;
		CMDBuf[3] = 0x00;
		CMDBuf[4] = dlength;			//取数据命令
		
		tda_timer_start(0x61,0x0a,0xc3,0x00);			
		
		local_irq_disable();						//关中断	
		ret=iso7816_sendcmd( CMDBuf, 5 );	
		if(ret!=0)
		{
			local_irq_enable();						//开中断
			return ret;
		}
		
		cnt=0; 
		while(1)    
	    {				
			Delay_us(50);							//增加延时2009-12-17
			USRval = tda_read(USR);
			if (USRval & USR_TBE_RBF_MASK)			//判断串口是否有数据		
			{	
				for(i=0;i<58001;i++)					//因为在读车辆信息较长的时候超时时间不够，所以延长了2009-5-17
			    {
			     	  Delay_us(50);					//增加延时2009-12-17
			     	  USRval=tda_read(USR);
			     	  MSRval=tda_read(MSR);
			     	  if( (USRval&USR_TBE_RBF_MASK) || (!(MSRval&MSR_FE_MASK)) )
			     	  {
					       val = tda_read(URR);					
					   	   TmpBuf[tmplen++] = val; 
					  	   if( (tmplen>1) && (TmpBuf[tmplen-2] ==0x90) && (TmpBuf[tmplen-1]==0x00) )
						   {
								//------- 检验是否存在命令头0xc0 -----------
								for(i=0;i<6;i++)						//在这里最多判断6次是否存在命令头0xc0,如果6次判断都没有命令头就认为命令错误，并丢掉				
								{										
									if( TmpBuf[i]==0xc0 ) 				
										break;
									if(i==5)  
									{	 
										local_irq_enable();				//开中断
										return PSAM_T0_RESPONSE_NOT_C0;						
									}	
								}
								if( (tmplen-i-1)==(dlength+2) )			//检测到命令头后再检验长度是否正确   (之前解决数据中有9000的逻辑有bug，现在已经改正了林树亮 2009-8-5 修改)
								{	
									*resp_len=tmplen-1;					//保存数据长度
									for(i=0;i<tmplen-1;i++)				//保存数据
								  		resp_buf[i]=TmpBuf[i+1];	 
									break;	
								}		  
							}		   
					   }
					   else if ( (USRval&USR_TOL3_MASK) || (i==58000) )
					   {
					   		local_irq_enable();			//开中断
					   		//检查是否存在状态信息 
							ret=Check_SW1SW2( TmpBuf, tmplen, SWbuf );		
							if(ret==0)
							{
								*resp_len=2;		 			
								for(j=0;j<2;j++)			 
									resp_buf[j]=SWbuf[j];
								return NO_ERROR; 	
							} 
							else
							{
					   			return DISPOSE_PSAM_T0_TIMEOUT_ERROR;   	      
					   		}	
					   }  
				 }  
				 break;  
			}  
		//	else   if (USRval & USR_TOL3_MASK)
			else
			{
				cnt++;  
				if(cnt==42000)  					//将超时修改成次数限制的方式,因为这个超时寄存器容易不起作用导致卡在驱动里 2009-12-17
				{	
					local_irq_enable();			//开中断
					return DISPOSE_PSAM_T0_TIMEOUT_ERROR;  	
				}	 
			}
	    } 	      
   
//	 for(j=0;j<tmplen;j++)	
//		printk ("TmpBuf=0x%02x\n",TmpBuf[j]);	 
	//-------------- 
	
	local_irq_enable();				//开中断
	
	return NO_ERROR; 
	
} 

/*****************************************************************
 函数名称：gv_ioctl
 函数描述：功能性IO操作接口
 输入参数：filp，文件信息结构
		   cmd，命令号
		   arg，选择性输入参数
 输出参数：arg，选择性输出结果
 返回说明：成功标识
 其它说明：
 *****************************************************************/
static long gv_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    //struct gv_dev *dev = filp->private_data;
	//struct gv_tda_io *dio = dev->dio;
	int ret = 0;
	int psam_num = arg;
	int i, j;

	switch (cmd)
	{
		//========== 0 发送APDU个PSAM卡并取响应数据  ================
		case IOCTL_PSAM_APDU_DISPOSE:
			
			ret = copy_from_user( &g_SendInfo,(IoctlInfo *)arg ,sizeof(IoctlInfo) );		//从用户态拷贝数据    	    

#if 0		    
		    printk ("apdu send len = %d: ", g_SendInfo.datalen); 	
	 		for(i=0;i<g_SendInfo.datalen;i++) 
	  			printk ("%02x ", g_SendInfo.databuf[i]); 			
		    printk("\n");
#endif
		    g_RecvInfo.psamnum = g_SendInfo.psamnum;
		    ret=TDA_Send_Recv_APDU( g_SendInfo.psamnum, g_SendInfo.databuf, g_SendInfo.datalen, g_RecvInfo.databuf, &g_RecvInfo.datalen );	
			if(ret==0) 
			{
			 	ret = copy_to_user( (IoctlInfo *)arg, &g_RecvInfo ,sizeof(g_RecvInfo) );	//将结果拷贝到用户态 	 		 	
#if 0			
				printk("apdu recv, len = %d: ", g_RecvInfo.datalen);
				for(i=0;i<g_RecvInfo.datalen;i++) 
	 				printk ("%02x ", g_RecvInfo.databuf[i]); 	
				printk("\n");
#endif
				
			}  
			
			else
			{
				printk("tda rsp error, ret = %02x\n", ret);							
			}
	
			return ret;		
		
		//============ 1  复位单个PSAM卡操作  ========================
		case IOCTL_ONE_PSAM_RESET_DISPOSE:			
	
			if( (arg>0) && (arg<7) )
			{
				psam_led_control(psam_num , 0);				//复位先先关“PSAM卡存在”指示灯
				ret=tda_psam_reset(psam_num);		//复位PSAM卡
				if(ret<0)
				{ 
					psam_led_control(psam_num , 0);			//关“PSAM卡存在”指示灯
					g_RecvInfo.resetbuf[psam_num-1] = 0;		//失败 

					printk ("Reset PSAM%d ERROR!\n", psam_num);
				}	
				else
				{
					psam_led_control( psam_num , 1);			//亮“PSAM卡存在”指示灯
					g_RecvInfo.resetbuf[psam_num-1] = 1;		//成功
				
					if(g_resetspeed_buf[psam_num])
					{
						printk ("HIGH Speed Reset PSAM%d OK , T=%d\n", psam_num, g_cardtype_buf[psam_num]); 	
					}
					else
					{
						printk ("LOW Speed Reset PSAM%d OK , T=%d\n", psam_num, g_cardtype_buf[psam_num]); 	
					}
			
				}	
 
				g_cmdtype_buf[ psam_num ] = 0;		//清命令类型 
				
				//对所有检测到的PSAM卡执行取终端机编号操作2011-4-19
			//	ret=get_all_psam_termid();
	
				return ret;	  
			}
	 
			return 1; 
			

		
		//================ 3  TDA上电操作  ==========================
		case IOCTL_TDA_POWERUP_DISPOSE:			
			
			 printk("TDA PowerUP ...! \n");

			
			tda_init();						//TDA8007初始化
    		tda_select_channel(1);			//选择通道A
			ret=tda_powerup( POWERUP_3V );	//8007上电、复位 
			if(ret<0)
				printk("TDA PowerUP Error! \n");
    		else
    			printk("TDA PowerUP OK! \n");
			return ret;
		
		//============== 4  TDA掉电操作  ==========================
		case IOCTL_TDA_POWERDOWN_DISPOSE:			
			
			tda_select_channel(1);
			tda_powerdown(); 
			tda_select_channel(2);
			tda_powerdown();				//关闭卡片

  		printk("Power Down\n");
	
			//P1_C1_L;
			//P1_C2_L; 
			//P1_C3_L;						//PSAM卡选择都输出低
			
			return NO_ERROR;


		//============== 2 复位所有PSAM卡操作  ========================
		case IOCTL_ALL_PSAM_RESET_DISPOSE:			

			for (i=1; i<7; i++)
			{
					 psam_led_control(i, 0);
			}
			
			for(i=1;i<7;i++)
			{
				ret=tda_psam_reset( i );				//复位PSAM卡
				if(ret<0)
				{
					psam_led_control( i , 0);					//关“PSAM卡存在”指示灯
					g_RecvInfo.resetbuf[i-1] = 0;		//失败
				}	
				else
				{
					psam_led_control( i , 1);					//亮“PSAM卡存在”指示灯
					g_RecvInfo.resetbuf[i-1] = 1;		//成功	
				}
				g_cmdtype_buf[ i ] = 0;					//清命令类型
			}
			
			ret = copy_to_user( (IoctlInfo *)arg, &g_RecvInfo ,sizeof(g_RecvInfo) );	//将结果拷贝到用户态 	 		 	
			
			return 0;


		case IOCTL_ReadPsamInfo_DISPOSE: 						
			j=0;
			for(i=0;i<6;i++)
				PSAMInfoBuf[j++]=g_RecvInfo.resetbuf[i];	//复位信息
			for(i=0;i<6;i++) 
				PSAMInfoBuf[j++]=g_resetspeed_buf[1+i];		//卡速信息 
			for(i=0;i<6;i++)
				PSAMInfoBuf[j++]=g_cardtype_buf[1+i];		//卡类型信息 
			
			ret = copy_to_user((void *)arg, PSAMInfoBuf ,18 ); 	
			return NO_ERROR; 
				
		default:
			printk("drv_tda ioctl cmd %d not support\n", cmd);
			return -1;
	}


    return 0;
}

/*****************************************************************
 函数名称：gv_open
 函数描述：打开地感设备接口
 输入参数：inode,设备节点
		   filp，文件参数
 输出参数：无
 返回说明：成功标识
 其它说明：
 *****************************************************************/
static int gv_open(struct inode *inode, struct file *filp)
{
    struct gv_dev *dev = container_of(inode->i_cdev, struct gv_dev, cdev);

    unsigned long flags = 0;

    //mutex open    
    if (!mutex_trylock(&dev->open_lock))
    {
        printk("tda dev already open, return.\n");
        return -1;
    }

    spin_lock_irqsave(&dev->recv_lock, flags);
    filp->private_data = dev;   
    dev->recv_ready = 0;
    spin_unlock_irqrestore(&dev->recv_lock, flags); 

    return 0;
}

/*****************************************************************
 函数名称：gv_release
 函数描述：释放地感设备接口
 输入参数：inode,设备节点
		   filp，文件参数
 输出参数：无
 返回说明：成功标识
 其它说明：
 *****************************************************************/
static int gv_release(struct inode *inode, struct file *filp)
{
    struct gv_dev *dev = container_of(inode->i_cdev, struct gv_dev, cdev);
    
    filp->private_data = dev;
    mutex_unlock(&dev->open_lock);

    return 0;
}

/*****************************************************************
 函数名称：gv_poll
 函数描述：等待列队查询资源是否就绪, 系统调用select内部被使用
 输入参数：filp，文件参数
		   table,等待列队
 输出参数：无
 返回说明：> 0,资源可用
 其它说明：
 *****************************************************************/
static unsigned int gv_poll(struct file *filp, poll_table *wait)
{
    unsigned int mask;
    unsigned long flags;
    struct gv_dev *dev = filp->private_data;
    
    poll_wait(filp, &dev->recv_wait, wait);

    spin_lock_irqsave(&dev->recv_lock, flags);
    if (dev->recv_ready)
    {
        mask =   (POLLIN | POLLRDNORM);
    }
    else           
    {
        mask = 0;
    }
    spin_unlock_irqrestore(&dev->recv_lock, flags);

    return mask;
}

/*****************************************************************
 函数名称：gv_write
 函数描述：设备写接口
 输入参数：file，文件信息结构
		   buf，写数据缓存
		   count，写数据大小
		   f_pos，写入位置
 输出参数：f_pos，更新写入后位置
 返回说明：<0，写入失败，其他为写入成功数据大小
 其它说明：
 *****************************************************************/
static ssize_t gv_write(struct file *filp, const char __user *buf,
        size_t count, loff_t *f_pos)
{
	int ret;
	u8 sbuf[256];
    struct gv_dev *dev = filp->private_data;
	
	if (count > 126)
	{
		printk("tda send buf too long, len = %d\n", count);
		return -1;
	}

	dev->recv_ready = 0;
	ret = copy_from_user(sbuf, (u8 *)buf, count);
	sbuf[count] = cal_bcc(sbuf, count);

	return count + 1;
	//return tda_write(dev, sbuf, count+1);
}

/*****************************************************************
 函数名称：gv_read
 函数描述：设备读接口
 输入参数：file，文件信息结构
		   count，读数据大小
		   f_pos，读位置
 输出参数：buf，读数据缓存
		   f_pos，更新读后的位置
 返回说明：<0，读失败，其他为读取成功数据大小
 其它说明：
 *****************************************************************/
static ssize_t gv_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    int ret = 0;
    struct gv_dev *dev = filp->private_data;

	while (dev->recv_ready == 0)
	{
		ret = wait_event_interruptible(dev->recv_wait, dev->recv_ready);
		if (ret < 0)
		{
			break; //interrupt by signal
		}
	}
	
	if ((dev->recv_ready > 0) && (dev->rlen > 0))
    {	
		ret = copy_to_user(buf, dev->recv_buf, dev->rlen);
		dev->recv_ready = 0;
		return dev->rlen;
	}

    return -1;	
}


//设备文件操作
static struct file_operations gv_fops = {
    .owner =    THIS_MODULE,
    .write =    gv_write,
    .read =     gv_read,
    .unlocked_ioctl = gv_ioctl,
    .open =     gv_open,
    .release =  gv_release,
    .llseek =   no_llseek,
    .poll   = gv_poll,
};

/*****************************************************************
 函数名称：cdev_setup
 函数描述：初始化字符设备并创建设备文件
 输入参数：pdata，设备私有数据
		major，字符设备主设备号
		minor，字符设备从设备号
		fops，字符设备文件操作
 输出参数：无
 返回说明：无
 其它说明：
 *****************************************************************/
static void cdev_setup(struct gv_dev *pdata, int major, int minor, struct file_operations *fops)
{
    struct cdev *cdev = &pdata->cdev;   
    struct device *dev;
    int err;
    dev_t devno = MKDEV(major, minor);

    cdev_init(cdev, fops);
    cdev->owner = THIS_MODULE;
    cdev->ops = fops;
    
    err = cdev_add(cdev, devno, 1);
    if (err)
    {
        printk("gpmc add cdev error\n");
    }   
    
    dev = device_create(tda_class, &pdata->pdev->dev, devno, pdata, "%s", pdata->name);
    if (!IS_ERR(dev))
    {
        pdata->devno = devno;       
    }
}

/*****************************************************************
 函数名称：cdev_release
 函数描述：注销字符设备并删除设备文件
 输入参数：dev，设备私有数据
 输出参数：无
 返回说明：无
 其它说明：
 *****************************************************************/
static void cdev_release(struct gv_dev *dev)
{
    cdev_del(&dev->cdev);

    if (dev->devno)
    {
        device_destroy(tda_class, dev->devno);
    }
}

/*****************************************************************
 函数名称：tda_irq_handler
 函数描述：芯片操作就绪中断服务
 输入参数：irq，中断编号
		   dev_id，中断私有数据
 输出参数：无
 返回说明：中断处理成功标识
 其它说明：
 *****************************************************************/
static irqreturn_t tda_irq_handler(int irq, void *dev_id)
{
    struct gv_dev  *dev = dev_id;

	//dev->rlen = tda_read(dev);

	dev->recv_ready = 1;
	wake_up_interruptible(&dev->recv_wait);

    return IRQ_HANDLED;
}

/*****************************************************************
 函数名称：gv_drv_probe
 函数描述：设备探测函数，设备与驱动配对后探测能否正常驱动
 输入参数：pdev，设备参数
 输出参数：无
 返回说明：成功标识
 其它说明：
 *****************************************************************/
static int __devinit gv_drv_probe(struct platform_device *pdev)
{
    int retval = -1;
    struct gv_dev  *pdata;
    int res_size;
    struct resource *res, *irq_res;


    int irq_flags = IORESOURCE_IRQ_HIGHEDGE; //IORESOURCE_IRQ_LOWEDGE;

    pdata = kzalloc(sizeof(struct gv_dev), GFP_KERNEL);    
    dev_set_drvdata(&pdev->dev, pdata);
		tda_io = pdata->dio = pdev->dev.platform_data;
	

    pdata->id = pdev->id;
    pdata->pdev = pdev;

    printk("tda_drv_probe, pdev-id = %d, rw_io=%d...\n", pdev->id, pdata->dio->io_wr_out);


    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        printk("platform_get_resource idx = %d error\n", 0);
        return -1;
    }
    res_size = resource_size(res);

    pdata->ioaddr = ioremap_nocache(res->start, res_size);
    if (pdata->ioaddr == NULL) {
        printk("ioremap error\n");
        return -2;
    }
		tda_addr = pdata->ioaddr;

    irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if (!irq_res) {
        printk("drv_tda Could not allocate irq resource.\n");
        return -1;
    }

    pdata->irq = irq_res->start;
    irq_flags = irq_res->flags & IRQF_TRIGGER_MASK;


     retval = request_irq(pdata->irq, tda_irq_handler, irq_flags, "tda_irq", pdata);			
    
    if (retval) 
    {
        printk( "Unable to claim requested irq: %d\n", pdata->irq);
    }

    printk("id = %d, start = %08x, size = %d, ioaddr = %08x, irq = %d\n", pdev->id, res->start, res_size, (unsigned int)pdata->ioaddr, pdata->irq);



		gv_io_out(pdata->dio->io_wr_out, 0); 


    spin_lock_init(&pdata->recv_lock);
    init_waitqueue_head(&pdata->recv_wait);
    pdata->recv_ready = 0;

    mutex_init(&pdata->open_lock);

    pdata->name = DEV_FILE_NAME;
    cdev_setup(pdata, DEV_MAJOR, pdata->id, &gv_fops);

#if 0
	{
		int ret = 0;

	tda_init();								//TDA8007初始化
	
    tda_select_channel(1);					//选择通道A
	ret = tda_powerup( POWERUP_3V );		//8007上电、复位 	
	if(ret<0)
		printk("TDA PowerUP Error! \n");
    else
    	printk("TDA PowerUP OK! \n"); 

	mdelay(10); 		
		
		ret=tda_psam_reset(1);		
		if(ret<0)
		{
			printk ("Reset PSAM%d ERROR ## \n",1);  		
		}  		
	}
#endif

    return 0;
}

/*****************************************************************
 函数名称：gv_drv_remove
 函数描述：注销设备的资源
 输入参数：pdev，设备参数
 输出参数：无
 返回说明：无
 其它说明：
 *****************************************************************/
static int gv_drv_remove(struct platform_device *pdev)
{
    struct gv_dev  *pdata = dev_get_drvdata(&pdev->dev);
    printk("g90 fpga remove, id = %d\n", pdata->id);

    free_irq(pdata->irq, pdata);
    iounmap(pdata->ioaddr);

    kfree(pdata);
    cdev_release(pdata);    

    return 0;
}

//GPIO片内资源，平台设备驱动
static struct platform_driver gv_driver = {
    .probe = gv_drv_probe,
    .remove = __devexit_p(gv_drv_remove),
    .driver = {
        .name   = DEV_NAME,
        .owner  = THIS_MODULE,
    },
};

/*****************************************************************
 函数名称：gv_init
 函数描述：模块加载时初始化
 输入参数：无
 输出参数：无
 返回说明：成功标识
 其它说明：
 *****************************************************************/
static int __init gv_init(void)
{
    printk("G90 gpmc init, driver version = %s.\n", G_VER);

    tda_class = class_create(THIS_MODULE, "tda_class");
    return platform_driver_register(&gv_driver);
}

/*****************************************************************
 函数名称：gv_exit
 函数描述：模块注销时释放资源
 输入参数：无
 输出参数：无
 返回说明：无
 其它说明：
 *****************************************************************/
static void __exit gv_exit(void)
{
    printk("G90 gpmc deinit.\n");	
    platform_driver_unregister(&gv_driver);
    class_destroy(tda_class);    
}



module_init(gv_init);
module_exit(gv_exit);

MODULE_LICENSE("GPL"); 
MODULE_AUTHOR("R.wen");
   
