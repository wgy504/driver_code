
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/irq.h>
#include <linux/swab.h>
#include <linux/device.h>
#include <mach/gpio.h>
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
#define G90_FPGA        "g90_fpga"
#define PSAM_DEV_NAME       "psamdev"
#define WLM_DEV_NAME        "wlmdev"
#define WLM_MAJOR       241 



#define GPIO_OUT_INFO_LED1   (0)        //GPIO0[0]
#define GPIO_OUT_INFO_LED2   (1)        //GPIO0[1]

//some GPIO for controling
#define  FPGA_CS1_POWER     47   //GP1_15, use for fpga out at sigle-board version
#define  FPGA_WLM_SEND      58  //use for fpga out at sigle-board version


#define POWER_ON        (0)
#define POWER_OFF       (1)

//device buffer, now using for psam read/write only
#define BUF_MAX_LEN     8192
#define ALLOC_BUF_LEN       (BUF_MAX_LEN * 4)


//len read first for psam cmd
#define PSAM_CMD_HEADER_LEN     20 
//max erea for psam
#define PSAM_AREA_MAX           20
//psam cmd max len
#define PSAM_DATA_LEN_MAX       256
//psam platform id = 0
#define PSAM_DEV_ID 0


//4 area for each wlm-module(4 * 5 = 20)
#define WLM_MODULE_AREA     4	//新版的改为8路.duanshichao
//5 chip-select for wlm
#define GPMC_CS_NUM_MAX     2  
#define WLM_AREA_MAX        (WLM_MODULE_AREA * GPMC_CS_NUM_MAX)

#define WLM_DATA_LEN_MAX    1024



//================the psam fpga memory map =======================
#define PSAM_INFO_SIZE  (23 * 1024)

#define     IRQ_STATUS_PSAM_DATA            0x80
#define     IRQ_STATUS_PSAM_UPDATE_NEW      0x40 
#define     IRQ_STATUS_PSAM_UPDATE          0x20

//NOTE: packed me!!!
struct psam_public_info
{
    u16  revid;
    u16  status;
    short temp[2];
    u32 card_exist; 
    u8  irq_status;
    u8 resv[1024-13];
};

struct psam_update_cmd
{
    short len;
    u8    data[1024 - 2];   
};

struct psam_update_rq
{
    short len;
    u16   crc; //bcc
    u32   sn;
    u16   file_size; //use at sn=0 only, kbytes
    u16   slen;
    u8    data[1024 - 12];
};

struct psam_update_rs
{
    short len;
    u16   status;
    u32   sn;
    u8    resv[1024-8];
};


struct psam_update_info
{
    struct psam_update_rq rq;
    struct psam_update_rs   rs; 
    //u8 r[2048];
};

struct psam_scmd_info
{
    short len;
    u8     ctl1;
    u8    ctl2;
    u32 timeout;
    u8    type;
    u8    slen; 

    u8    data[512 - 10];
};

struct psam_rcmd_info
{
    short len;
    u8     sr1;
    u8    sr2;
    u8    type;
    u8    rlen; 
    u8    data[512 - 6];
};

struct psam_cmd_info
{
    struct psam_scmd_info  si;
    struct psam_rcmd_info  ri;
};

struct psam_info 
{
    struct psam_public_info pi;
    struct psam_update_info ui;
    struct psam_cmd_info ci[20];
};


static struct psam_info psam_info_fake;
#define psam_offset(member)  ((u8 *)&psam_info_fake.member - (u8 *)&psam_info_fake)


//============= psam cmd ===========
struct psam_cmd_send_info
{
    int psam_id;
    struct psam_scmd_info si;
};


struct psam_cmd_recv_info
{
    int psam_id;
    struct psam_rcmd_info ri;
};


//============= wlm cmd ===========
struct wlm_cmd_info
{
    short  idx;
    short  len;
    char   data[1020];
};

struct cmd_data_dump
{
    short  idx;
    short  rw;      
    char   data[1024];
};

struct  psam_cmd_temp
{
    short temp1;
    short temp2;
};


//============= ioctl cmd ================
#define GPMC_IOC_MAGIC          'k'

//rw
#define GPMC_IOC_SEND_PULSE     _IO(GPMC_IOC_MAGIC, 1)  
#define GPMC_IOC_CMD_PSAM_UPDATE        _IO(GPMC_IOC_MAGIC, 2)  
    
#define GPMC_IOC_WLM_SYNC       _IO(GPMC_IOC_MAGIC, 10)


//read
#define GPMC_IOC_GET_TEMP       _IOR(GPMC_IOC_MAGIC, 3, struct  psam_cmd_temp)  
#define GPMC_IOC_PSAM_GET_CARD_INFO     _IOR(GPMC_IOC_MAGIC, 4, u32)    
#define GPMC_IOC_GET_REVID          _IOR(GPMC_IOC_MAGIC, 5, u16)    
#define GPMC_IOC_CMD_RECV       _IOR(GPMC_IOC_MAGIC, 6, struct psam_cmd_recv_info)
#define GPMC_IOC_CMD_WLM_RECV_RAW   _IOR(GPMC_IOC_MAGIC, 8, struct wlm_cmd_info)
#define GPMC_IOC_DATA_DUMP      _IOR(GPMC_IOC_MAGIC, 9, struct cmd_data_dump)
#define GPMC_IOC_DATA_CLEAR     _IOR(GPMC_IOC_MAGIC, 10, u32)

#define GPMC_IOC_GET_CARD_EXIST     _IOR(GPMC_IOC_MAGIC, 11, int)


//write
#define GPMC_IOC_CMD_SEND       _IOW(GPMC_IOC_MAGIC, 3, struct psam_cmd_send_info)
#define GPMC_IOC_CMD_RECV_TEST      _IOW(GPMC_IOC_MAGIC, 7, u32)
#define GPMC_IOC_CMD_WLM_SEND_RAW   _IOW(GPMC_IOC_MAGIC, 8, struct wlm_cmd_info)

#define GPMC_IOC_BUF_WRITE      _IOW(GPMC_IOC_MAGIC, 9, struct cmd_data_dump)

#define GPMC_IOC_ANT_CS         _IOW(GPMC_IOC_MAGIC, 10, unsigned char)

//同步检测
#define SYNC_IOC_EDGE_TRIGGER   _IO(GPMC_IOC_MAGIC, 20) 
#define SYNC_IOC_WAIT_SIGNAL    _IO(GPMC_IOC_MAGIC, 22) 
#define SYNC_IOC_WAIT_TIMEOUT   _IO(GPMC_IOC_MAGIC, 23) 

//地感检测
#define SWITCH_IOC_GET_COUNT   _IO(GPMC_IOC_MAGIC, 25) 
#define SWITCH_IOC_CLR_COUNT   _IO(GPMC_IOC_MAGIC, 26)

#define SWITCH_IOC_SIGNAL_DETECT	_IO(GPMC_IOC_MAGIC, 27)
/* 添加地感驱动程序 */
#define IRQ_SW1_SIGNAL	6
#define IRQ_SW2_SIGNAL	25
#define SWITCH1_PIN	5
#define SWITCH2_PIN	32+19	//GP1-19
static int sw1_irq     = OMAP_GPIO_IRQ(IRQ_SW1_SIGNAL);
static int sw2_irq     = OMAP_GPIO_IRQ(IRQ_SW2_SIGNAL);
static int sw1_count   = 0;
static int sw2_count   = 0;
//==================gpmc device =====================

static struct class *gpmc_class;

struct am389x_gpmc_dev 
{
    void __iomem *ioaddr[GPMC_CS_NUM_MAX];  
    char cs_mod_name[GPMC_CS_NUM_MAX][32];
    char *name;
    int id;
    int irq[GPMC_CS_NUM_MAX];
    int busw;
    int cs_count;

    struct platform_device *pdev;
    struct cdev cdev;
    dev_t devno; //device node num

    int recv_wait_done;
    wait_queue_head_t recv_wait;
    u32 recv_ready;
    int update;

    u32 recv_time; //us
    u32 wake_time;
    u32 wakeup_time;
    u32 user_read_time;


    spinlock_t recv_lock;
    int recv_len[20];

    struct mutex open_lock;

    void *devbuf;
};

//未用到的静态函数，会产生编译器报怨，调试时再打开duanshichao
#if 0
static u32 get_time(void)
{
    struct timespec ts;

    getnstimeofday(&ts);
    return  ((ts.tv_sec%1000) * 1000 + (ts.tv_nsec / 1000));
}

#endif

//=================== GPMC commom functions ==========================

#if 0
static void gpmc_dump(char *prefix, unsigned char *buf, int len)
{
    int i=0; 
    
    if (prefix)
    {
        printk("%s\n", prefix);
    }
    else
    {
        printk("dump: \n");
    }   

    for (i=0; i<len; i++)
    {
        printk("%02x ", buf[i]);        
        
        if ((i+1) % 16 == 0)
        {
            printk("\n");
        }
    }
    
    printk("\n");
}
#endif

static int gpmc_read(struct am389x_gpmc_dev *dev, int cs_idx, int addr, unsigned char *buf, int len)
{
    int i;
    volatile unsigned char *offset  = ((volatile unsigned char *)(dev->ioaddr[cs_idx])) + addr;

//  printk("gpmc read out offset: %d, len = %d\n", addr, len);
//  printk("ioaddr = %p, cs_idx = %d\n", dev->ioaddr[cs_idx], cs_idx);
    

    if ((addr + len) > ALLOC_BUF_LEN)
    {
        return -1;
    }


    for (i=0; i<len; i++)
    {
        buf[i] = offset[i];
    }


#if 0
    if (len == 2 && buf[1] == 0)
    {
        return len;
    }

    printk("gpmc read out offset: %d, len = %d\n", addr, len);
    gpmc_dump("gpmc read out: \n", buf, len);
#endif

    return len;
}


static int gpmc_write(struct am389x_gpmc_dev *dev, int cs_idx, int addr, unsigned char *buf, int len)
{
    int i;
    volatile unsigned char *offset  = ((volatile unsigned char *)(dev->ioaddr[cs_idx])) + addr;

    if ((addr + len) > ALLOC_BUF_LEN)
    {
        return -1;
    }

#if 0
    printk("gpmc write out cs = %d, offset: %d, len = %d\n", cs_idx, addr, len);
    gpmc_dump("gpmc write out: \n", buf, len);
#endif

    for (i=0; i<len; i++)
    {
        offset[i] = buf[i];
    }

    return len;
}


static inline void gpmc_io_out(int ionum, int val)
{   
    gpio_set_value(ionum, val);
}




<<<<<<< .mine
#define GPIO_SYNC_IN		7		//GP0[7]

#define	GPIO_WLM_SYNC		21 			//GP0[21]
=======
#define GPIO_WLM_SYNC       21          //GP0[21]
>>>>>>> .r1533

#define WLM_ANT_CS_NUM  (4)
int io_wlm_an_cs[WLM_ANT_CS_NUM] = {49, 50, 42, 26};

static int sync_io_gpio_setup(void)
{
    int i = 0;

<<<<<<< .mine
	//for sync in
	if ((gpio_request(GPIO_SYNC_IN, "GPIO_SYNC_IN") == 0) &&
	    (gpio_direction_input(GPIO_SYNC_IN) == 0)) {
		gpio_export(GPIO_SYNC_IN, 0);
	} 


	if ((gpio_request(GPIO_WLM_SYNC, "GPIO_WLM_SYNC") == 0) &&
	    (gpio_direction_output(GPIO_WLM_SYNC, 1) == 0))
	{
		gpio_export(GPIO_WLM_SYNC, 0);
	}
=======
    if ((gpio_request(GPIO_WLM_SYNC, "GPIO_WLM_SYNC") == 0) &&
        (gpio_direction_output(GPIO_WLM_SYNC, 1) == 0))
    {
        gpio_export(GPIO_WLM_SYNC, 0);
    }
>>>>>>> .r1533

    for (i=0; i<WLM_ANT_CS_NUM; i++)
    {
        
        if ((gpio_request(io_wlm_an_cs[i], "GPIO_WLM_ANT_CS") == 0) &&
            (gpio_direction_output(io_wlm_an_cs[i], 1) == 0))
        {
            gpio_export(io_wlm_an_cs[i], 0);
        }
    }

    return 0;
}


static void wlm_io_set_cs(unsigned char cs)
{
    int i=0; 

    for (i=0; i<WLM_ANT_CS_NUM; i++)
    {
        if (cs & (0x01 << i))
        {               
            gpmc_io_out(io_wlm_an_cs[i], 1);
        }
        else
        {
            gpmc_io_out(io_wlm_an_cs[i], 0);
        }
    }
}

/*
  同步输入信号中断处理程序
*/
#define IRQ_SYNC_SIGNAL 0x07    //同步检测信号引脚 duanshichao
static int sync_irq     = OMAP_GPIO_IRQ(IRQ_SYNC_SIGNAL);
static int sync_signal  = 0;
static DECLARE_WAIT_QUEUE_HEAD(sync_waitq);

static irqreturn_t sync_irq_handler(int irq, void * dev_id)
{       
    if(irq == sync_irq) 
    {       
        sync_signal = 1;        
        wake_up_interruptible(&sync_waitq);     
        //printk("sync interrupt is done irq = %d.\r\n",gpio_get_value(IRQ_SYNC_SIGNAL));
    }
    return IRQ_HANDLED;
}

//==================== psam file operation functions ==============

static irqreturn_t psam_irq_handler(int irq, void *dev_id)
{
    int i = 0, offset = 0, ret = 0;
    unsigned long flags = 0;
    struct am389x_gpmc_dev *dev = dev_id;
    u8 irq_status;
    
#if 1
    offset = psam_offset(pi.irq_status);
    gpmc_read(dev, 0, offset, &irq_status, 1); 
    gpmc_write(dev, 0, offset, (u8 *)&flags, 1); 

    //printk("irq = %02x\n", irq_status);
#endif

    if ((irq_status & IRQ_STATUS_PSAM_UPDATE) ||  (irq_status & IRQ_STATUS_PSAM_UPDATE_NEW))
    {
        u16 us;

        offset = psam_offset(ui.rs);
        gpmc_read(dev, 0, offset, (u8 *)&us, 2); 
        //printk("offset = %d, len = %04x\n", offset, us);  
        if (us & 0x80) //get respond
        {
            //printk("irq u, get data\n");
            dev->update = 1;
            goto out;           
        }
    }


    if (irq_status & IRQ_STATUS_PSAM_DATA)
    {
        for (i=0; i<PSAM_AREA_MAX; i++)
        {
            int len = 0;  
            char buf[2] = {0, 0};

            offset = psam_offset(ci[i].ri);
            ret = gpmc_read(dev, 0, offset, buf, 2); 
            if (ret < 0)
            {
                printk("irq read gpmc error\n");
                return IRQ_HANDLED;
            }       

            len = buf[0] << 8 | buf[1];
            if (len & 0x8000)
            {
                len = len & 0x7fff;
            } else {
                len = 0;
            }

            spin_lock_irqsave(&dev->recv_lock, flags);
            if (len > 0)
            {
                dev->recv_len[i] = len;
                dev->recv_ready |= (1 << i);
                
                if (len > 128)
                {
                    printk("[%u] psam len error: offset = %d, idx = %d, len = %d\n", (unsigned int)jiffies, offset, i, len);
                }   
            }
            spin_unlock_irqrestore(&dev->recv_lock, flags);
        }
    }

out:
    //need lock?    
    spin_lock_irqsave(&dev->recv_lock, flags);
    if ((dev->recv_ready != 0) || (dev->update))
    {
        wake_up_interruptible(&dev->recv_wait);
    }
    spin_unlock_irqrestore(&dev->recv_lock, flags);

    return IRQ_HANDLED;
}


static ssize_t psam_write(struct file *filp, const char __user *buf,
        size_t count, loff_t *f_pos)
{
    struct am389x_gpmc_dev *dev = filp->private_data;
    char *kbuf = dev->devbuf;
    int miss = 0;

    if (count > PSAM_INFO_SIZE)
    {
        return -EFAULT;
    }
    
    miss = copy_from_user(kbuf, buf, count);
    if (miss != 0)
    {
        return -EFAULT;
    }

    return gpmc_write(dev, 0, 0, kbuf, count);
}


static ssize_t psam_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    int ret = 0;
    struct am389x_gpmc_dev *dev = filp->private_data;
    char *kbuf = dev->devbuf;

    if (count > PSAM_INFO_SIZE)
    {
        return -EFAULT;
    }

    ret = gpmc_read(dev, 0, 0, kbuf, count);
    if (ret > 0)
    {
        int miss = copy_to_user(buf, kbuf, ret);
        if (miss == ret) //miss all
        {           
            ret = -EFAULT;
        } else {
            ret = ret - miss;
        }
    }
        
    return ret;
}


static int get_card_exist(struct am389x_gpmc_dev *dev)
{
    int offset = psam_offset(pi.card_exist);
    int card_exist;

    int ret = gpmc_read(dev, 0, offset, (char *)&card_exist, sizeof(card_exist));
    if (ret < 0)
    {
        return -1;
    }

    card_exist = (~swab32(card_exist)) & 0x000fffff; //20bits
    return card_exist;
}


static void psam_recv_clean_idx(struct am389x_gpmc_dev *dev, int idx)
{
        unsigned long flags = 0;
        unsigned short flag = 0;
        int offset = psam_offset(ci[idx].ri);
        
        spin_lock_irqsave(&dev->recv_lock, flags);
        gpmc_write(dev, 0, offset, (char *)&flag, 2);
        dev->recv_ready &= ~(1 << idx);
        spin_unlock_irqrestore(&dev->recv_lock, flags); 
}


static int psam_get_first_ready_data(struct am389x_gpmc_dev *dev, struct psam_cmd_recv_info *ri)
{
    int i = 0;
    int ret = 0;
    int offset = 0;
    unsigned short clean = 0;
    int len = 0;

    int trycnt = PSAM_AREA_MAX;

    if (ri->psam_id >=0 && ri->psam_id<PSAM_AREA_MAX)
    {
        trycnt = 1;
    }

    if (dev->recv_ready == 0)
    {
        return 0; //no data ready.
    }

    for (i=0; i<trycnt; i++)
    {
        if (ri->psam_id >=0 && ri->psam_id<PSAM_AREA_MAX)
        {
            i = ri->psam_id; //get the fixed one
        }

        if (((dev->recv_ready >> i) & 0x1) == 0)
        {
            continue;
        }   

        len = dev->recv_len[i];
        if (len > PSAM_DATA_LEN_MAX || len <= 0)
        {
            printk("[%u][PSAM]recv len error, id = %d, len = %d\n", (unsigned int)jiffies, i, len);

            //clean this psam recv buffer
            psam_recv_clean_idx(dev, i);
            return -1;
        }

        offset = psam_offset(ci[i].ri);
        ret = gpmc_read(dev, 0, offset, (char *)&ri->ri, len);
        if (ret < 0)
        {
            return -2;
        }
        
        
        ret = gpmc_write(dev, 0, offset, (char *)&clean, 2);
        if (ret < 0)
        {
            printk("clean the readbuf error\n");
            return -3;
        }

        ri->psam_id = i;
        ri->ri.len = len;

        return (len + sizeof(ri->psam_id)); //return the length                 
    }   

    return 0;  //no data ready
}


//{0x01, 7, 0x00, 0xa4, 0x00, 0x00, 0x02, 0x3f, 0x00},
static int psam_data_check(u8 *data, int len)
{
    int num = data[0];
    int i=0;
    int idx = 1;
    int slen = 0;

    for (i=0; i<num; i++)
    {
        int clen = data[idx];

        //printk("idx = %d, clen = %d\n", idx, clen);
        
        slen += clen;
        idx += (clen + 1);  
    }   

    //printk("slen = %d, len = %d\n", slen, len);

    if ((slen + num + 1) == len)
    {
        return 0;
    } else {
        return -1;
    }
}

static long psam_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct am389x_gpmc_dev *dev = filp->private_data;
    void __user *argp = (void __user *)arg;
    int ret = 0;

    switch(cmd)
    {
        case GPMC_IOC_CMD_PSAM_UPDATE:
        {
            u8 clr_flag = 0;
            unsigned long flags = 0;
            struct psam_update_rq rq;
            struct psam_update_rs rs;

            int offset_rq = psam_offset(ui.rq);
            int offset_rs = psam_offset(ui.rs);
            int len = 0;
    
            if (copy_from_user(&rq, argp, sizeof(struct psam_update_rq)))
            {
                return -EFAULT;
            }   

            //printk("get update: len = %d, crc = %04x, sn = %u, slen = %d, offset = %d\n", rq.len, rq.crc, rq.sn, rq.slen, offset_rq);

            len = rq.len;
            rq.len = swab16(rq.len);
            rq.crc = swab16(rq.crc);
            rq.sn = swab32(rq.sn);
            rq.slen = swab16(rq.slen);
            rq.file_size = swab16(rq.file_size);

            ret = gpmc_write(dev, 0, offset_rq, (char *)&rq, len);
            
            {
                //fire
                u8 flag = 0x80 | (len >> 8);                            
                gpmc_write(dev, 0, offset_rq, &flag, 1);
            }

            wait_event_interruptible(dev->recv_wait, dev->update);

            spin_lock_irqsave(&dev->recv_lock, flags);
            if (dev->update)
            {
                dev->update = 0;
                spin_unlock_irqrestore(&dev->recv_lock, flags); 
            } else {            
                spin_unlock_irqrestore(&dev->recv_lock, flags); 
                return -1;          
            }

            ret = gpmc_read(dev, 0, offset_rs, (char *)&rs, 8);         
            rs.len = ((rs.len & 0x7f) << 8) | ((rs.len >> 8) & 0xff);
            rs.status = swab16(rs.status);
            rs.sn = swab32(rs.sn);          

            if (copy_to_user(argp, (u8 *)&rs, 8))
            {
                return -EFAULT;
            }       
            
            //clear rx buf          
            gpmc_write(dev, 0, offset_rs, (char *)&clr_flag, 1);

            //printk("get rs: len = %d, status = %04x, sn = %d\n", rs.len, rs.status, rs.sn);
                                    
            return ret;
        }

        case GPMC_IOC_CMD_SEND:
        {
            struct psam_cmd_send_info  psi; 
            int offset = 0;
            int len = PSAM_CMD_HEADER_LEN;

            //printk("cmd send data.\n");

            //copy header first
            if (copy_from_user(&psi, argp, len))
            {
                return -EFAULT;
            }   
                    
            len += psi.si.slen;
            if (len > PSAM_DATA_LEN_MAX)
            {
                printk("cmd send : data length error, len = %d\n", len);
                return -EFAULT;
            }

            //copy data and header again    
            if (copy_from_user(&psi, argp, len))
            {
                return -EFAULT;
            }   
            

            if (psi.psam_id >= PSAM_AREA_MAX)
            {
                printk("psam id error = %d\n", psi.psam_id);
                return -EFAULT;
            }

#if 1
            //clean the recv buffer
            {
            unsigned short clean = 0;
            unsigned long flags = 0;

            spin_lock_irqsave(&dev->recv_lock, flags);

            offset = psam_offset(ci[psi.psam_id].ri);
            gpmc_write(dev, 0, offset, (char *)&clean, 2);

            dev->recv_ready &= ~(1 << psi.psam_id);
            spin_unlock_irqrestore(&dev->recv_lock, flags); 
            }
#endif

            len = psi.si.len;
            psi.si.len = swab16(len); //the psam fpga use the big-endian
            psi.si.timeout = swab32(psi.si.timeout);

            offset = psam_offset(ci[psi.psam_id].si);

            //check write out
#if 0
            {
                u8 write_out = 0;
                gpmc_read(dev, 0, offset, (char *)&write_out, 1);   
                if (write_out & 0x80)
                {
                    printk("[%u][PSAM]WARN: the last data is not write out yet.\n", (unsigned int)jiffies); 
                    return -EFAULT; 
                }
            } 
#endif

#if 1 //check the data format
/*添加了(psi.si.type == 1)条件，只有发送至PSAM的内容才进行合法性检查
*否则PSAM的同步命令不能正常执行 duanshichao
*/
            if ((psi.si.type == 1) && (psi.si.slen > 0) && (psam_data_check(psi.si.data, psi.si.slen) < 0))
            {
                int i=0;

                printk("[%u]psam send data format error, data(len =%d):\n", (unsigned int)jiffies, psi.si.slen);
                for (i=0; i<psi.si.slen; i++)
                {
                    printk("%02x ", psi.si.data[i]);            
                }   
                printk("\n");

                return -4;
            }
#endif

            //gpmc_dump("psam write out: ", (char *)&psi.si, len);
            
            ret = gpmc_write(dev, 0, offset, (char *)&psi.si, len);
            
            {
                //fire
                u8 flag = 0x80 | (len >> 8);                            
                gpmc_write(dev, 0, offset, &flag, 1);
            }

            return ret;
        }

        case GPMC_IOC_CMD_RECV:
        {           
            int len;
            struct psam_cmd_recv_info ri;

            //copy header first
            if (copy_from_user(&ri, argp, 4))
            {
                return -EFAULT;
            }   

            //printk("psam_id = %d\n", ri.psam_id);

            while(1)
            {
                int ret = -1;
                if (ri.psam_id >=0 && ri.psam_id<PSAM_AREA_MAX)
                {
                    //int wait_flag = dev->recv_ready & (1 << ri.psam_id); //NOTE!!! can NOT using arg here for wait_event()
                    ret = wait_event_interruptible(dev->recv_wait, dev->recv_ready & (1 << ri.psam_id));
                } else {
                    ret = wait_event_interruptible(dev->recv_wait, dev->recv_ready);
                }
        
                if (ret < 0)
                {
                    break; //interrupt by signal
                }

                //check
                if (ri.psam_id >=0 && ri.psam_id<PSAM_AREA_MAX)
                {
                        if (dev->recv_ready & (1 << ri.psam_id))
                        {
                            break; //the psam got data
                        }
                } else {
                        break; // id = -1
                }
            }
        
            len = psam_get_first_ready_data(dev, &ri);
            if (len > 0)
            {
                unsigned long flags;

                //copy all the data
                if (copy_to_user(argp, (u8 *)&ri, len))
                {
                    return -EFAULT;
                }           


                //gpmc_dump("psam get data:", (unsigned char *)&ri, len);

                spin_lock_irqsave(&dev->recv_lock, flags);
                dev->recv_ready &= ~(1 << ri.psam_id);
                spin_unlock_irqrestore(&dev->recv_lock, flags); 
            }

            return len;     
        }

        case GPMC_IOC_CMD_RECV_TEST:
        {
            struct psam_cmd_recv_info ri;
            int offset;
            u32 idx = 0;
            
            if (get_user(idx, (__u32 __user *)arg)) 
            //if (copy_from_user(&idx, argp, 4))
            {
                return -EFAULT;
            } 

            if (idx >= 20)
            {
                printk("error: idx = %d\n", idx);
                return -1;
            }

            offset = psam_offset(ci[idx].ri);
            ret = gpmc_read(dev, 0, offset, (char *)&ri.ri, 256);
            if (ret < 0)
            {
                return -EFAULT;
            }

            ri.psam_id = idx;
            ri.ri.len = ret;

            //copy all the data
            if (copy_to_user(argp, (u8 *)&ri, sizeof(struct psam_cmd_recv_info)))
            {
                return -EFAULT;
            }           

            return (ret + sizeof(ri.psam_id)); //return the length          
        }


        case GPMC_IOC_GET_TEMP:
        {
            int offset = psam_offset(pi.temp[0]);
            u32 temp;

            struct  psam_cmd_temp tmp;

            //printk("gpmc get temp.\n");
            ret = gpmc_read(dev, 0, offset, (char *)&temp, sizeof(temp));
            if (ret < 0)
            {
                return -2;
            }

            temp = swab32(temp);

            //printk("temp = %08x\n", temp);

            tmp.temp1 = (temp & 0xffff0000) >> 16;
            tmp.temp2 = (temp & 0xffff);

            tmp.temp1 = tmp.temp1 / 128 * 5;
            tmp.temp2 = tmp.temp2 / 128 * 5;

            //ret = put_user(temp, (__u32 __user *)argp);   
            if (copy_to_user(argp, (u8 *)&tmp, sizeof(struct  psam_cmd_temp)))
            {
                return -EFAULT;
            }

            return sizeof(temp);
        }

        case GPMC_IOC_GET_CARD_EXIST:
        {
            int card_exist = get_card_exist(dev);
            ret = put_user(card_exist, (int __user *)argp); 

            return sizeof(card_exist);
        }

        case GPMC_IOC_GET_REVID:
        {
            int offset = psam_offset(pi.revid);
            u16 revid;

            //printk("gpmc get revid.\n");
            ret = gpmc_read(dev, 0, offset, (char *)&revid, sizeof(revid));
            if (ret < 0)
            {
                return -2;
            }

            revid = swab16(revid);
            ret = put_user(revid, (__u16 __user *)argp);    

            return ret;
        }
        default:
            printk("error cmd %d\n", cmd);
    }

    return 0;
}


static int psam_open(struct inode *inode, struct file *filp)
{
    struct am389x_gpmc_dev *dev = container_of(inode->i_cdev, struct am389x_gpmc_dev, cdev);

    int offset = 0;
    unsigned long flags = 0;
    unsigned short clear = 0;
    int i=0;    

    //mutex open    
    if (!mutex_trylock(&dev->open_lock))
    {
        printk("psam dev already open, return.\n");
        return -1;
    }


    spin_lock_irqsave(&dev->recv_lock, flags);

    filp->private_data = dev;   
    dev->recv_ready = 0;
    dev->update = 0;

    for (i=0; i<PSAM_AREA_MAX; i++)
    {
        offset = psam_offset(ci[i].ri);
        gpmc_write(dev, 0, offset, (char *)&clear, 2);

        offset = psam_offset(ci[i].si);
        gpmc_write(dev, 0, offset, (char *)&clear, 2);
    }

    spin_unlock_irqrestore(&dev->recv_lock, flags); 

    return 0;
}


static int psam_release(struct inode *inode, struct file *filp)
{
    struct am389x_gpmc_dev *dev = container_of(inode->i_cdev, struct am389x_gpmc_dev, cdev);
    
    filp->private_data = dev;
    mutex_unlock(&dev->open_lock);

    return 0;
}


static unsigned int gpmc_poll(struct file *filp, poll_table *wait)
{
    unsigned int mask;
    unsigned long flags;
    struct am389x_gpmc_dev *dev = filp->private_data;
    
    poll_wait(filp, &dev->recv_wait, wait);

    spin_lock_irqsave(&dev->recv_lock, flags);
    //if (dev->recv_ready)
    if (dev->recv_ready || dev->update)
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


static unsigned int psam_poll(struct file *filp, poll_table *wait)
{
    return  gpmc_poll(filp, wait);
}


static struct file_operations psam_fops = {
    .owner =    THIS_MODULE,
    .write =    psam_write,
    .read =     psam_read,
    .unlocked_ioctl = psam_ioctl,
    .open =     psam_open,
    .release =  psam_release,
    .llseek =   no_llseek,
    .poll   = psam_poll,
};



//============ wlm file operation functions ============


int wlm_irq_cnt = 0;
static irqreturn_t wlm_irq_handler(int irq, void *dev_id)
{
    int cs = 0;
    int i = 0, offset = 0, ret = 0;
    unsigned long flags;
    struct am389x_gpmc_dev *dev = dev_id;
    int len = 0;

	/*wlm模块申请了166号中断，这是地感用的，所以在这里加个判定*/
	if(0);
	else if(irq == sw1_irq) 
    { 
		//printk("switch1 interrupt is done irq = %d.\r\n",irq);
		sw1_count++;
		//wake_up_interruptible(&switch_waitq);
		//稍后应该加个定时器防抖
		return IRQ_HANDLED;
    }
	else if(irq == sw2_irq)
	{
		//printk("switch2 interrupt is done irq2 = %d.\r\n",irq);
		sw2_count++;
		//wake_up_interruptible(&switch_waitq);
		//稍后应该加个定时器防抖
		return IRQ_HANDLED;
	}

	//printk("<0>" "wlm irq = %d.\r\n",irq);


    wlm_irq_cnt++;

    //get the cs num
    for (i=0; i<GPMC_CS_NUM_MAX; i++)
    {
        if (irq == dev->irq[i])
        {
            cs = i;
            break;
        }   
    }

    //for (i=0; i<1; i++)
    for (i=0; i<WLM_MODULE_AREA; i++)
    {
        char buf[2] = {0, 0};

        offset = 1024 + i * 2048;
        ret = gpmc_read(dev, cs, offset, buf, 2); 
        if (ret < 0)
        {
            printk("irq read gpmc error\n");
            return IRQ_HANDLED;
        }       

        len = buf[0] << 8 | buf[1];
        if (len & 0x8000)
        {
            len = len & 0x7fff;
        } else {
            len = 0;
        }

        
        spin_lock_irqsave(&dev->recv_lock, flags);
        if (len > 0)
        {
            int idx = i + cs * WLM_MODULE_AREA;
            dev->recv_len[idx] = len;
            dev->recv_ready |= (1 << idx);
        }
        spin_unlock_irqrestore(&dev->recv_lock, flags);
    }

    //need lock?    
    spin_lock_irqsave(&dev->recv_lock, flags);
    if (dev->recv_ready != 0)
    {
        wake_up_interruptible(&dev->recv_wait);
    }
    spin_unlock_irqrestore(&dev->recv_lock, flags);

    return IRQ_HANDLED;

}


static ssize_t wlm_write(struct file *filp, const char __user *buf,
        size_t count, loff_t *f_pos)
{
    printk("wlm write not ready yet ...\n");

    return -1;
}


static ssize_t wlm_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    printk("wlm read not ready yet...\n");  
    return -1;
}


static int wlm_get_first_ready_data(struct am389x_gpmc_dev *dev, struct wlm_cmd_info *ri)
{
    int i = 0;
    int ret = 0;
    int offset = 0;
    int cs = 0;
    int len = 0;


    if (dev->recv_ready == 0)
    {
        return 0; //no data ready.
    }

    for (i=0; i<WLM_AREA_MAX; i++)
    {
        if (((dev->recv_ready >> i) & 0x1) == 0)
        {
            continue;
        }   

        len = dev->recv_len[i];
        if ((len > WLM_DATA_LEN_MAX) || (len <= 0))
        {
            printk("wlm len error, len = %d\n", len);
            return -1;
        }

        cs = i / WLM_MODULE_AREA;
        offset = 1024 + (i % WLM_MODULE_AREA) * 2048;
        ret = gpmc_read(dev, cs, offset, (char *)&ri->len, len);
        if (ret < 0)
        {
            return -2;
        }

#if 0           
        {
        //fire!
        unsigned short flag = 0;    
        gpmc_write(dev, cs, offset, (char *)&flag, 2);
        }
#endif

        ri->idx = i;
        ri->len = len;

        return (len + sizeof(ri->idx)); //return the length                 
    }   

    return 0;
}

static long wlm_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct am389x_gpmc_dev *dev = filp->private_data;
    void __user *argp = (void __user *)arg;
    int ret = 0;
    unsigned long flags;      

    switch(cmd)
    {
        case GPMC_IOC_CMD_WLM_RECV_RAW:
        {
            int len;
            struct wlm_cmd_info ri;

            wait_event_interruptible(dev->recv_wait, dev->recv_ready);
            
            len = wlm_get_first_ready_data(dev, &ri);           
            if (len > 0)
            {
                //copy all the data
                if (copy_to_user(argp, (u8 *)&ri, len))
                {
                    return -EFAULT;
                }           
#if 0
                //clear it by manual, at cmd GPMC_IOC_DATA_CLEAR
                spin_lock_irqsave(&dev->recv_lock, flags);
                dev->recv_ready &= ~(1 << ri.idx);
                spin_unlock_irqrestore(&dev->recv_lock, flags); 
#endif
            }
            
            //printk("wlm rlen = %d\n", len);
            return len;     
        }

        case GPMC_IOC_CMD_WLM_SEND_RAW:
        {
            struct wlm_cmd_info si;
            int offset = 0;
            int cs = 0;
            int len = 4;

            //copy header first
            if (copy_from_user(&si, argp, len))
            {
                return -EFAULT;
            }   
                    
            len += si.len;
            if (len > WLM_DATA_LEN_MAX)
            {
                printk("cmd send: data length error, len = %d\n", len);
                return -EFAULT;
            }

            //copy data and header  
            if (copy_from_user(&si, argp, len))
            {
                return -EFAULT;
            }   
            
            if (si.idx >= 20)
            {
                printk("wlm id error = %d\n", si.idx);
                return -EFAULT;
            }

            si.len += 2; //add itself
            len = si.len;
            si.len = swab16(len);

			//新的模块用一同一个片选 duanshichao
            cs = si.idx / WLM_MODULE_AREA;          
            offset = (si.idx % WLM_MODULE_AREA) * 2048;
            ret = gpmc_write(dev, cs, offset, (char *)&si.len, len);

#if 0
            printk("[%u]gpmc write out cs = %d, offset: %d, len = %d\n", (unsigned int)jiffies, cs, offset, len);
            gpmc_dump("wlm gpmc write out: \n", (char *)&si.len, len);
#endif


            {
            u8 flag = 0x80 | (len >> 8);                            
            gpmc_write(dev, cs, offset, &flag, 1);
            }

            return ret;
        }

        case GPMC_IOC_SEND_PULSE:
        {
            //printk("wlm_irq_cnt = %d\n", wlm_irq_cnt);
            
            gpmc_io_out(GPIO_OUT_INFO_LED1, 1);
            udelay(100);    
            gpmc_io_out(GPIO_OUT_INFO_LED1, 0);

            //printk("time: recv=%u, wk=%u, wkup=%u, read=%u\n", dev->recv_time, dev->wake_time, dev->wakeup_time, dev->user_read_time);

#if 0
            //printk("gpmc send pulse.\n");
            gpmc_io_out(FPGA_WLM_SEND, 0);
            udelay(100);            
            gpmc_io_out(FPGA_WLM_SEND, 1);
#endif
            return 0;
        }


        case GPMC_IOC_WLM_SYNC:
        {
            gpmc_io_out(GPIO_WLM_SYNC, 0);
            udelay(100);    
            gpmc_io_out(GPIO_WLM_SYNC, 1);          

            //printk("get wlm sync\n");
            return 0;
        }


        case GPMC_IOC_ANT_CS:
        {
            u8 cs;
                        
            if (get_user(cs, (__u8 __user *)arg))   
            //if (copy_from_user(&idx, argp, 4))
            {       
                return -EFAULT;
            } 
            
            wlm_io_set_cs(cs);
    
            //printk("get ant cs = %02x\n", cs);
            return 0;
        }


        case GPMC_IOC_DATA_CLEAR:
        {
            int cs, offset, idx, i;
            u16 flag = 0;
            
            if (get_user(idx, (__u32 __user *)arg)) 
            //if (copy_from_user(&idx, argp, 4))
            {
                return -EFAULT;
            } 

            for (i=0; i<WLM_AREA_MAX; i++)
            {
                if (idx & 0x1)
                {
                    cs = i / WLM_MODULE_AREA;
                    offset = 1024 + (i % WLM_MODULE_AREA) * 2048;
                
                    ret = gpmc_write(dev, cs, offset, (char *)&flag, 2);
                    if (ret < 0)
                    {
                        printk("GPMC_IOC_DATA_CLEAR clean the readbuf error\n");
                        return -EFAULT;
                    }
#if 1
                    //clear it here by user
                    spin_lock_irqsave(&dev->recv_lock, flags);
                    dev->recv_ready &= ~(1 << i);
                    spin_unlock_irqrestore(&dev->recv_lock, flags); 
#endif
                }

                idx = idx >> 1;
                if (idx == 0)
                {
                    break;
                }
            }

            return 0;
        }

        case GPMC_IOC_DATA_DUMP:
        {
            struct cmd_data_dump dr;
            int cs = 0;
            int offset = 0;

            //copy header first
            if (copy_from_user(&dr, argp, 4))
            {
                return -EFAULT;
            }   

            cs = dr.idx / WLM_MODULE_AREA;
            offset = (dr.rw == 1) ? 1024 : 0;           
            offset += (dr.idx % WLM_MODULE_AREA) * 2048;

            ret = gpmc_read(dev, cs, offset, (char *)&dr.data, 1024);
            if (ret < 0)
            {
                return -2;
            }           

            //copy all the data
            if (copy_to_user(argp, (u8 *)&dr, sizeof(struct cmd_data_dump)))
            {
                return -EFAULT;
            }   

            return sizeof(struct cmd_data_dump);
        }

        case GPMC_IOC_BUF_WRITE:
        {
            struct cmd_data_dump dr;
            int cs = 0;
            int offset = 0;

            //copy header first
            if (copy_from_user(&dr, argp, sizeof(struct cmd_data_dump)))
            {
                return -EFAULT;
            }   

            cs = dr.idx / WLM_MODULE_AREA;
            offset = (dr.rw == 1) ? 1024 : 0;   //1 for read buf        
            offset += (dr.idx % WLM_MODULE_AREA) * 2048;        

            //printk("buf write cs = %d, offset = %d\n", cs, offset);
    
            ret = gpmc_write(dev, cs, offset, (char *)&dr.data, 1024);
            
            return ret;         
        }
        
        case SYNC_IOC_EDGE_TRIGGER:       
			if(arg == 1)
			{
				/* 同步信号设置上升沿触发*/
	            printk("sync edge set to rising.\r\n");
	            set_irq_type(sync_irq, IRQ_TYPE_EDGE_RISING);  
			}
			else if(arg == 2)
			{
				/* 同步信号设置下降沿触发*/
				printk("sync edge set to falling.\r\n");
            	set_irq_type(sync_irq, IRQ_TYPE_EDGE_FALLING); 
			}
			else if(arg == 3)
			{
				/* 双边沿触发 */
				printk("sync edge set to rising && falling.\r\n");
            	set_irq_type(sync_irq, IRQ_TYPE_EDGE_BOTH); 
			}
            sync_signal = 0;
            break;
        /* 等待同步信号 */
        case SYNC_IOC_WAIT_SIGNAL:
            sync_signal = 0;
            wait_event_interruptible(sync_waitq,sync_signal);   //这里要判断一下是不是其他信号中断
            break;  
        /* 等待同步信号或者超时 arg为超时时间，单位为ms */
        case SYNC_IOC_WAIT_TIMEOUT:             
            sync_signal = 0;  
            arg = msecs_to_jiffies(arg);		//转换成等待jiffies
            //printk("wait %ld jiffies\r\n",arg);
			wait_event_interruptible_timeout(sync_waitq,sync_signal,arg);//等待超时
			if(!sync_signal)
			{
				return -1;
			}
            break;  

		/* 地感处理， 先放这里，稍后抽出 */
		case SWITCH_IOC_GET_COUNT:
			if(arg == 0)
			{
				return sw1_count;
			}
			if(arg == 1);
			{
				return sw2_count;
			}
			break;

		case SWITCH_IOC_CLR_COUNT:
			if(arg == 0)
			{
				sw1_count = 0;
			}
			if(arg == 1);
			{
				sw2_count = 0;
			}
			break;
		case SWITCH_IOC_SIGNAL_DETECT:			
			if(arg == 0)
			{
				arg = gpio_get_value(SWITCH1_PIN);
				//printk("switch1 pin is %d\r\n",arg);
				return arg;
			}
			else if(arg == 1)
			{
				arg = gpio_get_value(SWITCH2_PIN);
				//printk("switch2 pin is %d\r\n",arg);
				return arg;
			}

			return -EFAULT;
			break;
			
        default:           
            printk("error cmd %d\n", cmd);
            return -EFAULT;
    }

    return 0;
}


static int wlm_open(struct inode *inode, struct file *filp)
{
    struct am389x_gpmc_dev *dev = container_of(inode->i_cdev, struct am389x_gpmc_dev, cdev);    

    //mutex open    
    if (!mutex_trylock(&dev->open_lock))
    {
        printk("wlm device already open, return.\n");
        return -1;
    }

    filp->private_data = dev;
    dev->recv_ready = 0;
    dev->update = 0;

    return 0;
}


static int wlm_release(struct inode *inode, struct file *filp)
{
    struct am389x_gpmc_dev *dev = container_of(inode->i_cdev, struct am389x_gpmc_dev, cdev);

    mutex_unlock(&dev->open_lock);
    return 0;
}


static unsigned int wlm_poll(struct file *filp, poll_table *wait)
{
    return  gpmc_poll(filp, wait);
}


static struct file_operations wlm_fops = {
    .owner =    THIS_MODULE,
    .write =    wlm_write,
    .read =     wlm_read,
    .unlocked_ioctl = wlm_ioctl,
    .open =     wlm_open,
    .release =  wlm_release,
    .llseek =   no_llseek,
    .poll   = wlm_poll,
};



static void cdev_setup(struct am389x_gpmc_dev *pdata, int major, int minor, struct file_operations *fops)
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
    
    dev = device_create(gpmc_class, &pdata->pdev->dev, devno, pdata, "%s", pdata->name);
    if (!IS_ERR(dev))
    {
        pdata->devno = devno;       
    }
}


static void cdev_release(struct am389x_gpmc_dev *dev)
{
    cdev_del(&dev->cdev);

    if (dev->devno)
    {
        device_destroy(gpmc_class, dev->devno);
    }
}


static int g90_sync_init(void)
{
    //添加了同步检测中断功能，用于检测同步. Duanshichao
    //setup irq    
    int ret;
	printk("g90_sync_init\r\n");
    if ((gpio_request(IRQ_SYNC_SIGNAL, "SYNC_IRQ") == 0) &&
        (gpio_direction_input(IRQ_SYNC_SIGNAL) == 0)) {
        gpio_export(IRQ_SYNC_SIGNAL, 0);
    } else {
        printk(KERN_ERR "could not obtain gpio %d, for GPMC IRQ\n", IRQ_SYNC_SIGNAL);
        return -1;
    }   

    ret = request_irq(sync_irq, sync_irq_handler, IRQ_TYPE_EDGE_RISING, NULL, NULL);    
    return ret;
}
static void g90_sync_exit(void)
{
	//释放同步检测资源 duan
	printk("release sync irq = %d.\r\n",sync_irq);
	free_irq(sync_irq,NULL);
    gpio_free(IRQ_SYNC_SIGNAL);    
}



//static int sw1_count   = 0;
//static int sw2_count   = 0;
//static DECLARE_WAIT_QUEUE_HEAD(switch_waitq);不需要阻塞

#if 0
static irqreturn_t switch_irq_handler(int irq, void * dev_id)
{ 
	if(0);
	#if 0
    else if(irq == sw1_irq) 
    { 
		printk("switch interrupt is done irq = %d.\r\n",irq);
		sw1_count++;
		//wake_up_interruptible(&switch_waitq);
    }
	#endif
	else if(irq == sw2_irq)
	{
		//printk("switch interrupt is done irq2 = %d.\r\n",irq);
		sw2_count++;
		//wake_up_interruptible(&switch_waitq);
	}

	return IRQ_HANDLED;
}
#endif
static int g90_switch_init(void)
{
    int ret;

	printk("irq request: irq1 = %d.irq2 = %d\r\n",sw1_irq,sw2_irq);
	
	//地感1
    if ((gpio_request(IRQ_SW1_SIGNAL, "SWITCH1_IRQ") == 0) &&
        (gpio_direction_input(IRQ_SW1_SIGNAL) == 0)) 
    {
        gpio_export(IRQ_SW1_SIGNAL, 0);
    } 
	else 
	{
        printk(KERN_ERR "could not obtain gpio %d, for GPMC IRQ\n", IRQ_SW1_SIGNAL);
        //return -1;
    }   

	if ((gpio_request(SWITCH1_PIN, "SWITCH1_PIN") == 0) &&
        (gpio_direction_input(SWITCH1_PIN) == 0)) 
    {
        gpio_export(SWITCH1_PIN, 0);
    } 
	else 
	{
        printk(KERN_ERR "could not obtain gpio %d, for GPMC IRQ\n", SWITCH1_PIN);
        //return -1;
    }   

	
	//地感2
	#if 1
	if ((gpio_request(IRQ_SW2_SIGNAL, "SWITCH2_IRQ") == 0) &&
        (gpio_direction_input(IRQ_SW2_SIGNAL) == 0)) 
    {
        gpio_export(IRQ_SW2_SIGNAL, 0);
		
    } 
	else 
	{
        printk(KERN_ERR "could not obtain gpio %d, for GPMC IRQ\n", IRQ_SW2_SIGNAL);
        //return -1;
    }  
	ret = request_irq(sw2_irq, wlm_irq_handler, IRQ_TYPE_EDGE_FALLING, NULL, NULL);

	printk("request %d irq %s.\r\n",sw2_irq,ret?"failed":"successful");

	if ((gpio_request(SWITCH2_PIN, "SWITCH2_PIN") == 0) &&
        (gpio_direction_input(SWITCH2_PIN) == 0)) 
    {
        gpio_export(SWITCH2_PIN, 0);
    } 
	else 
	{
        printk(KERN_ERR "could not obtain gpio %d, for GPMC IRQ\n", SWITCH1_PIN);
        //return -1;
    }   
	#endif
    return ret;
}

static void g90_switch_exit(void)
{
	printk("release irq: %d,%d\r\n",sw1_irq,sw2_irq);
	//free_irq(sw1_irq,NULL);		//因为是在wlm里注册，所以也不用在这里释放
	gpio_free(IRQ_SW1_SIGNAL);
	
	free_irq(sw2_irq,NULL);	
	gpio_free(IRQ_SW2_SIGNAL);	

	gpio_free(SWITCH1_PIN);	
	gpio_free(SWITCH2_PIN);	
}


static int __devinit g90_fpga_drv_probe(struct platform_device *pdev)
{
    int i = 0;
    int retval = -1;
    struct am389x_gpmc_dev  *pdata;

    int res_size, irq_flags;
    struct resource *res, *irq_res;

    printk("g90_fpga_drv_probe...\n");

    pdata = kzalloc(sizeof(struct am389x_gpmc_dev), GFP_KERNEL);    
    dev_set_drvdata(&pdev->dev, pdata);

    pdata->id = pdev->id;
    pdata->pdev = pdev;


    if (pdata->id == PSAM_DEV_ID)
    {
        pdata->cs_count = 1;
    } else {
        pdata->cs_count = 2;
    }
    
    for (i=0; i<pdata->cs_count; i++)
    {
        res = platform_get_resource(pdev, IORESOURCE_MEM, i);
        if (!res) {
            printk("platform_get_resource idx = %d\n", i);
            return -1;
        }
        res_size = resource_size(res);

        pdata->ioaddr[i] = ioremap_nocache(res->start, res_size);
        if (pdata->ioaddr[i] == NULL) {
            printk("ioremap error\n");
            return -2;
        }

        irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, i);
        if (!irq_res) {
            printk("FPGA Could not allocate irq resource.\n");
            return -1;
        }
        pdata->irq[i] = irq_res->start;
        irq_flags = irq_res->flags & IRQF_TRIGGER_MASK;

        if (pdata->id == PSAM_DEV_ID)
        {
            printk("check the psam info struct size...");
            if (sizeof(struct psam_info) != PSAM_INFO_SIZE)
            {
                printk("ERROR. size=%d, it sholud be %d.\n", sizeof(struct psam_info), PSAM_INFO_SIZE);
            } else {
                printk("OK. size = %d\n", PSAM_INFO_SIZE);
            }
        
            sprintf(pdata->cs_mod_name[i], "psam");
            retval = request_irq(pdata->irq[i], psam_irq_handler, irq_flags, pdata->cs_mod_name[i], pdata);
        } else {
            sprintf(pdata->cs_mod_name[i], "wlm_%d", i);
            retval = request_irq(pdata->irq[i], wlm_irq_handler, irq_flags, pdata->cs_mod_name[i], pdata);			
        }
        if (retval) 
        {
            printk( "Unable to claim requested irq: %d\n", pdata->irq[i]);
        }

        printk("id = %d, start = %08x, size = %d, ioaddr = %08x, irq = %d\n", pdev->id, res->start, res_size, (unsigned int)pdata->ioaddr[i], pdata->irq[i]);
    }


#if 1
    printk("config to 8bit bus\n");
    gpmc_cs_configure(1, GPMC_CONFIG_DEV_SIZE, 0);  //set to 8bits bus
    pdata->busw = 0;
#endif
    
    spin_lock_init(&pdata->recv_lock);
    init_waitqueue_head(&pdata->recv_wait);
    pdata->recv_wait_done = 0;
    pdata->recv_ready = 0;

    mutex_init(&pdata->open_lock);


    if (pdata->id == PSAM_DEV_ID)
    {
        pdata->name = PSAM_DEV_NAME;
        cdev_setup(pdata, WLM_MAJOR, pdata->id, &psam_fops);
    } else {
        pdata->name = WLM_DEV_NAME;
        cdev_setup(pdata, WLM_MAJOR, pdata->id, &wlm_fops); 
    }

    pdata->devbuf = (void *)__get_free_pages(GFP_ATOMIC, get_order(ALLOC_BUF_LEN));
    if (pdata->devbuf == NULL)
    {
        printk("alloc pages error\n");
    }

    //struct psam_info pi;
    //printk("offset = %d(%d)\n", ((u8 *)&pi.ui.rq_type - (u8 *)&pi), psam_offset(ci[1].ri.sr2));

    if (sizeof(struct psam_info) != 23 * 1024)
    {
        printk("ERROR: sizeof psam_info = %d(%d)\n", sizeof(struct psam_info), 23 * 1024);
    }

    gpmc_io_out(FPGA_CS1_POWER, POWER_ON);

    sync_io_gpio_setup();

    return 0;
}



static int g90_fpga_drv_remove(struct platform_device *pdev)
{

    struct am389x_gpmc_dev  *pdata = dev_get_drvdata(&pdev->dev);
    int i=0;

    printk("g90 fpga remove, id = %d, addr = %08x\n", pdata->id, (unsigned int)pdata->ioaddr);

    for (i=0; i<pdata->cs_count; i++)
    {
        free_irq(pdata->irq[i], pdata);
        iounmap(pdata->ioaddr[i]);
    }

    if (pdata->devbuf != NULL)
    {
        free_pages((unsigned long)pdata->devbuf, get_order(ALLOC_BUF_LEN));
    }

    kfree(pdata);
    cdev_release(pdata);    

    gpmc_io_out(FPGA_CS1_POWER, POWER_OFF);

    
    return 0;
}

static struct platform_driver g90_fpga_driver = {
    .probe = g90_fpga_drv_probe,
    .remove = __devexit_p(g90_fpga_drv_remove),
    .driver = {
        .name   = G90_FPGA,
        .owner  = THIS_MODULE,
    },
};


static int __init g90_gpmc_init(void)
{
    printk("G90 gpmc init, driver version = %s.\n", G_VER);

	g90_sync_init();    //添加了同步中断输入 duanshichao
	g90_switch_init();	//添加了地感中断输入		

    gpmc_class = class_create(THIS_MODULE, "gpmc_class");
    return platform_driver_register(&g90_fpga_driver);
}

static void __exit g90_gpmc_exit(void)
{
    printk("G90 gpmc deinit.\n");	
    platform_driver_unregister(&g90_fpga_driver);
    class_destroy(gpmc_class);    

	g90_sync_exit();
	g90_switch_exit();
}

module_init(g90_gpmc_init);
module_exit(g90_gpmc_exit);

MODULE_LICENSE("GPL"); 
MODULE_AUTHOR("R.wen");
   
