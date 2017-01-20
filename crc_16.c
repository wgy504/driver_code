#include <stdio.h>

typedef unsigned char u8;
typedef signed char s8; //注意char在arm gcc下面是无符号数,rwen
typedef unsigned short u16;
typedef signed short s16;
typedef unsigned int u32;
typedef signed int s32;
typedef unsigned long long u64;
typedef signed long long s64;

#define STX_ETX     0xFF
#define DLE_CHAR    0xFE
#define STX_EXIST   0
#define ETX_EXIST   1
#define DLE_EXIST   2
#define swab16(x) ((u16)( \
(((u16)(x) & (u16)0x00FFU) << 8) | \
(((u16)(x) & (u16)0xFF00U) >> 8)))




#define RSU_CMD_AREA_SELECT 0xCB
unsigned short check_crc16(unsigned char *crc_data, int len)
{
    int i;
    int k;
    unsigned long crc = 0L;
    for(i = 0; i < len; i++) //loop
    {
        crc ^= crc_data[i];
        for(k = 0; k < 8; k++) //每一位为1时跟0x8408进行异或
        {
            if(crc & 0x1)
            {
                crc = (crc >> 1) ^ 0x8408;
            }
            else
            {
                crc = (crc >> 1);
            }
        }
    }

    crc = ~crc & 0xFFFF;
    return (unsigned short)crc;
}


unsigned short crc16(char *bytes, int len)
{
    int i, j, crc = 0;
    //16 字节数据进行 crc 校验
    for (i = 0; i < len; i++)
    {
        crc = crc ^ bytes[i] << 8;
        for (j = 0; j < 8; j++)
        {
            if((crc & ((int) 0x8000)) != 0)
            {
                crc = crc << 1 ^ 0x1021;
            }
            else
            {
                crc = crc << 1;
            }
        }
    }
    return (unsigned short)(crc & 0xFFFF) ;
} 

struct area_select_cmd_t
{
	u8 sn;
	u8 cmd;
	u8 sw;
}__attribute__ ((packed));

struct wlm_cmd_data_t
{
	u8 head_h;
	u8 head_l;
	struct area_select_cmd_t cmd_str;
	u8 crc;
	u8 tail;
		

}__attribute__ ((packed));

struct wlm_send_cmd_t
{
	u8 head_h;
	u8 head_l;
	u8 sn;
	short len;//从ctl_1到cmd_data
	u8 ctl_1;
	u8 ctl_2;
	u8 ctl_3;
	u8 baud;
	u8 modulation;
	u8 square;
	u8 data_type;
	u8 timeout_high;
	u8 timeout_mid;
	u8 timeout_low;
	u8 len_high;//
	u8 len_low;//cmd_data的长度
	struct wlm_cmd_data_t cmd_data;
	short crc_16;
	u8 tail_h;
	u8 tail_l;
}__attribute__ ((packed));



int convert_to_itsc_format(unsigned char *dst, unsigned char* src, int len)
{
    int i;
    int sp;
    unsigned char bcc;

    sp = 0;
    bcc = 0x00;

    dst[sp++] = 0xFF; //rwen
    dst[sp++] = 0xFF;

    if(len > 1020) //原始数据长度超长
    {
        printf("原始数据长度超长\n");
        return -1;
    }
    for(i = 0; i < len; i++) //转义处理
    {
        bcc = bcc ^ src[i];
        if((src[i] == DLE_CHAR) || (src[i] == 0xFF))
        {
            dst[sp++] = DLE_CHAR;
            dst[sp++] = src[i] ^ DLE_CHAR;
        }
        else
        {
            dst[sp++] = src[i];
        }
    }
    if((bcc == 0xFF) || (bcc == DLE_CHAR)) //bcc
    {
        dst[sp++] = DLE_CHAR;
        dst[sp++] = bcc ^ DLE_CHAR;
    }
    else
    {
        dst[sp++] = bcc;
    }
    dst[sp++] = 0xFF;

    if(sp > 1024) //数据转义后长度超长
    {
        printf("数据转义后长度超长##\n");
        return -2;
    }
    return sp;
}

void main()
{
	//unsigned char crc_buf[] = {0x06, 0x80, 0x18, 0x94, 0x00, 0x02, 0x04, 0x1a, 0x10, 0x55, 0x80, 0x00, 0x00, 0x00, 0x0a, 0xff, 0xff, 0x88, 0xc1, 0x0a, 0x01, 0x04, 0x00, 0x46, 0xff};
	
	
	//unsigned short crc = crc16(crc_buf, sizeof(crc_buf));
	
	int len = 0;
	
	struct area_select_cmd_t ant_cmd = {0};
	struct wlm_cmd_data_t cmd_data = {0};
	struct wlm_send_cmd_t wlm_cmd = {0};
	

	ant_cmd.sn = 0x68;
	ant_cmd.cmd = RSU_CMD_AREA_SELECT;
	ant_cmd.sw = 0x01;

	wlm_cmd.head_h = 0x55;
	wlm_cmd.head_l = 0xAA;
	wlm_cmd.tail_h = 0xAA;
	wlm_cmd.tail_l = 0x55;
	wlm_cmd.sn = 0x56;

	wlm_cmd.len = swab16(0x8015);
	wlm_cmd.ctl_1 =0x94;
	wlm_cmd.ctl_2 = 0x00;
	wlm_cmd.ctl_3 = 0x02;
	wlm_cmd.baud = 0x04;
	wlm_cmd.modulation = 0x1A;
	wlm_cmd.square = 0x10;
	wlm_cmd.data_type = 0x55;
	wlm_cmd.timeout_high = 0x80;
	wlm_cmd.timeout_mid = 0x00;
	wlm_cmd.timeout_low = 0x00;
	wlm_cmd.len_high = 0x00;
	wlm_cmd.len_low = 0x07;


	len = convert_to_itsc_format(&cmd_data, &ant_cmd, sizeof(ant_cmd));
	if(len < 0)
	{
		return len;
	}
	//glog_data("ant_cmd", &ant_cmd, sizeof(ant_cmd));
	//glog_data("cmd_data", &cmd_data, len);

	memcpy(&wlm_cmd.cmd_data, &cmd_data, sizeof(cmd_data));
	wlm_cmd.crc_16 = crc16(&wlm_cmd.sn, (int)&wlm_cmd.crc_16 - (int)&wlm_cmd.sn);
	//glog_data("crc_16", &wlm_cmd.crc_16, 2);
	
	

	
	
	
	
	printf("crc = 0x%X\n", wlm_cmd.crc_16);
	
	
	
}