#include "para.h"
#include "type.h"

#include "SHT3x.h"

UserTypeDef UserPara;
uint8_t Cur_Param[USER_DEFAULT_LEN];

static uint8_t User_Default_Param[USER_DEFAULT_LEN] =
{
	0x00,
    0x03,
    0x22,
	0x03,
    0xF7,0x0E,          //���ֽ���ǰ��4031�൱��110�棬�¶ȱ�������ֵ
    0x7F,0x09,          //���ֽ���ǰ��2631�൱��-30�棬�¶ȱ�������ֵ        
    0xB7,0x08,          //���ֽ���ǰ��2231s������ʱ����ֵ    
    0x00               	//�Զ��ϴ�ʱ��
    
};

void ReadPara(void)
{
    uint8_t ParaInitFlag;
    flash_read_multi(RUN_ADDR_BASE+1, &ParaInitFlag, 1);
   if(ParaInitFlag != User_Default_Param[1])
    {
        Flash_Write_MultiBytes(RUN_ADDR_BASE, User_Default_Param, USER_DEFAULT_LEN);
    }
    flash_read_multi(RUN_ADDR_BASE, Cur_Param, USER_DEFAULT_LEN);
    UserPara.SlaveAddr = Cur_Param[2];
	UserPara.Baudrate  = Cur_Param[3];
    
    UserPara.Up_Thr = ((uint16_t)Cur_Param[5] << 8) +Cur_Param[4];
    UserPara.Do_Thr = ((uint16_t)Cur_Param[7] << 8) +Cur_Param[6];
    UserPara.Du_Thr = ((uint16_t)Cur_Param[9] << 8) +Cur_Param[8];
    
    UserPara.Upload_persist = Cur_Param[10];
    
    
}





