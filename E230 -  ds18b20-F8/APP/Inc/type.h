#ifndef _TYPE_H
#define _TYPE_H
#include "gd32e230.h"

#include "gd32e230c_eval.h"
#include "flash.h"
#include "modbus_asc.h"
#include "para.h"
#include "systick.h"
#include "modbus_ascii.h"

#include "string.h"


void Led_Control(uint8_t color);  // 1 ��ɫ  0��ɫ


typedef enum
{ 
  Bit_RESET = 0,
  Bit_SET = 1
}BitAction;


typedef enum
{
    STA_STOP = 1,   //ֹͣ״̬
    STA_WORK = 2,       //����
}RotateStaTypeDef;   //��ת״̬


typedef enum
{
    Stall    = 0,      //ֹͣ״̬
    Foreward = 1,       //��ת״̬
    Reversal = 2,       //��ת״̬
}DirectionState_TypeDef;                 //ת������


typedef enum
{
    STA_NORMAL = 0,
    STA_TIMEOUT,
}StandbyTypeDef;

typedef struct 
{
    int Temp;           //�¶�ֵ*10000 
    uint8_t SlaveAddr;
    uint8_t Baudrate;		                                                 //������
    uint8_t Parity;             			                         //��żУ��λ
    //uint8_t SenSta;     //������״̬  �ò���      
    uint16_t Up_Thr;    //�¶��Ϸ�ֵ  
    uint16_t Do_Thr;    //�¶��·�ֵ
    uint16_t Du_Thr;    //����ʱ�䷧ֵ������������ʱ��Ž���״̬�л���   
    uint32_t Duration;  //���������¶���ֵ����ʱ��
    uint32_t AlarmSta;  //����״̬
	uint8_t  Upload_persist;
}UserTypeDef;


/*
typedef struct 
{
    uint8_t SlaveAddr;                                                          //�ӻ���ַ
    uint8_t Baudrate;		                                                 //������
    uint8_t Parity;             			                         //��żУ��λ

    uint8_t Product_type;                                                    //������ͣ�00-��ѹ�� Ĭ��  01�����ͣ�    
    uint8_t Vout_type;                                                        //��ѹ�������� �� 00:0.5-4.5VĬ��    01:1-5V     02:0-10V    ��
    uint8_t HeightUnit;                                                      //Һλ�߶ȵ�λ
   
    uint16_t SensorRange;                                                      //����������
    uint16_t LiquidAdMin;                                                       //Һλ�궨   ��ֵ�������ѹΪ0.5Vʱ��ADֵ��
    uint16_t LiquidAdMax;                                                       //Һλ�궨  ��ֵ�������ѹΪ4.5Vʱ��ADֵ�� 
    uint8_t FilterLevel;                                                      //�˲��ȼ�
    uint8_t No_defined2;                                                         //δ����2
     
    uint16_t AdAvgVol;
    
    uint16_t AD_Value_Filter;
  
    uint16_t LiquidAdVal;                                               //ҺλADֵ
    uint16_t LiquidHeight;                                              //Һλ�߶� 
    
    uint8_t FilterBufMax;                                                //�˲��������ֵ
    uint16_t FilterCycle;                                                 //�˲�����
    
    uint32_t HFil[10];                                                                 //�߽��˲�����
    uint32_t HFilBak[10];                                                              //�߽��˲����鱸��
    uint32_t LFil[96];                                                                 //�ͽ��˲�����
    uint32_t LFilBak[96];                                                              //�ͽ��˲����鱸��     
   
}UserTypeDef;
*/

/*
typedef struct 
{
    uint16_t CurCnt;                                                            //��ǰ����ֵ
    uint16_t PreCnt;                                                            //�ϴεļ���ֵ
    uint16_t OneSecCnt;                                                         //ÿ��ļ���ֵ
}TIM2_ParaDef;
*/


#endif