#ifndef _SHT3X_H
#define _SHT3X_H

#include "gd32e230.h"
//#include "stm32l0xx.h"

#define SHT3X_ADDR                                              0x88            //Ĭ�ϵ�ַΪ0x44��ǰ7��bitsΪ��ַ�����һ��bitΪ��/д������Ҫ����7λ 

#define SPERIODIC_MODE_FETCH_DATA_CMD                           0xE000          //����ģʽ�»�ȡ����ָ�� 
#define SPERIODIC_MODE_ART_CMD                                  0x2B32          //����ģʽ������Ӧʱ��ָ��        
#define SPERIODIC_MODE_BREAK_CMD                                0x3093          //ֹͣ/�ж�����ģʽָ��
#define SOFTWARE_RESET_CMD                                      0x30A2          //оƬ�������ָ��,��ָ������
#define HARDWARE_RESET_CMD                                      0x0006          //Ӳ������������nReset���Ž�������(�رյ�Դ����һ����ʽ��Ӳ������)       
#define HEATER_ENABLE_CMD                                       0x306D          //�ڲ�������ʹ��
#define HEATER_DISABLE_CMD                                      0x3066          //�ڲ�������ʧ��
#define READ_REG_STATUS_CMD                                     0xF32D          //���Ĵ���״̬
#define CLEAR_REG_STATUS_CMD                                    0x3041          //����Ĵ���״̬

//ģʽѡ��ָ������ֽڣ�����ͻ��ģʽ������һ��ָ��SHT3xת��һ�Σ���Ҫת��ʱ��
#define SINGLE_MODE_CLOCK_ENABLE_HIGHT                          0x2C06          //ʱ�Ӹߵȼ�������ʹ�ܵ��ֽ�
#define SINGLE_MODE_CLOCK_ENABLE_MEDIUM                         0x2C0D          //ʱ���еȼ�������ʹ�ܵ��ֽ�
#define SINGLE_MODE_CLOCK_ENABLE_LOW                            0x2C10          //ʱ�ӵ͵ȼ�������ʹ�ܵ��ֽ�
 
#define SINGLE_MODE_CLOCK_DISABLE_HIGHT                         0x2400          //ʱ�Ӹߵȼ�������ʧ�ܵ��ֽ�
#define SINGLE_MODE_CLOCK_DISABLE_MEDIUM                        0x240B          //ʱ���еȼ�������ʧ�ܵ��ֽ�
#define SINGLE_MODE_CLOCK_DISABLE_LOW                           0x2416          //ʱ�ӵ͵ȼ�������ʧ�ܵ��ֽ�


//ģʽѡ��ָ������ֽڣ�����ת��ģʽ������һ��ָ��SHT3x��ͣת����ת����ȴ���ȡ
#define SPERIODIC_MODE_SET_0_5_HIGHT                            0x2032          //����Ϊ0.5mps�ߵȼ�����ֽ�
#define SPERIODIC_MODE_SET_0_5_MEDIUM                           0x2024          //����Ϊ0.5mps�еȼ�����ֽ�
#define SPERIODIC_MODE_SET_0_5_LOW                              0x202F          //����Ϊ0.5mps�͵ȼ�����ֽ�

#define SPERIODIC_MODE_SET_1_0_HIGHT                            0x2130          //����Ϊ1mps�ߵȼ�����ֽ�
#define SPERIODIC_MODE_SET_1_0_MEDIUM                           0x2126          //����Ϊ1mps�еȼ�����ֽ�
#define SPERIODIC_MODE_SET_1_0_LOW                              0x212D          //����Ϊ1mps�͵ȼ�����ֽ�

#define SPERIODIC_MODE_SET_2_0_HIGHT                            0x2236          //����Ϊ2mps�ߵȼ�����ֽ�
#define SPERIODIC_MODE_SET_2_0_MEDIUM                           0x2220          //����Ϊ2mps�еȼ�����ֽ�
#define SPERIODIC_MODE_SET_2_0_LOW                              0x222B          //����Ϊ2mps�͵ȼ�����ֽ�

#define SPERIODIC_MODE_SET_4_0_HIGHT                            0x2334          //����Ϊ4mps�ߵȼ�����ֽ�
#define SPERIODIC_MODE_SET_4_0_MEDIUM                           0x2322          //����Ϊ4mps�еȼ�����ֽ�
#define SPERIODIC_MODE_SET_4_0_LOW                              0x2329          //����Ϊ4mps�͵ȼ�����ֽ�

#define SPERIODIC_MODE_SET_10_0_HIGHT                           0x2737          //����Ϊ10mps�ߵȼ�����ֽ�
#define SPERIODIC_MODE_SET_10_0_MEDIUM                          0x2721          //����Ϊ10mps�еȼ�����ֽ�
#define SPERIODIC_MODE_SET_10_0_LOW                             0x272A          //����Ϊ10mps�͵ȼ�����ֽ�

     

#define SENSOR_NUM      1
#define FILTER_NUM      10
#define TEM_CH_NUM      SENSOR_NUM                                              //�¶�ͨ����
#define HUM_CH_NUM      SENSOR_NUM                                              //ʪ��ͨ����
#define TEM_FIL_NUM     FILTER_NUM                                              //�¶��˲���        
#define HUM_FIL_NUM     FILTER_NUM                                              //ʪ���˲���
#define DEL_HEAD_NUM    3                                                       //ȥ��ͷ�����ݸ���
#define DEL_TAIL_NUM    3                                                       //ȥ��β�����ݸ���


typedef enum {NExist = 0x00, Exist}SensorSta_Typedef;                          //������״̬


typedef struct
{
    uint16_t Temp[SENSOR_NUM];                                                  //�¶ȣ�K�ϣ�
    uint16_t Humi[SENSOR_NUM];                                                  //ʪ�ȣ��ٷֱȣ�
    uint16_t UpThreshold[2*SENSOR_NUM];                                         //��ʪ���Ϸ�ֵ(0~3�¶ȣ�4~7ʪ��)
    uint16_t DoThreshold[2*SENSOR_NUM];                                         //��ʪ���·�ֵ
    uint16_t DurThreshold[2*SENSOR_NUM];                                            //��ʪ�ȳ���ֵ����ʱ��
    uint32_t Duration[2*SENSOR_NUM];
    
	   
    uint8_t  Upload_persist[2*SENSOR_NUM];                                      //??????
    uint8_t  dev_num;
    uint32_t AlarmSta[2*SENSOR_NUM];                                            //����״̬ 
}SHT3_Typedef;  


void SHT3_Init(void);
void ReadTH(void);
void Get_Duration(void);
void APP_ReadTH(void);
SensorSta_Typedef Check_Status(uint8_t SlaveAddr);

uint8_t SHT3x_WriteData(uint8_t DriverAddr, uint16_t Command);
uint8_t SHT3x_ReadData(uint8_t DriverAddr, uint16_t Command, uint8_t* DataBuff, uint8_t DataLen);
uint8_t Set_Sht3_Periodi(uint8_t sChannal, uint16_t sPeriodi);
uint8_t Read_Sht3_TH(uint8_t sChannal, uint16_t sReadCmd, uint8_t *sReadBuf, uint16_t sReadLen);

#endif