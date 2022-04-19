#ifndef _PARA_H
#define _PARA_H

#include "type.h"

#define ds18b20							1

#define RUN_ADDR_BASE				                                  0x00								//????????,????????EEPROMP
#define SLAVE_ADDR										(RUN_ADDR_BASE		+ 0x02)
#define Baudrate_ADDR									(SLAVE_ADDR		+ 0x01)
#define TEM_UP_THR                                      (Baudrate_ADDR             + 0x01) //??1???
#define TEM_DO_THR                                      (TEM_UP_THR             + 0x02)

#define TEM_DU_THR                                      (TEM_DO_THR             + 0x02)
#define TEM_UPLOAD_PERSIST                             	(TEM_DU_THR            + 0x02) //????????

#define USER_DEFAULT_LEN				(TEM_UPLOAD_PERSIST             + 0x02)

//---------------------------------------------------
#define COMPANY						(USER_DEFAULT_LEN       + 0x40) 
#define DEV_ENCODING				        (COMPANY		+ 0x40)
#define HWVERSION					(DEV_ENCODING		+ 0x40)
#define SFVERSION					(HWVERSION	        + 0x40)
#define DEV_ID						(SFVERSION		+ 0x40)
#define CUSTOMERCODE                                    (DEV_ID		        + 0x40)  


void ReadPara(void);


//#define WORK_TIME_BASEE 					0x0800E810	
//#define PULSE_TOTAL_BASEE					0x0800F010
//#define RUN_ADDR_BASE				                                   0x00	                //������ʼ����־λ������ÿ���ϵ綼дEEPROMP
//#define SLAVE_ADDR					(RUN_ADDR_BASE		 + 0x01)                //�ӻ���ַ
//#define BAUDRATE		                        (SLAVE_ADDR	         + 0x01)                //������
//#define PARITY			                        (BAUDRATE	         + 0x01)                //��żУ��λ

//#define WORK_TIME_BASE1                                  (PARITY                  + 0x1)                 //������ʱ���ֵ
//#define STOP_TIME_BASE                                  (WORK_TIME_BASE1              + 0x04)                 //ֹͣʱ���ֵ
//#define POSITIVE_ROTATE_TIME_BASE                       (STOP_TIME_BASE               + 0x04)                 //��תʱ���ֵ
//#define NEGATIVE_ROTATE_TIME_BASE                       (POSITIVE_ROTATE_TIME_BASE              + 0x04)      //��תʱ���ֵ
//#define PULSE_TOTAL_BASE                                (NEGATIVE_ROTATE_TIME_BASE              + 0x04)       //��������ֵ
//#define DURATION_BASE                                   (PULSE_TOTAL_BASE              + 0x04)       //��������ֵ

//#define USER_DEFAULT_LEN				(DURATION_BASE                       + 0x04)


////---------------------------------------------------
//#define COMPANY						(USER_DEFAULT_LEN       + 0x40) 
//#define DEV_ENCODING				        (COMPANY		+ 0x40)
//#define HWVERSION					(DEV_ENCODING		+ 0x40)
//#define SFVERSION					(HWVERSION	        + 0x40)
//#define DEV_ID						(SFVERSION		+ 0x40)
//#define CUSTOMERCODE                                    (DEV_ID		        + 0x40)  
//		
//#define WORK_TIME_BASE					  (CUSTOMERCODE		        + 0x10)  		


//#define BAUDRATE_ADDR				0x08006000
//#define WORK_TIME_ADDR				0x08006800
//#define PULSE_TOTAL_ADDR			0x08007000


//void ReadPara(void);
//uint32_t GetDelExtremeAndAverage(uint32_t Array[], const uint32_t ArraySize,const uint32_t SortHeadSize, const uint32_t SortTailSize);
//void Switch_Fiter(uint8_t value);

#endif