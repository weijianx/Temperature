#ifndef ADC_H
#define ADC_H

#include "gd32e230.h"


//����ʱ�ӳ�ʼ����GPIO�ĳ�ʼ��
void rcu_config_inject(void);
	
void gpio_config(void);
void dma_config(void);
void adc_config(void);
void display_adc_value(void);



#endif



