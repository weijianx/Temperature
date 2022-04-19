#include "timer.h"
#include "type.h"



/*!
    \brief      TIMER configuration function
    \param[in]  none
    \param[out] none
    \retval     none
*/
//void timer_config(void)
//{
//   timer_oc_parameter_struct timer_ocintpara;
//    timer_parameter_struct timer_initpara;
//	timer_ic_parameter_struct timer_icinitpara;

//    rcu_periph_clock_enable(RCU_TIMER0);
//	rcu_periph_clock_enable(RCU_GPIOA);
////	rcu_periph_clock_enable(RCU_AF);
//	
////	gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_8 | GPIO_PIN_9);
////	gpio_af_set(GPIOA, GPIO_AF_2, GPIO_PIN_8 | GPIO_PIN_9);
////	gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8 | GPIO_PIN_9);
//	
//	timer_deinit(TIMER0);

//    /* TIMER configuration */
//    timer_initpara.prescaler         = 71;
//    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
//    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
//    timer_initpara.period            = 65535;
//    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
//    timer_initpara.repetitioncounter = 0;
//    timer_init(TIMER0,&timer_initpara);
//	
////	*(uint32_t *)(TIMER0 + 0x18) |= (1<<4 | 1<<12);			//�˲�����
////	*(uint32_t *)(TIMER0 + 0x20) &= ~(1<<1);				//ch0����������
////	*(uint32_t *)(TIMER0 + 0x18) |= 1;						//ch0���� is0ӳ����ci0fe0
////	
////	*(uint32_t *)(TIMER0 + 0x20) |= (1<<5);					//ch1�����½���
////	*(uint32_t *)(TIMER0 + 0x18) |= (2<<8);					//ch1���� is1ӳ����ci0fe1
////	
////	*(uint32_t *)(TIMER0 + 0x0c) |= (3<<1);					//ʹ��ch0 ch1�Ƚϲ����ж�
////	*(uint32_t *)(TIMER0 + 0x20) |= (1|1<<4);				//ʹ��ch0 ch1�Ƚϲ���
////	
////	*(uint32_t *)(TIMER0 + 0x00) |= 1;						//������ʹ��
//	
////	
//	timer_channel_input_struct_para_init(&timer_icinitpara);    //�����벶��ṹ�������Ϊ��ʼֵ
//	timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;    //ͨ�����뼫��
//	timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI; //ͨ������ģʽѡ��
//	timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;           //ͨ������Ԥ��Ƶ��
//	timer_icinitpara.icfilter    = 0x00;                         //ͨ�����벶���˲�
//	timer_input_capture_config(TIMER0, TIMER_CH_0, &timer_icinitpara);
//	

//	timer_channel_input_struct_para_init(&timer_icinitpara);    //�����벶��ṹ�������Ϊ��ʼֵ
//	timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;    //ͨ�����뼫��
//	timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI; //ͨ������ģʽѡ��
//	timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;           //ͨ������Ԥ��Ƶ��
//	timer_icinitpara.icfilter    = 0x00;                         //ͨ�����벶���˲�
//	timer_input_capture_config(TIMER0, TIMER_CH_1, &timer_icinitpara);
//	
//	
//    /* auto-reload preload enable */
//    timer_auto_reload_shadow_enable(TIMER0);
////    timer_primary_output_config(TIMER0, ENABLE);
//	timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_CH0);
//	timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_CH1);
//	
//	timer_interrupt_enable(TIMER0, TIMER_INT_FLAG_CH0);
//	timer_interrupt_enable(TIMER0, TIMER_INT_FLAG_CH1);
//	
//	nvic_irq_enable(TIMER0_Channel_IRQn, 2);
//	
////	TIMER_CNT(TIMER0) = 0;
//	timer_enable(TIMER0);
//	
//}


void TIMER2_Init(void)
{
	timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER2);

    timer_deinit(TIMER2);

    /* TIMER configuration */
    timer_initpara.prescaler         = 7199;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 9999;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER2,&timer_initpara);

    /* CH0 configuration in PWM mode1 */
//    timer_ocintpara.ocpolarity  = TIMER_OC_POLARITY_LOW;
//    timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
//    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocintpara);

//    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, 100);
//    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM1);
//    timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

	nvic_irq_enable(TIMER2_IRQn, 2);
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER2);
//    timer_primary_output_config(TIMER0, ENABLE);
	
	timer_interrupt_enable(TIMER2, TIMER_INT_UP);
	timer_enable(TIMER2);

	
}



void TIMER15_Init(void)
{
	timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER15);

    timer_deinit(TIMER15);

    /* TIMER configuration */
    timer_initpara.prescaler         = 7199;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 9999;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER15,&timer_initpara);

    /* CH0 configuration in PWM mode1 */
//    timer_ocintpara.ocpolarity  = TIMER_OC_POLARITY_LOW;
//    timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
//    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocintpara);

//    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, 100);
//    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM1);
//    timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

	nvic_irq_enable(TIMER15_IRQn, 3);
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER15);
//    timer_primary_output_config(TIMER0, ENABLE);
	
	timer_interrupt_disable(TIMER15, TIMER_INT_UP);
	timer_disable(TIMER15);

	
}



uint16_t tim_cnt = 0;   //��ʱ���ۼ�
uint16_t tim_1min_cnt = 0;   //1minʱ���ۼ�

uint8_t Time_5s_flag = 0;
uint8_t Time_1s_flag = 0;
uint16_t CH0_cnt = 0;
uint16_t CH1_cnt = 0;

uint16_t CH_cnt = 0;

extern UserTypeDef UserPara;
BitAction  PulseFlag = Bit_RESET;                                         //��ʼװ�������־

UserTypeDef status;


uint8_t Time_1min_flag = 0;
uint8_t first_20s = 0;   //�������ǰ20��
extern uint16_t Current_pulse;  //��ǰ������

uint16_t pulse_flag0 = 0;           //������־λ
uint16_t pulse_flag1 = 0;			//��ת��־λ


uint16_t status_flag = 0;			//����״̬��־λ

uint16_t timeD_flag = 0;			//�ж�ʱ����־λ

void TIMER0_Channel_IRQHandler(void)
{

	if(timer_interrupt_flag_get(TIMER0, TIMER_INT_FLAG_CH0))
	{

		timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_CH0);
		
		if(pulse_flag0 >= pulse_flag1)
		{
			pulse_flag0++;
			CH0_cnt++;

		}
		
		else
		{
			pulse_flag1 = 0;
		}
		
		timer_interrupt_enable(TIMER0, TIMER_INT_FLAG_CH1);
		
	}

	if(timer_interrupt_flag_get(TIMER0, TIMER_INT_FLAG_CH1))
	{

		timer_interrupt_disable(TIMER0, TIMER_INT_FLAG_CH0);

		timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_CH1);
		
		if(pulse_flag1 >= pulse_flag0)
		{

			pulse_flag1++;

			CH1_cnt++;
				
		}
		
		else
		{			
			pulse_flag0 = 0;
		}	

		timer_interrupt_enable(TIMER0, TIMER_INT_FLAG_CH0);
	}


}


//uint16_t tim_cnt1=0;
void TIMER2_IRQHandler(void)
{
	uint16_t read_pulse = 0;  //��ǰ������
	uint16_t i;
	
//	uint8_t ch[] = "awegf\r\n";
	if(timer_interrupt_flag_get(TIMER2, TIMER_INT_FLAG_UP))
	{
		timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);
		Time_1s_flag = 1;


	}


}


uint16_t AutoUpLoad_counter=0;
void TIMER15_IRQHandler(void)
{
	uint16_t read_pulse = 0;  //��ǰ������
	uint16_t i;
	
//	uint8_t ch[] = "awegf\r\n";
	if(timer_interrupt_flag_get(TIMER15, TIMER_INT_FLAG_UP))
	{
		timer_interrupt_flag_clear(TIMER15, TIMER_INT_FLAG_UP);
		AutoUpLoad_counter++;
//		Time_1s_flag = 1;


	}


}


