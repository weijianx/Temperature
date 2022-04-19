/*!
    \file  main.c
    \brief USART HyperTerminal Interrupt
    
    \version 2018-06-19, V1.0.0, firmware for GD32E230
*/

/*
    Copyright (c) 2018, GigaDevice Semiconductor Inc.

    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32e230.h"
#include "gd32e230c_eval.h"
#include "systick.h"
#include <stdio.h>

#include "timer.h"
#include "type.h"
#include "para.h"

#include "algorithm.h"

#include "SHT3x.h"
#include "TH_IIC.h"

#if ds18b20
#include "ds18b20.h"
#endif

#define THR(x) (x - 2731) * 10000  
#define TENM_FIL_NUM    10

uint8_t receive = 1;
uint32_t receive1=0;
uint8_t bou,bou1;
uint16_t tt1=0;
uint32_t time1=0;
extern BitAction UartRecvFrameOK;

extern UserTypeDef UserPara;
extern FlagStatus UartRecvFlag;
FlagStatus StaChangeFlag = RESET;

int TemFilterBuf[TENM_FIL_NUM];
int TemFilterBufBak[TENM_FIL_NUM];

uint8_t transmitter_buffer[] = "\n\rUSART interrupt test\n\r";

uint8_t time_tick = 10;  //默认滤波方式  为平稳滤波


static void TemData_Handle(void);

void DRV_IWDG_Init(void)
{
	rcu_osci_on(RCU_IRC40K);
    while(SUCCESS != rcu_osci_stab_wait(RCU_IRC40K));
	
	/*	FWDG clock is independent lsi 40KHz, DIV128 IS 312.5Hz, that is 3.2 ms per clock
	reload value is 1000, delay time 2500 * 3.2ms = 8 s	*/
	fwdgt_config( 2500, FWDGT_PSC_DIV256 );
	fwdgt_enable();
	
}



void rs485_Init(void)
{
	/* enable the GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);
	/* configure PA2(ADC channel2) as analog input */
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_4);
	
	gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_4);
	
	gpio_bit_reset(GPIOA,GPIO_PIN_4);

}

void delayms()
{
	uint16_t i=0xFF,j=0xFF;
	for(;i>0;i--)
		for(;j>0;j--);
}


/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int16_t temp1;
int PreTemp;
uint16_t i = 0;
uint8_t j; 
int main(void)
{
	//中断向量设置，APP升级
	nvic_vector_table_set(FLASH_BASE, 0x4000);  
	
	uint8_t ch[] = "dggsgawegf\r\n";
	
	j = 0;
	
	nvic_irq_enable(USART1_IRQn, 2);
     
    /* initilize the com */
    com_gpio_init();
    com_usart_init();
    
    /* enable USART TBE interrupt */  
//    usart_interrupt_enable(USART1, USART_INT_TBE);
    
	usart_interrupt_enable(USART1, USART_INT_RBNE);	
	
	DRV_IWDG_Init();
	
//	timer_config();
	TIMER2_Init();

	TIMER15_Init();
		
	ReadPara(); 
	
	DS18B20_Init();
	
    while (1)
	{
//		gpio_bit_toggle(GPIOA, GPIO_PIN_1);
//		delay_2us();
		fwdgt_counter_reload();

		if(Time_1s_flag)
		{
			i++;
			if(i >=2 )
				i = 3;
			Time_1s_flag = 0;

			TemData_Handle(); 
		}

//		TemData_Handle(); 

		if(i >= 2)                                                      //前3s的数据不要（因为会读出个85度）
		{
			MBASC_Function();
		}
		

	}
	
}

int red_temp;
static void TemData_Handle(void)
{
    static uint32_t TemCountCnt = 0;                                           //必须初始化为0
    static uint32_t ReadTemCnt = 0; 
    static FlagStatus FillBufStatus = RESET;
    static uint8_t temcnt=0;
	
//    int PreTemp;
	

	PreTemp = DS18B20_Get_Temp();                                               //每秒读一次温度值
	
	if(j < 3)
	{
		j++;
		red_temp = PreTemp;
	}
	
	if((-1000000 > (PreTemp - red_temp)) || ((PreTemp - red_temp) > 1000000))
	{
		DS18B20_Init();                                                         //出现故障时复位一下总线
        ReadTemCnt = 0;
		fwdgt_counter_reload();
        return;
	}
	else
	{
		red_temp = PreTemp;
	}

    if(PreTemp == -27310000)                                                    //初始化18B20不成功,上电会读个85度上来，去掉    
    {
        DS18B20_Init();                                                         //出现故障时复位一下总线
        ReadTemCnt = 0;
        return;
    }
    
    if((PreTemp < -4000000) || (PreTemp > 11000000))                            //温度不在-30~110范围内丢掉
    {
        return;
    }
//    if(UartRecvFlag == RESET)                                                   //在接收数据的过程中会产生串口中断
//    {
        UserPara.Temp = PreTemp;                                                //影响读数据的准确性,所以在接收数据的过程中读取的数据丢掉
//    }
//    if(FillBufStatus == RESET)                                                  //开始的十几秒得到稳定数据再打开串口
//    {
//        TemFilterBuf[temcnt++] = UserPara.Temp;
//        if(temcnt >= TENM_FIL_NUM)
//        {
//            memcpy((uint8_t*)TemFilterBufBak, (uint8_t*)TemFilterBuf, TENM_FIL_NUM * 4);
//            UserPara.Temp = (int)GetDelExtremeAndAverage(TemFilterBufBak, TENM_FIL_NUM, TENM_FIL_NUM / 3u, TENM_FIL_NUM / 3u); 

//			usart_interrupt_enable(USART1, USART_INT_RBNE);
//            FillBufStatus = SET;
//        }
//        else
//        {
//            return;
//        }
//    }
//    memcpy((uint8_t*)TemFilterBuf, (uint8_t*)TemFilterBuf + 4, (TENM_FIL_NUM - 1) * 4);
//    *(TemFilterBuf + (TENM_FIL_NUM - 1)) = UserPara.Temp;                       //得到最新值，滤波
//    memcpy((uint8_t*)TemFilterBufBak, (uint8_t*)TemFilterBuf, TENM_FIL_NUM * 4);
//    UserPara.Temp = (int)GetDelExtremeAndAverage(TemFilterBufBak, TENM_FIL_NUM, TENM_FIL_NUM / 3u, TENM_FIL_NUM / 3u);    
    
    if(UserPara.Temp >  THR(UserPara.Up_Thr))                                   //高于最高值
    {
        if(!(UserPara.AlarmSta & 0x00010000))                                   //从正常状态跳转到高于高值
        {
            if(TemCountCnt++ >= UserPara.Du_Thr - 1)                            //如果持续超过阀值时间
            {         
                TemCountCnt = 0;
                UserPara.AlarmSta |= 0x00010000;                                //报警
                UserPara.AlarmSta &= 0x00010000;
            }
        }
        else                                                                    //持续时间+1s
        {
            TemCountCnt = 0;                                                    //如果在阈值附近抖动，计数值重新开始
            UserPara.Duration += 1;
        }
    }
    else if(UserPara.Temp < THR(UserPara.Do_Thr))                               //低于最低值
    {
        if(!(UserPara.AlarmSta & 0x00000001))                                   //从正常状态跳转到低于低值
        {
            if(TemCountCnt++ >= UserPara.Du_Thr - 1)                            //如果持续超过阀值时间
            {
                TemCountCnt = 0;
                UserPara.AlarmSta |= 0x00000001;                                //报警
                UserPara.AlarmSta &= 0x00000001;
            }
        }
        else                         
        {
            TemCountCnt = 0;                                                    //如果在阈值附近抖动，计数值重新开始
            UserPara.Duration += 1;                                             //持续时间+1s
        }
    }
    else                                                                        //在阈值范围内
    {
        if(UserPara.AlarmSta != 0)                                              //从报警状态切换到正常状态
        {
            UserPara.Duration += 1;                                             //先要继续+1s
            if(TemCountCnt++ >= UserPara.Du_Thr - 1)                            //如果持续超过阀值时间
            {
                TemCountCnt = 0;
                UserPara.AlarmSta = 0;
                UserPara.Duration = 0;
            }
        }
        else
        {
            TemCountCnt = 0;                                                    //如果在阈值附近抖动，计数值重新开始
            UserPara.AlarmSta = 0;
            UserPara.Duration = 0;
        }
    }    
}

/*!
    \brief      initilize the com GPIO
    \param[in]  none
    \param[out] none
    \retval     none
*/
void com_gpio_init(void)
{
    /* enable COM GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);

    /* connect port to USARTx_Tx */
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_2);

    /* connect port to USARTx_Rx */
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_3);

    /* configure USART Tx as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_2);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_2);

    /* configure USART Rx as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_3);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_3);
}

/*!
    \brief      initilize the USART configuration of the com
    \param[in]  none
    \param[out] none
    \retval     none
*/
extern UserTypeDef UserPara;
void com_usart_init(void)
{
//	rs485_Init();
	
    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART1);

	flash_read_multi(Baudrate_ADDR, &(UserPara.Baudrate), 1);
    /* USART configure */
    usart_deinit(USART1);
    switch(UserPara.Baudrate)
	{
		case 01:
			usart_baudrate_set(USART1, 2400U);
			break;
		
		case 02:
			usart_baudrate_set(USART1, 4800U);
			break;
		
		case 03:
			usart_baudrate_set(USART1, 9600U);
			break;
		
		case 04:
			usart_baudrate_set(USART1, 19200U);
			break;
		
		case 05:
			usart_baudrate_set(USART1, 38400U);
			break;

		case 06:
			usart_baudrate_set(USART1, 57600U);
			break;
		
		case 07:
			usart_baudrate_set(USART1, 115200U);
			break;
		default:
			usart_baudrate_set(USART1, 9600U);
			break;
	
	}
//	usart_baudrate_set(USART1, 9600U);
	
	rs485_Init();
	
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);

    usart_enable(USART1);
}




/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART1, (uint8_t) ch);
    while(RESET == usart_flag_get(USART1, USART_FLAG_TBE));
    return ch;
}
