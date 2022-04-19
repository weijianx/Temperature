//#include "BSP.h"
#include "TH_IIC.h"


uint8_t x, y1, y2;


void Delay_1Us(uint32_t cnt)
{
//    cnt = cnt  ;      //�����2,50�����ϻ���Ϊ0�ȣ����Ϊ4��������¶�
//    while (cnt--);      //16M����Ƶ����ʱҪ�޸�
	cnt = cnt * 1;

    while (cnt--);
}


void Delay_2Us(uint32_t cnt)
{
//    cnt = cnt  ;      //�����2,50�����ϻ���Ϊ0�ȣ����Ϊ4��������¶�
//    while (cnt--);      //16M����Ƶ����ʱҪ�޸�
	cnt = cnt * 5;

    while (cnt--);
}



void Delay_4Us(uint32_t cnt)
{
//    cnt = cnt  ;      //�����2,50�����ϻ���Ϊ0�ȣ����Ϊ4��������¶�
//    while (cnt--);      //16M����Ƶ����ʱҪ�޸�
	cnt = cnt * 16;

    while (cnt--);
}


void Delay_1Ms(uint32_t cnt)
{
    cnt = cnt * 940;

    while (cnt--);
   
  
}



/*******************************************************************************
// ����         : Get_GPIO_PIN
// ��������     : 2017-07-29
// ����         : ÷����
// ����         : ͨ��ͨ���Ż�ö�Ӧ��GPIO��PIN
// �������     : ��
// �������     : ��
// ���ؽ��     : ��
// ע���˵��   : ͨ����0-PA12;1-PA5;2-PB3;3-PB4
// �޸�����     : 
********************************************************************************/
void Get_Channal_Pin(uint8_t sChannel)
{
    switch(sChannel)
    {     
    case 0:
        x = 0;
        y1 = 9;
        y2 = 10;
        break;
        
    case 1:
        x = 1;
        y1 = 3;
        y2 = 4;
        break; 
        
    case 2:
        x = 1;
        y1 = 5;
        y2 = 12;
        break;

    case 3:
        x = 1;
        y1 =13;
        y2 = 14;
        break ;
        
    default:
        break;
    }
}



//******************************************************************************
// ����         : IIC_Init()
// ��������     : 2017-11-07
// ����         : IIC GPIO����
// �������     : ��
// �������     : ��
// ���ؽ��     : ��
// ע���˵��   : 
// �޸�����     :
//******************************************************************************
void TH_IIC_Init(void)
{
	
	rcu_periph_clock_enable(RCU_GPIOB);
	/* configure PA2(ADC channel2) as analog input */
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_6);
	
	gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
	
	gpio_bit_write(GPIOB, GPIO_PIN_6, RESET);
	Delay_1Us(10000);
	gpio_bit_write(GPIOB, GPIO_PIN_6, SET);
	
	gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_7);
	
	gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
	
	gpio_bit_write(GPIOB, GPIO_PIN_7, RESET);
	Delay_1Us(10000);
	gpio_bit_write(GPIOB, GPIO_PIN_7, SET);
//    GPIOA->MODER &= 0xFFF3FFFF;            
//    GPIOA->MODER |= 0x00040000;           //PA9ͨ�����ģʽ
//    GPIOA->OTYPER &= ~(1<<9);             //�������
//    GPIOA->OSPEEDR |= (3<<18);            //����  
//    GPIOA->PUPDR &= 0xFFF3FFFF;           //����
//    GPIOA->PUPDR |= 0x00040000;  
//    GPIOA->BRR = GPIO_PIN_9;            //���������ߣ����ֹ���ʱ��λһ������
//    Delay_1Us(10000);
//    GPIOA->BSRR = GPIO_PIN_9;
//    
//    GPIOA->MODER &= 0xFFCFFFFF;            
//    GPIOA->MODER |= 0x00100000;           //PA10ͨ�����ģʽ
//    GPIOA->OTYPER &= ~(1<<10);             //�������
//    GPIOA->OSPEEDR |= (3<<20);            //����  
//    GPIOA->PUPDR &= 0xFFCFFFFF;           //����
//    GPIOA->PUPDR |= 0x00100000; 
//    GPIOA->BRR = GPIO_PIN_10;
//    Delay_1Us(10000);
//    GPIOA->BSRR = GPIO_PIN_10;

}

void SDA_IN()
{
	rcu_periph_clock_enable(RCU_GPIOB);
	/* configure PA2(ADC channel2) as analog input */
    gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_7);
	
	gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
}


void SDA_OUT()
{
	rcu_periph_clock_enable(RCU_GPIOB);
	/* configure PA2(ADC channel2) as analog input */
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_7);
	
	gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);

}

/**@brief       ����IIC��ʼ�ź�
* @param[in]    ��
* @param[out]   ��
* @return       ��
* - None
* @note         
*/
void TH_IIC_Start(void)
{
    SDA_OUT();     //sda
    Set_IIC_SDA;
    Set_IIC_SCL;
//    GPIO_SetBits(GPIOC, GPIO_Pin_9);//del frank cl
    Delay_4Us(1);
//    GPIO_ResetBits(GPIOC, GPIO_Pin_9);
    Reset_IIC_SDA;//START:when CLK is high,DATA change form high to low
    Delay_4Us(1);
    Reset_IIC_SCL;
}

//void TH_IIC_Stop(void)
//{
//    SDA_I2C_OUT;
//    SCL_I2C_L;
//    SDA_I2C_L; 
//    Delay_1Us(4);
//    SCL_I2C_H;    
//    SDA_I2C_H;
//    Delay_1Us(4);
//}
/**@brief       ����IICֹͣ�ź�
* @param[in]    ��
* @param[out]   ��
* @return       ��
* - None
* @note         
*/
void TH_IIC_Stop(void)
{
    SDA_OUT();//sda
    Reset_IIC_SCL;
    Reset_IIC_SDA;//STOP:when CLK is high DATA change form low to high
    Delay_4Us(1);
    Set_IIC_SCL;
//    Set_IIC_SDA;//I2C change frank cl
    Delay_4Us(1);
	Set_IIC_SDA;//I2C
}


/**@brief       �ȴ�Ӧ���źŵ���
* @param[in]    ��
* @param[out]   ��
* @return       1������Ӧ��ʧ��  0������Ӧ��ɹ�
* - None
* @note         
*/
uint8_t TH_IIC_Wait_Ack(void)
{
    uint8_t ucErrTime = 0;
    SDA_IN();      //SDA
    Set_IIC_SDA;
    Delay_1Us(1);
    Set_IIC_SCL;
    Delay_1Us(1);

    while (READ_SDA)
    {
        ucErrTime++;

        if (ucErrTime > 250)
        {
            TH_IIC_Stop();
            return 1;
        }
    }

    Reset_IIC_SCL;

    return 0;
}


/**@brief       ����ACKӦ��
* @param[in]    ��
* @param[out]   ��
* @return       ��
* - None
* @note         
*/
void TH_IIC_Ack(void)
{
    Reset_IIC_SCL;
    SDA_OUT();
    Reset_IIC_SDA;
    Delay_2Us(1);
    Set_IIC_SCL;
    Delay_2Us(1);
    Reset_IIC_SCL;
}

/**@brief       ������ACKӦ��
* @param[in]    ��
* @param[out]   ��
* @return       ��
* - None
* @note         
*/
void TH_IIC_NAck(void)
{
    Reset_IIC_SCL;
    SDA_OUT();
    Set_IIC_SDA;
    Delay_2Us(1);
    Set_IIC_SCL;
    Delay_2Us(1);
    Reset_IIC_SCL;
}


/**@brief       IIC����һ���ֽ�
* @param[in]    ��
* @param[out]   ��
* @return       ��
* - None
* @note         
*/
void TH_IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
    SDA_OUT();
    Reset_IIC_SCL;

    for (t = 0;t < 8;t++)
    {
        Write_SDA((txd&0x80) >> 7);
        txd <<= 1;
        Delay_2Us(1);   
        Set_IIC_SCL;
        Delay_2Us(1);
        Reset_IIC_SCL;
        Delay_2Us(1);
    }
}


/**@brief       ��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK 
* @param[in]    ��
* @param[out]   ��
* @return       ��
* - None
* @note         
*/
uint8_t TH_IIC_Read_Byte(uint8_t ack)
{
    uint8_t i, receive = 0;
    SDA_IN();//SDA?????

    for (i = 0;i < 8;i++)
    {
        Reset_IIC_SCL;
        Delay_2Us(1);
        Set_IIC_SCL;
        receive <<= 1;

        if (READ_SDA)receive++;

        Delay_1Us(1);
    }

    if (!ack)
        TH_IIC_NAck();
    else
        TH_IIC_Ack(); 

    return receive;
}
