#include "SHT3x.h"
#include "TH_IIC.h"
//#include "usart.h"
#include "string.h"

#if 0
float gTemBuf[TEM_CH_NUM][TEM_FIL_NUM];                                        //4·�¶��˲�����
float gHumBuf[HUM_CH_NUM][HUM_FIL_NUM];                                        //4·ʪ���˲����� 
SensorSta_Typedef  SensorSta[2*SENSOR_NUM] = {NExist};                          //4·�¶�4·ʪ�ȵ�״̬����4�ԣ�  
SHT3_Typedef T_H;


//��鴫������״̬����modbus�е��ã�������鴫�����Ƿ����
SensorSta_Typedef Check_Status(uint8_t SlaveAddr)
{
    if((SlaveAddr >= 0x21) && (SlaveAddr <= 0x28))
    {
        return  SensorSta[SlaveAddr - 0x21];            
    }
    return NExist;
}



//��ó���ֵ����ʱ�䣬4���ֽڣ�0x00010000��ʾ����(��ʪ��)������0x00000001��ʾ����(��ʪ��)����
void Get_Duration(void)
{  
    static uint32_t TemCountCnt1 = 0,TemCountCnt2 = 0;                                          //�����ʼ��Ϊ0
    uint8_t  i,k;
    
    for(i = 0; i < SENSOR_NUM; i++)
    {
        if(T_H.Temp[i] > T_H.UpThreshold[i])                                   //�������ֵ
        {
            if(!(T_H.AlarmSta[i] & 0x00010000))                                   //������״̬��ת�����ڸ�ֵ
            {
                if(TemCountCnt1++ >= T_H.DurThreshold[i] - 1)                            //�������������ֵʱ��
                {         
                    TemCountCnt1 = 0;
                    T_H.AlarmSta[i] |= 0x00010000;                                //����
                    T_H.AlarmSta[i] &= 0x00010000;
                }
            }
            else                                                                   //����ʱ��+1s
            {
                TemCountCnt1 = 0;                                                    //�������ֵ��������������ֵ���¿�ʼ
                T_H.Duration[i] += 1;
            }
        }
        else if(T_H.Temp[i] < T_H.DoThreshold[i])                               //�������ֵ
        {
            if(!(T_H.AlarmSta[i] & 0x00000001))                                   //������״̬��ת�����ڵ�ֵ
            {
                if(TemCountCnt1++ >= T_H.DurThreshold[i] - 1)                            //�������������ֵʱ��
                {
                    TemCountCnt1 = 0;
                    T_H.AlarmSta[i] |= 0x00000001;                                //����
                    T_H.AlarmSta[i] &= 0x00000001;
                }
            }
            else                         
            {
                TemCountCnt1 = 0;                                                    //�������ֵ��������������ֵ���¿�ʼ
                T_H.Duration[i] += 1;                                             //����ʱ��+1s
            }
        }
        else                                                                        //����ֵ��Χ��
        {
            if(T_H.AlarmSta[i] != 0)                                              //�ӱ���״̬�л�������״̬
            {
                T_H.Duration[i] += 1;                                             //��Ҫ����+1s
                if(TemCountCnt1++ >= T_H.DurThreshold[i] - 1)                            //�������������ֵʱ��
                {
                    TemCountCnt1 = 0;
                    T_H.AlarmSta[i] = 0;
                    T_H.Duration[i] = 0;
                }
            }
            else
            {   
                TemCountCnt1 = 0;                                                    //�������ֵ��������������ֵ���¿�ʼ
                T_H.AlarmSta[i] = 0;
                T_H.Duration[i] = 0;
            }
        } 
    }

    
     for(i = SENSOR_NUM; i < 2 * SENSOR_NUM; i++)
    {
       k = i - SENSOR_NUM;
      if(T_H.Humi[k]  > T_H.UpThreshold[i])                                   //�������ֵ
        {
            if(!(T_H.AlarmSta[i] & 0x00010000))                                   //������״̬��ת�����ڸ�ֵ
            {
                if(TemCountCnt2++ >= T_H.DurThreshold[i] - 1)                            //�������������ֵʱ��
                {         
                    TemCountCnt2 = 0;
                    T_H.AlarmSta[i] |= 0x00010000;                                //����
                    T_H.AlarmSta[i] &= 0x00010000;
                }
            }
            else                                                                   //����ʱ��+1s
            {
                TemCountCnt2 = 0;                                                    //�������ֵ��������������ֵ���¿�ʼ
                T_H.Duration[i] += 1;
            }
        }
        else if(T_H.Humi[k]  < T_H.DoThreshold[i])                               //�������ֵ
        {
            if(!(T_H.AlarmSta[i] & 0x00000001))                                   //������״̬��ת�����ڵ�ֵ
            {
                if(TemCountCnt2++ >= T_H.DurThreshold[i] - 1)                            //�������������ֵʱ��
                {
                    TemCountCnt2 = 0;
                    T_H.AlarmSta[i] |= 0x00000001;                                //����
                    T_H.AlarmSta[i] &= 0x00000001;
                }
            }
            else                         
            {
                TemCountCnt2 = 0;                                                    //�������ֵ��������������ֵ���¿�ʼ
                T_H.Duration[i] += 1;                                             //����ʱ��+1s
            }
        }
        else                                                                        //����ֵ��Χ��
        {
            if(T_H.AlarmSta[i] != 0)                                              //�ӱ���״̬�л�������״̬
            {
                T_H.Duration[i] += 1;                                             //��Ҫ����+1s
                if(TemCountCnt2++ >= T_H.DurThreshold[i] - 1)                            //�������������ֵʱ��
                {
                    TemCountCnt2 = 0;
                    T_H.AlarmSta[i] = 0;
                    T_H.Duration[i] = 0;
                }
            }
            else
            {              
                TemCountCnt2 = 0;                                                    //�������ֵ��������������ֵ���¿�ʼ
                T_H.AlarmSta[i] = 0;
                T_H.Duration[i] = 0;
            }
        } 
      
      
    }
}


//��ʼ���˲������Լ�����SHT3��ת������
void SHT3_Init(void)
{
    uint8_t i, j, sta;
    for(i = 0; i < SENSOR_NUM; i++)
    {
        for(j = 0; j < FILTER_NUM; j++)
        {
            gTemBuf[i][j] = 25.0f;                                              //�¶Ȼ����ʼ����Ϊ25��
            gHumBuf[i][j] = 50.0f;                                              //ʪ�Ȼ����ʼ����Ϊ50%
        }
    }
    for(i = 0; i < SENSOR_NUM; i++)
    {
        sta = Set_Sht3_Periodi(i, SPERIODIC_MODE_SET_4_0_LOW);                  //����ת���ʸ�һ�㣬������������˻�ûӦ��
        if(sta)
        {
            SensorSta[i + SENSOR_NUM] = SensorSta[i] = NExist;                  //IIC״̬����
        }
        else
        {
            SensorSta[i + SENSOR_NUM] = SensorSta[i] = Exist;                   //IIC״̬����
        }
    }       
}



//**************************************************************************************************
// ����         : SHT3x_WriteData()
// ��������     : 2017-09-21
// ����         : MMX
// ����         : ��SHT3xд���ݣ�������ָ��
// �������     : uint8_t DriverAddr      IIC�����ĵ�ַ
//                uint16_t Command        ���͵�ָ��     
// �������     : ��
// ���ؽ��     : ������Ӧ��״̬
// ע���˵��   : 
// �޸�����     : 
//**************************************************************************************************
uint8_t SHT3x_WriteData(uint8_t DriverAddr, uint16_t Command)
{
    uint8_t sta = 0;
   // CPU_SR_ALLOC();
   // OS_CRITICAL_ENTER();
    TH_IIC_Start();                                                                //��ʼ�ź�
    TH_IIC_Send_Byte(DriverAddr);	                                                //����дָ��
    sta += TH_IIC_Wait_Ack();                                                      //Ӧ��
    TH_IIC_Send_Byte((Command >> 8) & 0xff);                                       //����ָ����ֽ�
    sta += TH_IIC_Wait_Ack();                                                      //Ӧ��
    TH_IIC_Send_Byte(Command & 0xff);                                              //����ָ����ֽ�
    sta += TH_IIC_Wait_Ack();                                                      //Ӧ��
    TH_IIC_Stop();                                                                 //ֹͣ�ź�
   // OS_CRITICAL_EXIT();
    return sta;
}



//**************************************************************************************************
// ����         : SHT3x_ReadData()
// ��������     : 2017-09-21
// ����         : MMX
// ����         : ��SHT3x������
// �������     : uint8_t DriverAddr      IIC�����ĵ�ַ
//                uint16_t Command        ���͵�ָ�� 
//                uint8_t* DataBuff       ���ݻ��� 
//                uint8_t DataLen         �������ݳ���    
// �������     : ��
// ���ؽ��     : ������Ӧ��״̬
// ע���˵��   : 
// �޸�����     : 
//**************************************************************************************************
uint8_t SHT3x_ReadData(uint8_t DriverAddr, uint16_t Command, uint8_t* DataBuff, uint8_t DataLen)
{
    uint8_t i;
    uint8_t sta = 0;
   // CPU_SR_ALLOC();
    //OS_CRITICAL_ENTER();
    TH_IIC_Start();                                                             //��ʼ�ź�
    TH_IIC_Send_Byte(DriverAddr);	                                            //����дָ��
    sta += TH_IIC_Wait_Ack();                                                   //Ӧ��
    TH_IIC_Send_Byte((Command >> 8) & 0xff);                                    //����ָ����ֽ�
    sta += TH_IIC_Wait_Ack();                                                   //Ӧ��
    TH_IIC_Send_Byte(Command & 0xff);                                           //����ָ����ֽ�
    sta += TH_IIC_Wait_Ack();                                                   //Ӧ��
	

    TH_IIC_Start();                                                             //��ʼ�ź�
    TH_IIC_Send_Byte(DriverAddr + 1);	                                        //���Ͷ�ָ��
    sta += TH_IIC_Wait_Ack();                                                   //Ӧ��

    for(i = 0; i < DataLen - 1; i++)                                            //��ʼ��ȡ����
    {
        *(DataBuff + i) = TH_IIC_Read_Byte(1);                                  //ǰ�����ֽ���ҪӦ��                                           
    }
    *(DataBuff + i)  = TH_IIC_Read_Byte(0);                                     //���һ���ֽڲ���ҪӦ��
    TH_IIC_Stop();
    //OS_CRITICAL_EXIT();
    return sta;
}


uint8_t Set_Sht3_Periodi(uint8_t sChannal, uint16_t sPeriodi)
{
    uint8_t sStatus;
    Get_Channal_Pin(sChannal);
    sStatus = SHT3x_WriteData(SHT3X_ADDR, SPERIODIC_MODE_SET_4_0_LOW);
    return sStatus;
}


uint8_t Read_Sht3_TH(uint8_t sChannal, uint16_t sReadCmd, uint8_t *sReadBuf, uint16_t sReadLen)
{
    uint8_t sStatus;
    Get_Channal_Pin(sChannal);
    sStatus = SHT3x_ReadData(SHT3X_ADDR, sReadCmd, sReadBuf, sReadLen);
    return sStatus;    
}


//******************************************************************************
// ����         : Filter()
// ��������     : 2017-09-022
// ����         : ÷����
// ����         : �������ֵ
// �������     : float NewData             �²�����ֵ
//                float* FilterBuf          ��ά������е�ַ 
//                uint16_t FilterNum             �˲�����    
//                uint16_t DelHeadNum            ��Ҫɾ������С��Ԫ�ظ���  
//                uint16_t DelTailNum            ��Ҫɾ��������Ԫ�ظ��� 
// �������     : ��
// ���ؽ��     : ƽ��ֵ
// ע���˵��   : 
// �޸�����     :
//******************************************************************************
float TH_Filter(float NewData, float* FilterBuf, uint16_t FilterNum, uint16_t DelHeadNum, uint16_t DelTailNum)
{
    int i, j;
    float filter_temp;
    float filter_sum = 0;
    float FilterArray[TEM_FIL_NUM];

    if(FilterNum != TEM_FIL_NUM)
        return 999.0f;
    if(DelHeadNum + DelTailNum >= FilterNum)
        return 888.0f;
                                                                                      
    for(i = 0; i < FilterNum - 1 ; i ++)                                        //FIFO�����µ����ݷ����������һ��Ԫ����           
    {
        FilterBuf[i] = FilterBuf[i + 1];
    }
    FilterBuf[i] = NewData;
    
    memcpy(FilterArray, FilterBuf, sizeof(FilterArray));                       //��������(����Ǳ���ģ�����ֱ�Ӷ�FilterBuf�������������ᶪ����ֵ)
                                                                                //���Ƶĸ���������sizeof
    for (i = 0; i < DelTailNum; i ++)                                           //������N��ֵ�Ƶ������                                    
    {
        for (j = 0; j < FilterNum - 1 - i; j ++)                                
        {
            if(FilterArray[j] > FilterArray[j + 1])
            {
                filter_temp = FilterArray[j];
                FilterArray[j] = FilterArray[j + 1];
                FilterArray[j + 1] = filter_temp;
            }
        }
    }
           
    for(i = 0; i < DelHeadNum; i ++)                                            //����С��M��ֵ�Ƶ���ǰ��                                      
    {
        for(j = FilterNum - DelTailNum - 1 ; j > i; j --)
        {
            if(FilterArray[j - 1] > FilterArray[j])                             //�����ĺô���ֻ��Ҫ����N+M�Σ������������������
            {
                filter_temp = FilterArray[j - 1];
                FilterArray[j - 1] = FilterArray[j];
                FilterArray[j] = filter_temp;
            }
        }
    }
     
    for(i = DelHeadNum; i < FilterNum - DelTailNum; i ++)                      //ȥ����С������ֵ���ֵ
        filter_sum += FilterArray[i];
    
    filter_sum /= (FilterNum - (DelHeadNum + DelTailNum));
    
    memset(FilterArray, 0x00, sizeof(FilterArray));                             //�������0һ�£���������ݸ���������sizeof
    
    return filter_sum;

}


void ReadTH(void)
{
   // OS_ERR err;
    uint8_t i;
    uint8_t status;
    uint8_t sDataBuff[6];
    uint16_t sTH[2];
    float sTem[SENSOR_NUM], sHum[SENSOR_NUM];

   // TH_IIC_Init();
   // SHT3_Init();
    while(1)
    {
     //   OSTimeDlyHMSM(0u, 0u, 1u, 0u, OS_OPT_TIME_HMSM_STRICT, &err);
       // if(err == OS_ERR_NONE)
       // {
            for(i = 0; i < SENSOR_NUM; i++)
            {
                status = Read_Sht3_TH(i, SPERIODIC_MODE_FETCH_DATA_CMD, sDataBuff, sizeof(sDataBuff));
                
                if(status)
                {
                    SensorSta[i] = NExist;
                    SensorSta[i + SENSOR_NUM] = SensorSta[i];
                }
                else
                {
                    SensorSta[i] = Exist;
                    SensorSta[i + SENSOR_NUM] = SensorSta[i];
                    
                    sTH[0] = ((uint16_t)sDataBuff[0] << 8) | sDataBuff[1];      //�������¶�����
                    sTH[1] = ((uint16_t)sDataBuff[3] << 8) | sDataBuff[4];      //������ʪ������
                    sTem[i] = -45.0f + 175.0f * sTH[0] / 65535.0f;              //��������϶�
                    sHum[i] = 100.0f * sTH[1] / 65535.0f;                       //����ɰٷֱ�
                    sTem[i] = TH_Filter(sTem[i], gTemBuf[i], TEM_FIL_NUM, DEL_HEAD_NUM, DEL_TAIL_NUM);//�¶��˲�
                    sHum[i] = TH_Filter(sHum[i], gHumBuf[i], HUM_FIL_NUM, DEL_HEAD_NUM, DEL_TAIL_NUM);//ʪ���˲�
                    T_H.Temp[i] = (uint16_t)((sTem[i] + 273.1f) * 10.0f + 0.5f);
                    T_H.Humi[i] = (uint16_t)(sHum[i] * 10.0f + 0.5f);
                }
            }
            Get_Duration();
            break;
            //Re_Connect_Handler();
       // }
    }
}
#endif

