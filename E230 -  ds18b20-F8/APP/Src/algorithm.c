#include "algorithm.h"



//�˲�
int Get_filter(int PCap_buf)
{
	uint8_t i=0,j=0;	
	uint32_t buf1 = 0,bufz[7]={0};
	static uint32_t buf_PCap[9]={0},buf2;
        int buf3;
//	static uint16_t buf3;
	uint8_t signs = 0,initial = 0;
	
	if(signs >= 9)
		signs = 0;
	
	if(initial < 8)
	{
		buf2 = 0;
		initial++;
		buf_PCap[signs]=(uint32_t)PCap_buf;
		for(i = 0;i <= signs;)
		{
			buf2+=buf_PCap[i++];
		}
		signs++;
        return PCap_buf;        
	}
	
			
	buf_PCap[signs]=(uint32_t)PCap_buf;			//��Чֵ
	
	j = signs++;
	for(i = 0;i<7;i++)
	{
		bufz[i] = buf_PCap[j];
		if(j <= 0)
			j = 9;
		j--;
	}
	
	for(i=0; i<7; i++)
	{
		for(j=0; j<7-i; j++)
		{
			if(bufz[j] > bufz[j+1])
			{
				buf1 = bufz[j];
				bufz[j] = bufz[j+1];
				bufz[j+1] = buf1;
			}	
		}
	}

     buf3 = (int)(bufz[1]+bufz[2]+bufz[3]+bufz[4]+bufz[5]);

	return buf3;
}


//******************************************************************************
// ����         : SortArrayExtreme()
// ��������     : 2016-09-05
// ����         : ׯ��Ⱥ
// ����         : �������ڵ�Ԫ�ذ���С�����˳������
// �������     : u16 Array[]               ����
//                const u32 ArraySize       ����ĳ���
//                const u32 SortHeadSize    ����ͷ��Ԫ�ظ���   
//                const u32 SortTailSize    ����β��Ԫ�ظ���
// �������     : ��
// ���ؽ��     : ��
// ע���˵��   : 
// �޸�����     :
//******************************************************************************
static void SortArrayExtreme(int Array[], const uint32_t ArraySize,
                      const uint32_t SortHeadSize, const uint32_t SortTailSize)
{
    uint32_t i, j;
    int temp;

    for (i = 0; i < SortTailSize; i++)
    {
        for (j = 0; j < ArraySize - i - 1; j++)
        {
            if (Array[j] > Array[j+1])
            {
                temp = Array[j];
                Array[j] = Array[j+1];
                Array[j+1] = temp;
            }
        }
    }

    for (i = 0; i < SortHeadSize; i++)
    {
        for (j = ArraySize - SortTailSize - 1 ; j > i; j--)
        {
            if (Array[j - 1] > Array[j])
            {
                temp = Array[j - 1];
                Array[j - 1] = Array[j];
                Array[j] = temp;
            }
        }
    }
}



//******************************************************************************
// ����         : GetAverage()
// ��������     : 2016-09-05
// ����         : ׯ��Ⱥ
// ����         : ȥ��������С���ֵ
// �������     : u16 Array[]               ����
//                const u32 ArraySize       ����ĳ���
//                const u32 SortHeadSize    ����ͷ��Ԫ�ظ���   
//                const u32 SortTailSize    ����β��Ԫ�ظ���
// �������     : ��
// ���ؽ��     : ƽ��ֵ
// ע���˵��   : 
// �޸�����     :
//******************************************************************************
static int GetAverage(int Array[], const uint32_t ArraySize,
               const uint32_t DelHeadSize, const uint32_t DelTailSize)
{
    long long int sum = 0;

    if ((DelHeadSize + DelTailSize) >= ArraySize)
    {
        return 0;
    }

    for (long i = DelHeadSize; i < ArraySize - DelTailSize; i++)
    {
        sum += Array[i];
    }

    return(sum / (ArraySize - DelHeadSize - DelTailSize));
}



//******************************************************************************
// ����         : GetDelExtremeAndAverage()
// ��������     : 2016-09-05
// ����         : ׯ��Ⱥ
// ����         : �������ֵ
// �������     : u16 Array[]               ����
//                const u32 ArraySize       ����ĳ���
//                const u32 SortHeadSize    ����ͷ��Ԫ�ظ���   
//                const u32 SortTailSize    ����β��Ԫ�ظ���
// �������     : ��
// ���ؽ��     : ƽ��ֵ
// ע���˵��   : 
// �޸�����     :
//******************************************************************************
int GetDelExtremeAndAverage(int Array[], const uint32_t ArraySize,
                            const uint32_t SortHeadSize, const uint32_t SortTailSize)
{
    SortArrayExtreme(Array, ArraySize, SortHeadSize, SortTailSize);
    return(GetAverage(Array, ArraySize, SortHeadSize, SortTailSize));
}

