/*!
    \file  flash_access.c
    \brief flash access functions

    \version 2014-12-26, V1.0.0, demo for GD32F10x
    \version 2017-06-20, V2.0.0, demo for GD32F10x
    \version 2018-07-31, V2.1.0, demo for GD32F10x
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


#include "flash.h"

/* pages 0 and 1 base and end addresses */

//Functions implemnet
/***************************************************************************************
Name:   
Func:   
Para:   
Retn:   
***************************************************************************************/
bool GDFLASHOP_EraseSpecifiedFlashPages( u32 startaddr, u16 npages )
{
    if (!npages)
        return false;

    if (startaddr + npages * SOWAYFLASH_PAGE_SIZE > CHIPFLASH_EDGE_ADDR)
        return false;
    
//	__disable_irq();
    fmc_unlock();
	fmc_flag_clear(FMC_FLAG_END);
    fmc_flag_clear(FMC_FLAG_WPERR);
    fmc_flag_clear(FMC_FLAG_PGERR);
	
    while (npages --)
    {
#ifdef USE_IDWDG
            FEED_I_WATCHDOG();
#endif
        if (fmc_page_erase( startaddr ) !=  FMC_READY)
            return false;
        startaddr += SOWAYFLASH_PAGE_SIZE;
    }
	fmc_unlock();
	fmc_flag_clear(FMC_FLAG_END);
    fmc_flag_clear(FMC_FLAG_WPERR);
    fmc_flag_clear(FMC_FLAG_PGERR);
    fmc_lock();
//    __enable_irq();
    
    return true;
}



/***************************************************************************************
Name:   
Func:   
Para:   
Retn:   
***************************************************************************************/
bool GDFLASHOP_WriteData2SpecifiedFlash( u32 startaddr, u32 *psrc, u16 wdlen )
{
	uint32_t i= 0;
	if ((psrc == NULL) || (!wdlen))
        return false;
    
    if (startaddr + wdlen * 4 > CHIPFLASH_MAXIMUM_ADDR)
        return false;
    
     fmc_unlock();
    __disable_irq();
    while (wdlen --)
    {
      //  if ( fmc_word_program( startaddr, *psrc ) != FMC_READY)
		if ( fmc_word_program( startaddr,psrc[i++]) != FMC_READY)
            return false;
        startaddr += 4;
    }
    __enable_irq();
    fmc_lock();
    
    return true;
}




/***************************************************************************************
Name:   
Func:   
Para:   
Retn:   
***************************************************************************************/
bool GDFLASHOP_ReadDataFromChipFlash( u8* pdest, u32 op_addr, u32 dwbufsize, u32 dwlen )
{
	u32 val;
    
    if ((!pdest) || (dwbufsize < dwlen))
        return false;
    
    if (op_addr < SOWAYFLASH_OP_BASE_ADDR)
        return false;
    
    while (dwlen --)
    {
        val = *(u32*)op_addr;
        *pdest ++ = (u8)val;
        *pdest ++ = (u8)(val >> 8);
        *pdest ++ = (u8)(val >> 16);
        *pdest ++ = (u8)(val >> 24);
        op_addr += 4;
    }
    return true;
}



/* pages 0 and 1 base and end addresses */
uint32_t bak[128];
static uint32_t Flash_Buf[SOWAYFLASH_PAGE_SIZE];    ///< ��ʱ����


uint8_t Flash_Read_OneByte(uint32_t RWAddr)
{
	uint8_t RdData;
//	uint8_t *pSource = (uint8_t *)(RWAddr);
//	RdData = *pSource;
	RdData = *(uint8_t *)(RWAddr);
	return RdData;
}

void flash_read_multi(uint32_t readAdder, uint8_t *readBuf, uint16_t readLen)
{
	uint16_t i;
	
	for(i=0; i<readLen; i++)
	{
		*(readBuf + i) = Flash_Read_OneByte(readAdder);
		readAdder++;
//		if(*(readBuf + i) == NULL)
//			break;
	}
}


uint32_t  flash_read_MultiBytes(uint32_t read_addr, uint8_t *pBuf, uint16_t len)
{
    uint32_t i;
    uint8_t *pSource = (uint8_t *)(read_addr);

    /* Data transfer */
    
	for (i = 0; i < len; i++) {
		*pBuf++ = *pSource++;
	}
    

    return 0;
}

uint32_t  flash_read32_MultiBytes(uint32_t read_addr)
{
	uint32_t buf;
	buf = *(__IO uint32_t *)read_addr;
    return buf;
}	

/**@brief       ���ڲ�Flashָ��λ��д����ֽ�
* @param[in]    RWAddr : д��ʼ��ַ
* @param[in]    pWrbuf : ���ݻ���ָ��
* @param[in]    Wrlen : д���ݳ���
* @return       ����ִ�н�� 1 �ɹ�  0 ʧ��
* @note  		Wrlen < 2048,����������д2048���ֽڣ��ɿ�ҳд
*/
uint8_t Flash_Write_OneByte(uint32_t RWAddr, uint8_t WrData)
{
    uint32_t WrAddr;
    uint16_t i;
    uint8_t *buf = (uint8_t *)Flash_Buf;

    WrAddr = (RWAddr / SOWAYFLASH_PAGE_SIZE) * SOWAYFLASH_PAGE_SIZE;
    flash_read_MultiBytes(WrAddr, buf, SOWAYFLASH_PAGE_SIZE);
    buf[RWAddr % SOWAYFLASH_PAGE_SIZE] = WrData;
    /* ����flash����/���������� */
	fmc_unlock();  //����FLASH��̲���������
    fmc_flag_clear(FMC_FLAG_PGERR|FMC_FLAG_PGAERR|FMC_FLAG_WPERR|FMC_FLAG_END);//�����־λ
	
    if(buf[RWAddr % SOWAYFLASH_PAGE_SIZE] != 0xFF)
    {
		fmc_page_erase(RWAddr);     //����ָ����ַҳ
    }
   
    for(i = 0; i < SOWAYFLASH_PAGE_SIZE;)
    {
		fmc_word_program((WrAddr + i), (uint32_t)Flash_Buf[i/4]); //��ָ��ҳ��addr��ַ��ʼд
		i+=4;
	}
//	FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);//�����־λ
    fmc_lock();    //����FLASH��̲���������
	if( (*(__IO uint8_t *)RWAddr) !=  WrData)
		return 0;
	return 1;
}



/**@brief       ���ڲ�Flashָ��λ��д����ֽ�
* @param[in]    RWAddr : д��ʼ��ַ
* @param[in]    pWrbuf : ���ݻ���ָ��
* @param[in]    Wrlen : д���ݳ���
* @return       ����ִ�н�� 1 �ɹ�  0 ʧ��
* @note  		Wrlen < 2048,����������д2048���ֽڣ��ɿ�ҳд
*/
uint8_t Flash_Write_MultiBytes(uint32_t RWAddr, uint8_t const *pWrbuf, uint16_t Wrlen)
{
	uint32_t WrAddr;
    uint32_t i;
    uint8_t *buf = (uint8_t *)Flash_Buf;
	
	uint32_t buf_WrAddr;
	uint32_t RWAddr1 = 0;
	uint16_t Wrlen1=0;
	uint16_t remain;
    uint16_t off;
	
	WrAddr = (RWAddr / SOWAYFLASH_PAGE_SIZE) * SOWAYFLASH_PAGE_SIZE;		
	
	off = RWAddr % SOWAYFLASH_PAGE_SIZE;
	remain = SOWAYFLASH_PAGE_SIZE - off;
	if(Wrlen < remain)
	{
		remain = Wrlen;
	}

    flash_read_multi(WrAddr, buf, SOWAYFLASH_PAGE_SIZE);
//	 flash_read_MultiBytes(WrAddr, buf, SOWAYFLASH_PAGE_SIZE);
		for(i=0;i<remain;i++)
		{
			buf[off+i] = pWrbuf[i];
		}

		/* ����flash����/���������� */
		fmc_unlock();  //����FLASH��̲���������
		fmc_flag_clear(FMC_FLAG_PGERR|FMC_FLAG_PGAERR|FMC_FLAG_WPERR|FMC_FLAG_END);//�����־λ
		fmc_page_erase(RWAddr);     //����ָ����ַҳ

		for(i = 0; i < SOWAYFLASH_PAGE_SIZE;)
		{
			fmc_word_program((WrAddr + i), (uint32_t)Flash_Buf[i/4]); //��ָ��ҳ��addr��ַ��ʼд
			i+=4;
		}
		fmc_lock();  //����FLASH��̲���������

   

	
	if( (*(__IO uint8_t *)RWAddr) !=  pWrbuf[0])
		return 0;
	if(Wrlen1 != 0)
	{
		Flash_Write_MultiBytes(RWAddr1, pWrbuf+buf_WrAddr,Wrlen1);
		RWAddr1 = 0;
		Wrlen1=0;
	}
	return 1;
}


uint8_t Flash_Write_32Bytes(uint32_t RWAddr, uint32_t data)
{

	/* ����flash����/���������� */
	fmc_unlock();  //����FLASH��̲���������
    fmc_flag_clear(FMC_FLAG_PGERR|FMC_FLAG_PGAERR|FMC_FLAG_WPERR|FMC_FLAG_END);//�����־λ
    fmc_page_erase(RWAddr);     //����ָ����ַҳ
  

		fmc_word_program(RWAddr,data); //��ָ��ҳ��addr��ַ��ʼд


	return 1;
}

uint32_t flash_write_word (uint32_t write_addr, uint32_t *pBuf, uint16_t wreLen)
{
	uint8_t i;
//	uint32_t start_page = (write_addr / SOWAYFLASH_PAGE_SIZE) * SOWAYFLASH_PAGE_SIZE + BASE_ADDRESS;
//	uint32_t start_wr_addr = write_addr + BASE_ADDRESS;
	uint32_t start_wr_addr = write_addr;
	
	fmc_unlock();
//	fmc_page_erase(write_addr);
	fmc_flag_clear(FMC_FLAG_PGERR|FMC_FLAG_PGAERR|FMC_FLAG_WPERR|FMC_FLAG_END);
	for(i=0; i<wreLen; i++)
	{
		fmc_word_program(start_wr_addr + (i * 4), pBuf[i]);
	
	}
	fmc_lock();

	return 0;
}

