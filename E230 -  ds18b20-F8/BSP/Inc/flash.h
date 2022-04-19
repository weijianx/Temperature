/*!
    \file  flash_access.h 
    \brief the header file of flash_access.c

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

#ifndef FLASH_H
#define FLASH_H

#include "gd32e230.h"
#include "stdbool.h"
#include "string.h"

//#define	start_adder			0x0800F810

#define	start_adder			0x08003000

#define fmc_last_page		0x0800F800

#define SOWAYAPP_FLASH_PAGES        32			//64 - 8 = 56
#define SOWAYFLASH_PAGE_SIZE        1024
#define SOWAYFLASH_SRCPARA_ADDR     0x08003000  //Backup parameter 2K bytes
#define SOWAYFLASH_OP_BASE_ADDR     0x08003800  //System parameter 2K bytes
#define MAGNET_PARAMETER_ADDR       0x08003A00	//Magnet parameter address
#define SOWAYAPP_START_ADDR         0x08004000  //Soway application start address
#define CHIPFLASH_MAXIMUM_ADDR      0x0801FFFF  //According chip type modify
#define CHIPFLASH_EDGE_ADDR         0x08020000

#define FLASH_LAST_PAGE_ADDR		              0x0800F800					// Flash���һ�������׵�ַ
#define APPLICATION_ADDRESS                       (uint32_t)0x08004000      /* Start user code address: ADDR_FLASH_PAGE_8 */

#define FLASH_PAGE_SIZE                 0x00000800U    /*!< FLASHҳ���С��2K�ֽ� */



#define USER_FLASH_PAGE_SIZE          FLASH_PAGE_SIZE

#define FLASH_BASE_ADDR						 0x08003800					        // Flash�洢�����Ļ���ַ
#define COMMON_OFFSET						 0x00000000							// ͨ�ò����洢��ַƫ����
#define COMMON_BASE_ADDR					(FLASH_BASE_ADDR + COMMON_OFFSET)	// FLASH������ʼ��ַ
#define COMMON_UPGRADE_ADDR					(COMMON_BASE_ADDR + 0)				// ͨ�ò���ϵͳ������־��ַ
#define COMMON_SLAVE_ADDR					(COMMON_BASE_ADDR + 1)				// ͨ�ò����ӻ���ַ��ַ
#define COMMON_BAUD_ADDR					(COMMON_BASE_ADDR + 2)				// ͨ�ò��������ʵ�ַ
#define COMMON_PARITY_ADDR					(COMMON_BASE_ADDR + 3)				// ͨ�ò�����żУ���ַ
#define COMMON_OUTPUTMOD_ADDR				(COMMON_BASE_ADDR + 4)				// ͨ�ò������ģʽ��ַ
#define COMMON_RESERVED_ADDR				(COMMON_BASE_ADDR + 5)				// ͨ�ò���������ַ

// ����ʹ��
#define UPGRADE_DISABLE							0x00							// ������־ʧ��(Ĭ��ֵ)
#define UPGRADE_ENABLE							0xFF							// ������־ʹ��


#define EN_INT                  	__enable_irq();				// ϵͳ��ȫ���ж�  
#define DIS_INT                 	__disable_irq();			// ϵͳ��ȫ���ж�



uint8_t Flash_Read_OneByte(uint32_t RWAddr);

void flash_read_multi(uint32_t readAdder, uint8_t *readBuf, uint16_t readLen);

uint32_t  flash_read_MultiBytes(uint32_t read_addr, uint8_t *pBuf, uint16_t len);

uint8_t Flash_Write_OneByte(uint32_t RWAddr, uint8_t WrData);

uint8_t Flash_Write_MultiBytes(uint32_t RWAddr, uint8_t const *pWrbuf, uint16_t Wrlen);
//uint8_t Flash_Write_MultiBytes32(uint32_t RWAddr, uint8_t const *pWrbuf, uint16_t Wrlen);

uint32_t flash_write_word (uint32_t write_addr, uint32_t *pBuf, uint16_t wreLen);

uint8_t fmc_erase_for_app(uint32_t startAdder);

uint8_t fmc_erase_a_page(uint32_t startAdder);
uint8_t Flash_Write_32Bytes(uint32_t RWAddr, uint32_t data);

bool GDFLASHOP_EraseSpecifiedFlashPages( uint32_t startaddr, uint16_t npages );

#endif /* FLASH_ACCESS_H */



