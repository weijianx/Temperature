/*************************************************************************************
                                This file is the GD32F103 
						GD seiral mpu DMA congfig trans & receive data
*************************************************************************************/
//Include files
#include "Dmaconfig.h"


/*******************************************************************************
Name:   
Func:   
Para:   
Retn:   
*******************************************************************************/
void DMA_Rcv_TransDataParaConfig( PS_COMM_BUFPARA ppara )
{
	
	dma_parameter_struct dma_init_struct;
	
	//DMA0-CH3 interrupt config
	nvic_irq_enable(DMA_Channel3_4_IRQn, 0);
	
    /* enable DMA0 clock */
    rcu_periph_clock_enable( RCU_DMA );
    
    /* deinitialize DMA channel3(USART0 TxD) */
    dma_deinit(DMA_CH3);
    dma_struct_para_init(&dma_init_struct);

    dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_addr = (uint32_t)ppara->psnd;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = 0;
    dma_init_struct.periph_addr =  ((uint32_t)USART1_ADDR_DATA);	//USART1 receive address
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA_CH3, &dma_init_struct);
    
    /* deinitialize DMA channel4 (USART0 RxD) */
    dma_deinit(DMA_CH4);
    dma_struct_para_init(&dma_init_struct);

    dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr = (uint32_t)ppara->prcv;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = ppara->rcvsize;
    dma_init_struct.periph_addr = (uint32_t)&USART_RDATA(USART1);	//USART1_ADDR_DATA);
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA_CH4, &dma_init_struct);
  
    /* configure DMA mode */
    dma_circulation_disable(DMA_CH3);
    dma_memory_to_memory_disable(DMA_CH3);
    dma_circulation_enable(DMA_CH4);
    dma_memory_to_memory_disable(DMA_CH4);
    
    
    dma_interrupt_enable(DMA_CH3, DMA_INT_FTF);
    dma_interrupt_disable(DMA_CH4, DMA_INT_FTF);
    usart_dma_transmit_config(USART1, USART_DENT_ENABLE);	/* USART DMA1 enable for transmission */
	dma_channel_enable(DMA_CH4);
}


/*******************************************************************************
Name:   
Func:   
Para:   
Retn:   
*******************************************************************************/
void DMA_Channel3_4_IRQHandler( void )
{
	if (dma_interrupt_flag_get(DMA_CH3, DMA_FLAG_FTF) == SET)
	{
		dma_interrupt_flag_clear(DMA_CH3, DMA_INT_FLAG_G );
	}
	
	//Run TIMER3 
    timer_interrupt_enable( TIMER2, TIMER_INT_UP );
    timer_enable(TIMER2);
}

