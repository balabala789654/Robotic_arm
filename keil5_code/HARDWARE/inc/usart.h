#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 

#define RC_NVIC 7

void usart1_init(u32 bound);
void usart3_Init(uint8_t* rx1_buf, uint8_t* rx2_buf, uint16_t dma_buf_num);
void RC_unable(void);
void RC_restart(uint16_t dma_buf_num);



#define MAX_interaction_byte 20
#endif


