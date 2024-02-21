#ifndef __CAN_H
#define __CAN_H

#include "stm32f4xx.h"                  // Device header
#include "sys.h"
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);
u8 CAN2_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);


#define CAN_num CAN1

//can1 参数为1
//can2 参数为0
#if 1
	#define RCC_APB_CANx RCC_APB1Periph_CAN1
	#define GPIO_AF_CANx GPIO_AF_CAN1
	#define CAN_GPIOx GPIOA
	#define RCC_AHB_GPIOx RCC_AHB1Periph_GPIOA
	
	#define CAN_GPIO_Pin_1 GPIO_Pin_11
	#define CAN_GPIO_Pin_2 GPIO_Pin_12
	
	#define CAN_GPIO_PinSource_1 GPIO_PinSource11
	#define CAN_GPIO_PinSource_2 GPIO_PinSource12
#else
	#define RCC_APB_CANx RCC_APB1Periph_CAN2
	#define GPIO_AF_CANx GPIO_AF_CAN2
	#define CAN_GPIOx GPIOB
	#define RCC_AHB_GPIOx RCC_AHB1Periph_GPIOB
	
	#define CAN_GPIO_Pin_1 GPIO_Pin_5
	#define CAN_GPIO_Pin_2 GPIO_Pin_6
	
	#define CAN_GPIO_PinSource_1 GPIO_PinSource5
	#define CAN_GPIO_PinSource_2 GPIO_PinSource6
#endif


#define get_motor_measure(ptr, rx_message)                                              \
{                                                                                       \
    if((ptr)->ecd - (ptr)->last_ecd > 4096) (ptr)->count-- ;                            \
		else if((ptr)->ecd - (ptr)->last_ecd < -4096 ) (ptr)->count ++ ;											\
    (ptr)->last_ecd = (ptr)->ecd;                                                       \
    (ptr)->ecd = (uint16_t)((rx_message).Data[0] << 8 | (rx_message).Data[1]);          \
    (ptr)->speed_rpm = (uint16_t)((rx_message).Data[2] << 8 |(rx_message).Data[3]);     \
    (ptr)->given_current = (uint16_t)((rx_message).Data[4] << 8 | (rx_message).Data[5]); \
    (ptr)->temperate = (rx_message).Data[6];                                             \
    (ptr)->all_ecd=(ptr)->count*8191+(ptr)->ecd;                                     \
}


#endif

