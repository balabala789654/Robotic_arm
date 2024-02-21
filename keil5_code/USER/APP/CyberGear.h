#ifndef _CYBEARGEAR_H
#define _CYBEARGEAR_H

#include "stm32f4xx.h"                  // Device header

//主机id
#define master_can_id 0x11


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


typedef struct{
	uint8_t id;
	char HALL_error;
	char Encoder_error;
	char Overtemperature;
	char Overcurrent;
	char Undervoltage;
	char mode;
	char error_flag;
	
	float cur_angle;
	float cur_angle_acc;
	float cur_torque;
	float cur_temperature;
	
	void (*cyberGear_control)(float _torque, float angle, float speed);
	void (*cyberGear_stop)(void);

}_cyberGear;

void CybeaGear_init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);
void CyberGear_set(void);
void CyberGear_stop(void);
void CyberGear_rpm_0(void);
void CyberGear_control(float _torque, float angle, float speed);
void CyberGear_Get_Device_Id(uint8_t _master_id, uint8_t _id);
extern _cyberGear CyberGear;

#endif



