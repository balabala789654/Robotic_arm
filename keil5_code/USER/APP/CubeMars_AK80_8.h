#ifndef _CUBEMARS_AK80_8_H
#define _CUBEMARS_AK80_8_H

#include "stm32f4xx.h"                  // Device header

#define motor_id 0x7f


//can1 ����Ϊ1
//can2 ����Ϊ0
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

typedef enum {
	CAN_PACKET_SET_DUTY = 0, //ռ�ձ�ģʽ
	CAN_PACKET_SET_CURRENT, //������ģʽ
	CAN_PACKET_SET_CURRENT_BRAKE, // ����ɲ��ģʽ
	CAN_PACKET_SET_RPM, // ת��ģʽ
	CAN_PACKET_SET_POS, // λ��ģʽ
	CAN_PACKET_SET_ORIGIN_HERE, //����ԭ��ģʽ
	CAN_PACKET_SET_POS_SPD, //λ���ٶȻ�ģʽ
} CAN_PACKET_ID;

typedef struct{
	float cur_position;
	float cur_speed;
	float cur_curenty;
	int cur_temperature;
	int error_code;
}_ak80_8;


void AK80_8_init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);
void Ak80_8_coltrol(float _param);
void AK80_8_set(void);
void AK80_8_stop(void);
void Ak80_8_control_servo(float _param);
extern _ak80_8 AK80_8;

#endif



