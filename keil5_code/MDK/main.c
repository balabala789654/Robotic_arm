#include "stm32f4xx.h"                  // Device header
#include "arm_4.h"
#include "delay.h"
#include "can.h"
#include "timer.h"
#include "gpio.h"
#include "Remote_Control.h"

//debug
int speed_1 = 0;
int speed_2 = 0;
float pos_1 = 90.0f;
float pos_2 = -100.0f;
float pos_3 = 30.0f;
float pos_4 = 0.0f;

//世界坐标系
float x = 0.39;
float y = 0;
float z = 0.31;
float angle = 1.57f;

int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);	//设置系统中断优先级分组4
	delay_init(168);
//	TIM1_Int_Init(5000-1, 1680-1);
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
	CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
	
	delay_ms(3000);//等待电机
	TIM3_Int_Init(5000-1, 8400-1);

	gpio_Init();
//	remote_control_init();
	
	CAN_WakeUp(CAN1);
	CAN_WakeUp(CAN2);
	delay_ms(1);
	Arm_4_init();
	
	while(1){
//		Arm_4_debug(pos_1, pos_2, pos_3, pos_4);
		Arm_4_control(x, y, z, angle);
		delay_ms(1);
	}
}

