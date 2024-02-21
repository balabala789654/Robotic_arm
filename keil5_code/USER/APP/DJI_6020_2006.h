#ifndef _DJI_6020_2006_H
#define _DJI_6020_2006_H

#include "pid.h"
#include "stm32f4xx.h"                  // Device header

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


typedef struct{
	PidType pid_speed_params;
	PidType pid_angle_params;
	
	uint16_t ecd;
	uint16_t initial_ecd;
	int16_t speed_rpm;
	int16_t given_current;
	int32_t  all_ecd;
	int32_t  count; 
	
	uint8_t temperate;
	int16_t last_ecd;
}_DJI_motor;

extern _DJI_motor DJI_6020;
extern _DJI_motor DJI_2006;

void DJI_6020_set(void);
void DJI_motor_control(float _param1, float _param2);
#endif

