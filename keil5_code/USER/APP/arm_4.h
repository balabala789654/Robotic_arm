#ifndef _ARM_4_H
#define _ARM_4_H

#define pi 3.14
#include "math.h"
#include "CyberGear.h"
#include "CubeMars_AK80_8.h"
#include "Low_pass.h"
#include "DJI_6020_2006.h"

typedef struct{
	float l1,l2,l3;
	float theta_ori, theta_1, theta_2, theta_3;
	float alpha;
	float beta;
	
	float AK80_8_int_angle;
	float CyberGear_init_angle;
	float DJI_2006_init_angle;
	float joint_position[4];
	float end_effector_position[4];
}_arm;

extern _arm arm;
void Arm_4_init(void);
void Arm_4_stop(void);
void Arm_4_debug(float _param_1, float _param_2, float _param_3, float _param_4);
void Arm_4_control(float _x, float _y, float _z, float _angle);
float rad_to_deg(float _input);
float deg_to_rad(float _inputs);
#endif

