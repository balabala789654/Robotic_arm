#include "arm_4.h"
#include "delay.h"

/**
	该机械臂为四轴机械臂
	参数说明：
			arm.theta_ori	为第一关节旋转角度
			arm.theta_1		为第二关节旋转角度
			arm.theta_2		为第三关节旋转角度
			arm.theta_3		为第四关节旋转角度
			
			除了第一关节，其余关节旋转轴都相同方向
			
			
**/

_arm arm;

float rad_to_deg(float _input){
	return (_input/3.14f)*180.0f;
}

float deg_to_rad(float _inputs){
	return (_inputs/180.0f)*3.14f;
}

void joint_position_cal(float _x, float _y, float _z, float _angle){
	if(_angle == 1.57f){
		arm.joint_position[0] = _x;
		arm.joint_position[1] = _y;
		arm.joint_position[2] = _z + arm.l3;
		arm.joint_position[3] = _angle;
		return;
	}
	else if(_angle == 0){
		float gamma = atan2(_y, _x);
		float delta_x = cosf(gamma) * arm.l3;
		float delta_y = sinf(gamma) * arm.l3;
		
		arm.joint_position[0] = _x - delta_x;
		arm.joint_position[1] = _y - delta_y;	
		arm.joint_position[2] = _z;
		arm.joint_position[3] = _angle;
		return;
	}
	else return;
}

void alpha_cal(float _x, float _y, float _z, float* _alpha){
	float hypotenuse = sqrtf(powf(_x, 2)+powf(_y, 2));
	*_alpha = atanf(_z/hypotenuse);
	return;
}

char beta_cal(float _x, float _y, float _z, float* _beta){
	float hypotenuse = sqrtf(powf(_x, 2)+powf(_y, 2)+powf(_z, 2));
	float var = (float)(powf(hypotenuse,2)+powf(arm.l1,2)-powf(arm.l2,2))/(2*hypotenuse*arm.l1);
	if(var > 1){
		return 0;
	}
	else {
		*_beta = acosf(var);
		return 1;
	}
}

char theta_cal(float _x, float _y, float _z){
	float var;
	float hypotenuse = sqrtf(powf(_x, 2)+powf(_y, 2)+powf(_z, 2));
	
	alpha_cal(_x, _y, _z, &arm.alpha);
	if(beta_cal(_x, _y, _z, &arm.beta)){
		var = (powf(arm.l1,2)+powf(arm.l2,2)-powf(hypotenuse,2))/(2*arm.l1*arm.l2);
		if(var > 1){
			return 0;
		}
		else {
			arm.theta_ori = atan2f(_y, _x);
			arm.theta_1 = pi/2 - (arm.alpha + arm.beta);
			arm.theta_2 = pi/2 - acosf(var);
			arm.theta_3 = arm.joint_position[3] - arm.theta_1 - arm.theta_2;
			return 1;
		}
	}
	else {
		return 0;
	}
}


void Arm_4_control(float _x, float _y, float _z, float _angle){
	arm.end_effector_position[0] = _x;
	arm.end_effector_position[1] = _y;
	arm.end_effector_position[2] = _z;
	arm.end_effector_position[3] = _angle;
	
	joint_position_cal(arm.end_effector_position[0], arm.end_effector_position[1], arm.end_effector_position[2], arm.end_effector_position[3]);
	theta_cal(arm.joint_position[0], arm.joint_position[1], arm.joint_position[2]);
	
	
	Ak80_8_control_servo(arm.AK80_8_int_angle + rad_to_deg(arm.theta_1));
	CyberGear_control(5.0f, arm.CyberGear_init_angle + arm.theta_2, 0.0f);
	DJI_motor_control(arm.theta_ori, arm.DJI_2006_init_angle + arm.theta_3);
}

void Arm_4_stop(void){
	//CyberGear_rpm_0();
	//CyberGear_stop();
	AK80_8_stop();
}


float param_1, param_2;
void Arm_4_debug(float _param_1, float _param_2, float _param_3, float _param_4){
	
	Low_pass(_param_1, &param_1, 0);
	Low_pass(_param_2, &param_2, 1);
	
//	CyberGear_control(5.0f, deg_to_rad(param_2), 0.0f);
//	Ak80_8_coltrol(deg_to_rad(_param_1));
//	Ak80_8_control_servo(_param_1);
	DJI_motor_control(deg_to_rad(_param_3), deg_to_rad(_param_4));
	return;
}

void Arm_4_init(void){
	arm.l1 = 0.410f;
	arm.l2 = 0.390f;
	arm.l3 = 0.100f;
	arm.alpha = 0;
	arm.beta = 0;
	arm.theta_1 = 0;
	arm.theta_2 = 0;
	arm.theta_3 = 0;
	arm.theta_ori = 0;
	arm.AK80_8_int_angle = 100.0f;
	arm.CyberGear_init_angle = deg_to_rad(-150.0f);
	arm.DJI_2006_init_angle = deg_to_rad(120.0f);
	
	AK80_8_set();
	CyberGear_set();
	DJI_6020_set();
}


void CAN1_RX0_IRQHandler(void){
	CanRxMsg Rx1Message;
	CAN_Receive(CAN1,CAN_FIFO0,&Rx1Message);
	if(((Rx1Message.ExtId & 0xFF) == master_can_id) && ((Rx1Message.ExtId & 0x1f000000) >> 24) == 2){
		
		Rx1Message.ExtId = Rx1Message.ExtId & 0x00FFFF00;
		
		CyberGear.id = (Rx1Message.ExtId & 0xFF00) >> 8;
		CyberGear.Undervoltage = (Rx1Message.ExtId & 0x10000) >> 16;
		CyberGear.Overcurrent = (Rx1Message.ExtId & 0x20000) >> 17;
		CyberGear.Overtemperature = (Rx1Message.ExtId & 0x30000) >> 18;
		CyberGear.Encoder_error = (Rx1Message.ExtId & 0x40000) >> 19;
		CyberGear.HALL_error = (Rx1Message.ExtId & 0x100000) >> 20;
		CyberGear.mode = (Rx1Message.ExtId >> 22);
		
		CyberGear.cur_angle = (float)(((Rx1Message.Data[0] << 8) | Rx1Message.Data[1])^0x8000)/32768*4*3.14f;
		CyberGear.cur_angle_acc = (float)(((Rx1Message.Data[2] << 8) | Rx1Message.Data[3])^0x8000)/32768*30.0f;
		CyberGear.cur_torque = (float)(((Rx1Message.Data[4] << 8) | Rx1Message.Data[5])^0x8000)/32768*12.0f;
		CyberGear.cur_temperature = (float)((Rx1Message.Data[6] << 8) | Rx1Message.Data[7])/10.0f;
	}
	
	switch(Rx1Message.StdId)
	{
		case 0x205: get_motor_measure(&DJI_6020, Rx1Message);break;
		case 0x206: get_motor_measure(&DJI_2006, Rx1Message);break;		
	}
//		int16_t pos_int = (Rx1Message).Data[0] << 8 | (Rx1Message).Data[1];
//		int16_t spd_int = (Rx1Message).Data[2] << 8 | (Rx1Message).Data[3];
//		int16_t cur_int = (Rx1Message).Data[4] << 8 | (Rx1Message).Data[5];
//		AK80_8.cur_position = (float)( pos_int * 0.1f); //电机位置
//		AK80_8.cur_speed = (float)( spd_int * 10.0f);//电机速度
//		AK80_8.cur_curenty = (float) ( cur_int * 0.01f);//电机电流
//		AK80_8.cur_temperature = Rx1Message.Data[6] ;//电机温度
//		AK80_8.error_code = Rx1Message.Data[7] ;//电机故障码
	
	return;	
	
	
}

void CAN1_TX_IRQHandler(void){
	int i;
	i++;
}

void CAN2_TX_IRQHandler(void){
	int i;
	i++;
}
