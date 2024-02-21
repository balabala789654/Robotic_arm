#include "DJI_6020_2006.h"

_DJI_motor DJI_6020;
_DJI_motor DJI_2006;

#define DJI_6020_CANx CAN1
int DJI_can_send_error_times=0;

void DJI_6020_initial_set(void){
	DJI_6020.initial_ecd = DJI_6020.ecd;
	return;
}

void DJI_2006_initial_set(void){
	DJI_2006.initial_ecd = DJI_2006.ecd;
	return;
}

void DJI_2006_init(void){
	float pid_2006_params[2][6]={	5.0f, 0.0f, 0.1f, 5000.0f, 500.0f, 100.0f,
									0.1f, 0.0f, 0.1f, 1000.0f, 500.0f, 100.0f};
	
	PID_init(&DJI_2006.pid_speed_params, PID_POSITION, pid_2006_params[0][0],pid_2006_params[0][1],pid_2006_params[0][2],pid_2006_params[0][3],pid_2006_params[0][4],pid_2006_params[0][5]);
	PID_init(&DJI_2006.pid_angle_params, PID_POSITION, pid_2006_params[1][0],pid_2006_params[1][1],pid_2006_params[1][2],pid_2006_params[1][3],pid_2006_params[1][4],pid_2006_params[1][5]);
	
	DJI_2006_initial_set();
	DJI_2006.all_ecd=0;
	DJI_2006.count=0;
	
	return;
									
}
void DJI_6020_init(void){
	float pid_6020_params[2][6]={	20.0f, 0.0f, 0.1f, 15000.0f, 500.0f, 100.0f,
									1.0f, 0.0f, 0.1f, 1000.0f, 500.0f, 100.0f};
	
	PID_init(&DJI_6020.pid_speed_params, PID_POSITION, pid_6020_params[0][0],pid_6020_params[0][1],pid_6020_params[0][2],pid_6020_params[0][3],pid_6020_params[0][4],pid_6020_params[0][5]);
	PID_init(&DJI_6020.pid_angle_params, PID_POSITION, pid_6020_params[1][0],pid_6020_params[1][1],pid_6020_params[1][2],pid_6020_params[1][3],pid_6020_params[1][4],pid_6020_params[1][5]);
	
	DJI_6020_initial_set();
	DJI_6020.all_ecd=0;
	DJI_6020.count=0;
	return;
}

float DJI_6020_pid_output(float _input, float output){
	PID_calc(&DJI_6020.pid_angle_params, DJI_6020.all_ecd, _input);
	PID_calc(&DJI_6020.pid_speed_params, DJI_6020.speed_rpm, DJI_6020.pid_angle_params.out);
	return DJI_6020.pid_speed_params.out;
}


float DJI_2006_pid_output(float _input, float output){
	PID_calc(&DJI_2006.pid_angle_params, DJI_2006.all_ecd, _input);
	PID_calc(&DJI_2006.pid_speed_params, DJI_2006.speed_rpm, DJI_2006.pid_angle_params.out);
	return DJI_2006.pid_speed_params.out;
}

void DJI_motor_transmit(int16_t i1, int16_t i2){
	CanTxMsg TxMessage;
	
	TxMessage.StdId = 0x1ff;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.ExtId = 0;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
		
	TxMessage.Data[0] = i1>>8;
	TxMessage.Data[1] = i1;
	TxMessage.Data[2] = i2>>8;
	TxMessage.Data[3] = i2;
	TxMessage.Data[4] = 0;
	TxMessage.Data[5] = 0;
	TxMessage.Data[6] = 0;
	TxMessage.Data[7] = 0;
	
	uint8_t mail_box=0;
	if(!CAN_GetFlagStatus(DJI_6020_CANx, CAN_FLAG_FF0)){
		mail_box=CAN_Transmit(DJI_6020_CANx, &TxMessage); //CAN 口发送 TxMessage 数据
		if((CAN_TransmitStatus(DJI_6020_CANx, mail_box)==CAN_TxStatus_Failed)){
			DJI_can_send_error_times++;
		}
	}
	return;
}

static float param_6020;
static float param_2006;

void DJI_motor_control(float _param1, float _param2){
	float target_angle_6020;
	float target_angle_2006;
	
	target_angle_6020 = (_param1*8191.0f)/(2*3.14f) + DJI_6020.initial_ecd;
	target_angle_2006 = (_param2*8191.0f*36.0f)/(2*3.14f) + DJI_2006.initial_ecd;
	
	param_6020 = DJI_6020_pid_output(target_angle_6020, 0);
	param_2006 = DJI_2006_pid_output(target_angle_2006, 0);
	
	DJI_motor_transmit((param_6020), (param_2006));
	return;
}

void DJI_6020_set(void){
	DJI_6020_init();
	DJI_2006_init();
}





