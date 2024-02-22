#include "CubeMars_AK80_8.h"
#include "delay.h"
#include "math.h"

#define CAN_Ak_num CAN2

#define  P_MIN -12.5f
#define  P_MAX 12.5f
#define  V_MIN -30.0f
#define  V_MAX 30.0f
#define  T_MIN -18.0f
#define  T_MAX 18.0f
#define  Kp_MIN 0
#define  Kp_MAX 500.0f
#define  Kd_MIN 0
#define  Kd_MAX 5.0f
#define  Test_Pos 0.0f
	
_ak80_8 AK80_8;
int AK80_8_send_error_times=0;

void AK80_8_set(void);

void AK80_8_init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode){
	GPIO_InitTypeDef GPIO_InitStructure; 
	CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
   	NVIC_InitTypeDef  NVIC_InitStructure;
	
    //使能相关时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB_GPIOx, ENABLE);//使能PORTA时钟	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB_CANx, ENABLE);//使能CAN1时钟	
	
    //初始化GPIO
	GPIO_InitStructure.GPIO_Pin = CAN_GPIO_Pin_1| CAN_GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(CAN_GPIOx, &GPIO_InitStructure);//初始化PA11,PA12
	
	//引脚复用映射配置
	GPIO_PinAFConfig(CAN_GPIOx,CAN_GPIO_PinSource_1,GPIO_AF_CANx); //GPIOA11复用为CAN1
	GPIO_PinAFConfig(CAN_GPIOx,CAN_GPIO_PinSource_2,GPIO_AF_CANx); //GPIOA12复用为CAN1
	
  	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode= mode;	 //模式设置 
  	CAN_InitStructure.CAN_SJW=tsjw;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1	
  	CAN_Init(CAN_Ak_num, &CAN_InitStructure);   // 初始化CAN1 
    
	//配置过滤器
 	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
		
	
	CAN_ITConfig(CAN_Ak_num,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    

  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
	
	AK80_8_set();
	return;
}

void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
	if (len > 8) len = 8;
	CanTxMsg TxMessage;
	
	TxMessage.StdId = 0;
	TxMessage.IDE = CAN_Id_Extended;
	TxMessage.ExtId = id;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = len;
	for(int j=0; j<8; j++)
		TxMessage.Data[j]=0;
	for(int i=0;i<len;i++)
		TxMessage.Data[i]=data[i];
	
	uint8_t mail_box=0;
	
	if(!CAN_GetFlagStatus(CAN_Ak_num, CAN_FLAG_FF1)){
		mail_box=CAN_Transmit(CAN_Ak_num, &TxMessage); //CAN 口发送 TxMessage 数据
		if((CAN_TransmitStatus(CAN_Ak_num, mail_box)==CAN_TxStatus_Failed)){
			AK80_8_send_error_times++;
			comm_can_transmit_eid(id, data, len);
		}
	}
}

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
} 

void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t *index) {
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

/*********************************************** 伺服模式***********************************************/
//占空比模式
void comm_can_set_duty(uint8_t controller_id, float duty) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(duty * 100000.0f), &send_index);
	comm_can_transmit_eid(controller_id |
							((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}

/*
电流环模式
电流数值为 int32 类型， 数值-60000-60000 代表-60-60A
*/
void comm_can_set_current(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0f), &send_index);
	comm_can_transmit_eid(controller_id |
						((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

/*
电流刹车模式
刹车电流数值为 int32 类型， 数值 0-60000 代表 0-60A
*/
void comm_can_set_cb(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0f), &send_index);
	comm_can_transmit_eid(controller_id |
						((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}

/*
速度环模式
速度数值为 int32 型， 范围-100000-100000 代表-100000-100000 电气转速
*/
void comm_can_set_rpm(uint8_t controller_id, float rpm) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)rpm, &send_index);
	comm_can_transmit_eid(controller_id |
						((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}


/*
位置环模式
位置为 int32 型， 范围-360000000-360000000 代表位置-36000° ~36000°
*/
void comm_can_set_pos(uint8_t controller_id, float pos) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(pos * 10000.0f), &send_index);
	comm_can_transmit_eid(controller_id |
						((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}

//设置原点模式
void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode) {
	uint8_t buffer;
	buffer=set_origin_mode;	//设置指令为 uint8_t 型， 0 代表设置临时原点(断电消除)， 1 代表设置永久零点(参数自动保存);
	comm_can_transmit_eid(controller_id |
						((uint32_t) CAN_PACKET_SET_ORIGIN_HERE << 8), &buffer, 1);
}

/*
位置速度环模式
其中， 位置为 int32 型， 范围-360000000~360000000 对应-位置-36000° ~36000° 
其中， 速度为 int16 型， 范围-32768~32767 对应-327680~-327680 电气转速
其中， 加速度为 int16 型， 范围 0~32767， 对应 0~327670， 1 单位等于 10 电气转速/s2
*/
void comm_can_set_pos_spd(uint8_t controller_id, float pos,int16_t spd, int16_t RPA ) {
	int32_t send_index = 0;
	int16_t send_index1 = 4;
	uint8_t buffer[8];
	buffer_append_int32(buffer, (int32_t)(pos * 10000.0f), &send_index);
	buffer_append_int16(buffer,spd, &send_index1);
	buffer_append_int16(buffer,RPA, &send_index1);
	comm_can_transmit_eid(controller_id |
						((uint32_t)CAN_PACKET_SET_POS_SPD << 8), buffer, send_index1);
}
/*********************************************** 伺服模式***********************************************/



/***********************************************运控模式***********************************************/

float uint_to_float_ak(int x_int, float x_min, float x_max, int bits){
/// converts unsigned int to float, given range and number of bits ///
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

int float_to_uint_ak(float x, float x_min, float x_max, unsigned int bits){
/// Converts a float to an unsigned int, given range and number of bits ///
	float span = x_max - x_min;
	float offset = x_min;
	if(x > x_max) x=x_max;
	else if(x < x_min) x= x_min;
	
	return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

void pack_cmd(uint8_t* msg, float p_des, float v_des, float kp, float kd, float t_ff){
/// limit data to be within bounds ///

	p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
	v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
	kp = fminf(fmaxf(Kp_MIN, kp), Kp_MAX);
	kd = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
	t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);
	/// convert floats to unsigned ints ///
	int p_int = float_to_uint_ak(p_des, P_MIN, P_MAX, 16);
	int v_int = float_to_uint_ak(v_des, V_MIN, V_MAX, 12);
	int kp_int = float_to_uint_ak(kp, Kp_MIN, Kp_MAX, 12);
	int kd_int = float_to_uint_ak(kd, Kd_MIN, Kd_MAX, 12);
	int t_int = float_to_uint_ak(t_ff, T_MIN, T_MAX, 12);
	
	/// pack ints into the can buffer ///
	msg[0] = p_int>>8; //位置高 8
	msg[1] = p_int&0xFF; //位置低 8
	msg[2] = v_int>>4; //速度高 8 位
	msg[3] = ((v_int&0xF)<<4)|(kp_int>>8); //速度低 4 位 KP 高 4 位
	msg[4] = kp_int&0xFF; //KP 低 8 位
	msg[5] = kd_int>>4; //Kd 高 8 位
	msg[6] = ((kd_int&0xF)<<4)|(t_int>>8); //KP 低 4 位扭矩高 4 位
	msg[7] = t_int&0xff; //扭矩低 8 位
}

void Ak80_8_can_transmit(const uint8_t *data){
	CanTxMsg TxMessage;
	
	TxMessage.StdId = 0x7f;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.ExtId = 0;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	for(int j=0; j<8; j++)
		TxMessage.Data[j]=0;
	
	for(int i=0;i<8;i++)
		TxMessage.Data[i]=data[i];
	
	uint8_t mail_box=0;
	if(!CAN_GetFlagStatus(CAN_Ak_num, CAN_FLAG_FF1)){
		mail_box=CAN_Transmit(CAN_Ak_num, &TxMessage); //CAN 口发送 TxMessage 数据
		if((CAN_TransmitStatus(CAN_Ak_num, mail_box)==CAN_TxStatus_Failed)){
			AK80_8_send_error_times++;
		}
	}
}

void Ak80_8_origin_0(void){
	static uint8_t transmit_1[8];
	for(int i=0; i<7; i++){
		transmit_1[i] = 0xff;
	}
	transmit_1[7] = 0xfe;
	
	Ak80_8_can_transmit(transmit_1);
}

void Ak80_8_enable(void){
	static uint8_t transmit_2[8];
	for(int i=0; i<7; i++){
		transmit_2[i] = 0xff;
	}
	transmit_2[7] = 0xfc;
	
	Ak80_8_can_transmit(transmit_2);
	
}

void Ak80_8_unable(void){
	static uint8_t transmit_2[8];
	for(int i=0; i<7; i++){
		transmit_2[i] = 0xff;
	}
	transmit_2[7] = 0xfd;
	
	Ak80_8_can_transmit(transmit_2);
	
}
/***********************************************运控模式***********************************************/

void AK80_8_set(void){
//	Ak80_8_enable();
//	Ak80_8_origin_0();
	
	comm_can_set_origin(0x7f, 0);
}

float rpm = 5.0f;
float kp = 90.0f;
float kd = 1.5f;
float t_ff = -10.0f;

void Ak80_8_coltrol(float _param){
	static uint8_t transmit_3[8];
	pack_cmd(transmit_3, _param, rpm, kp, kd, t_ff);
	Ak80_8_can_transmit(transmit_3);
}

void AK80_8_stop(void){
	Ak80_8_unable();
}

void Ak80_8_control_servo(float _param){
	
	comm_can_set_pos_spd(0x7f, _param, 1000, 20);
//	comm_can_set_pos(0x7f, _param);
}


//void CAN1_RX0_IRQHandler(void){
//	CanRxMsg Rx1Message;
//	CAN_Receive(CAN_num,CAN_FIFO0,&Rx1Message);
//	int16_t pos_int = (Rx1Message).Data[0] << 8 | (Rx1Message).Data[1];
//	int16_t spd_int = (Rx1Message).Data[2] << 8 | (Rx1Message).Data[3];
//	int16_t cur_int = (Rx1Message).Data[4] << 8 | (Rx1Message).Data[5];
//	AK80_8.cur_position = (float)( pos_int * 0.1f); //电机位置
//	AK80_8.cur_speed = (float)( spd_int * 10.0f);//电机速度
//	AK80_8.cur_curenty = (float) ( cur_int * 0.01f);//电机电流
//	AK80_8.cur_temperature = Rx1Message.Data[6] ;//电机温度
//	AK80_8.error_code = Rx1Message.Data[7] ;//电机故障码
//}


//void CAN2_TX_IRQHandler(void){
//	int i;
//	i++;
//}

//void AK80_8_set(void){
//	comm_can_set_origin(motor_id, 0);
//}


//void AK80_8_control(float _param){
//	comm_can_set_pos(motor_id, _param);
//}

//void AK80_8_stop(void){
//	comm_can_set_rpm(motor_id, 0.0f);
//}

void CAN2_RX1_IRQHandler(void){
	CanRxMsg Rx1Message;
	CAN_Receive(CAN2,CAN_FIFO1,&Rx1Message);
	
//	if(((Rx1Message.ExtId & 0xFF) == master_can_id) && ((Rx1Message.ExtId & 0x1f000000) >> 24) == 2){
//		
//		Rx1Message.ExtId = Rx1Message.ExtId & 0x00FFFF00;
//		
//		CyberGear.id = (Rx1Message.ExtId & 0xFF00) >> 8;
//		CyberGear.Undervoltage = (Rx1Message.ExtId & 0x10000) >> 16;
//		CyberGear.Overcurrent = (Rx1Message.ExtId & 0x20000) >> 17;
//		CyberGear.Overtemperature = (Rx1Message.ExtId & 0x30000) >> 18;
//		CyberGear.Encoder_error = (Rx1Message.ExtId & 0x40000) >> 19;
//		CyberGear.HALL_error = (Rx1Message.ExtId & 0x100000) >> 20;
//		CyberGear.mode = (Rx1Message.ExtId >> 22);
//		
//		CyberGear.cur_angle = (float)(((Rx1Message.Data[0] << 8) | Rx1Message.Data[1])^0x8000)/32768*4*3.14f;
//		CyberGear.cur_angle_acc = (float)(((Rx1Message.Data[2] << 8) | Rx1Message.Data[3])^0x8000)/32768*30.0f;
//		CyberGear.cur_torque = (float)(((Rx1Message.Data[4] << 8) | Rx1Message.Data[5])^0x8000)/32768*12.0f;
//		CyberGear.cur_temperature = (float)((Rx1Message.Data[6] << 8) | Rx1Message.Data[7])/10.0f;
//	}
//	else if(((Rx1Message.ExtId & 0xFF) == master_can_id) && ((Rx1Message.ExtId & 0x1f000000) >> 24) == 0){
//		Rx1Message.ExtId = Rx1Message.ExtId & 0x00FFFF00;
//		CyberGear.id = (Rx1Message.ExtId & 0xFF00) >> 8;
//	}
//	else {
//		int16_t pos_int = (Rx1Message).Data[0] << 8 | (Rx1Message).Data[1];
//		int16_t spd_int = (Rx1Message).Data[2] << 8 | (Rx1Message).Data[3];
//		int16_t cur_int = (Rx1Message).Data[4] << 8 | (Rx1Message).Data[5];
//		AK80_8.cur_position = (float)( pos_int * 0.1f); //电机位置
//		AK80_8.cur_speed = (float)( spd_int * 10.0f);//电机速度
//		AK80_8.cur_curenty = (float) ( cur_int * 0.01f);//电机电流
//		AK80_8.cur_temperature = Rx1Message.Data[6] ;//电机温度
//		AK80_8.error_code = Rx1Message.Data[7] ;//电机故障码

//	}

//	if(((Rx1Message.ExtId & 0xFF) == master_can_id) && ((Rx1Message.ExtId & 0x1f000000) >> 24) == 2){
//		
//		Rx1Message.ExtId = Rx1Message.ExtId & 0x00FFFF00;
//		
//		CyberGear.id = (Rx1Message.ExtId & 0xFF00) >> 8;
//		CyberGear.Undervoltage = (Rx1Message.ExtId & 0x10000) >> 16;
//		CyberGear.Overcurrent = (Rx1Message.ExtId & 0x20000) >> 17;
//		CyberGear.Overtemperature = (Rx1Message.ExtId & 0x30000) >> 18;
//		CyberGear.Encoder_error = (Rx1Message.ExtId & 0x40000) >> 19;
//		CyberGear.HALL_error = (Rx1Message.ExtId & 0x100000) >> 20;
//		CyberGear.mode = (Rx1Message.ExtId >> 22);
//		
//		CyberGear.cur_angle = (float)(((Rx1Message.Data[0] << 8) | Rx1Message.Data[1])^0x8000)/32768*4*3.14f;
//		CyberGear.cur_angle_acc = (float)(((Rx1Message.Data[2] << 8) | Rx1Message.Data[3])^0x8000)/32768*30.0f;
//		CyberGear.cur_torque = (float)(((Rx1Message.Data[4] << 8) | Rx1Message.Data[5])^0x8000)/32768*12.0f;
//		CyberGear.cur_temperature = (float)((Rx1Message.Data[6] << 8) | Rx1Message.Data[7])/10.0f;
//	}
//	else if(((Rx1Message.ExtId & 0xFF) == master_can_id) && ((Rx1Message.ExtId & 0x1f000000) >> 24) == 0){
//		Rx1Message.ExtId = Rx1Message.ExtId & 0x00FFFF00;
//		CyberGear.id = (Rx1Message.ExtId & 0xFF00) >> 8;
//	}
/// unpack ints from can buffer ///
	int id = Rx1Message.Data[0]; //驱动 ID 号
	int p_int = (Rx1Message.Data[1]<<8)|Rx1Message.Data[2]; //电机位置数据
	int v_int = (Rx1Message.Data[3]<<4)|(Rx1Message.Data[4]>>4); //电机速度数据
	int i_int = ((Rx1Message.Data[4]&0xF)<<8)|Rx1Message.Data[5]; //电机扭矩数据
	int T_int = Rx1Message.Data[6] ;
	/// convert ints to floats ///
	float p = uint_to_float_ak(p_int, P_MIN, P_MAX, 16);
	float v = uint_to_float_ak(v_int, V_MIN, V_MAX, 12);
	float i = uint_to_float_ak(i_int, T_MIN, T_MAX, 12);
	float T =T_int;
	if(id == 0x7f){
		AK80_8.cur_position = p; //根据 ID 号读取对应数据
		AK80_8.cur_speed = v;
		AK80_8.cur_curenty = i;
		AK80_8.cur_temperature = T-40; //温度范围-40~215
	}
	return;	
}


