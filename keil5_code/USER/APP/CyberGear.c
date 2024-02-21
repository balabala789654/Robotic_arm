#include "CyberGear.h"
#include "delay.h"
/*

can通讯波特率为1Mbps  APB1时钟线频率为42Mhz 
CybeaGear_init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);

上电时， 会立即给电机当前位置设置成机械零位

在使用初始化函数之后， 使用CyberGear.cyberGear_control()函数即可控制电机
使用使用CyberGear.stop()函数即可停止电机

CyberGear.cyberGear_control()函数3个参数分别为 力矩(-12Nm~ 12Nm) 目标角度(-4pi~ 4pi) 目标角速度(-30rad/s~ 30rad/s)
例如：CyberGear.cyberGear_control(5.0, angle, 10.0);

*/

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -12.0f
#define T_MAX 12.0f


#define pi 3.14
#define CAN_CyberGear_num CAN2

int CyberGear_send_error_timers = 0;
_cyberGear CyberGear;
void CyberGear_set(void);

void CybeaGear_init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode){
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
  	CAN_Init(CAN_CyberGear_num, &CAN_InitStructure);   // 初始化CAN1 
    
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
		
	
	CAN_ITConfig(CAN_CyberGear_num,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    

  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannel = CAN2_TX_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
	
	
	CyberGear_set();
	
	return;
}

int float_to_uint(float x, float x_min, float x_max, int bits){
	float span = x_max - x_min;
	float offset = x_min;
	if(x > x_max) x=x_max;
	else if(x < x_min) x= x_min;
	
	return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

void data_frame_fill(CanTxMsg _CanTxMsg, uint16_t i1, uint16_t i2, uint16_t i3, uint16_t i4){
	_CanTxMsg.Data[0]=i1 >> 8;
	_CanTxMsg.Data[1]=i1;
	_CanTxMsg.Data[2]=i2 >> 8;
	_CanTxMsg.Data[3]=i2;
	_CanTxMsg.Data[4]=i3 >> 8;
	_CanTxMsg.Data[5]=i3;
	_CanTxMsg.Data[6]=i4 >> 8;
	_CanTxMsg.Data[7]=i4; 
	return;
}

void CyberGear_Get_Device_Id(uint8_t _master_id, uint8_t _id){
	CanTxMsg TxMessage;
	TxMessage.DLC=8;
	TxMessage.IDE=CAN_Id_Extended;
	TxMessage.RTR=CAN_RTR_Data;
	
	TxMessage.ExtId = (uint32_t)(_master_id << 8) | _id;
	
	data_frame_fill(TxMessage, 0, 0, 0, 0);
	
	uint8_t mail_box=0;
	mail_box = CAN_Transmit(CAN_CyberGear_num,&TxMessage);
	if((CAN_TransmitStatus(CAN_CyberGear_num, mail_box) == CAN_TxStatus_Failed)){
		CyberGear_send_error_timers++;
	}
	return;
}

void CyberGear_motion_control_mode(uint8_t _id, float torque, float _angle, float _angle_acc, float _kp, float _kd){
	CanTxMsg TxMessage;
	TxMessage.DLC=8;
	TxMessage.IDE=CAN_Id_Extended;
	TxMessage.RTR=CAN_RTR_Data;
	TxMessage.ExtId=(uint32_t)(1<<24) | (float_to_uint(torque, T_MIN, T_MAX, 16)<<8) | (_id);

//	uint16_t torque_ = (uint16_t)torque;
//	data_frame_fill(TxMessage, _angle, _angle_acc, _kp, _kd); //角度[-4pi~4pi] 角速度[-30rad/s~30rad/s] kp[0.0~500.0] kd[0.0~5.0] 	均为[0~65535]
	
	TxMessage.Data[0]=float_to_uint(_angle, P_MIN, P_MAX, 16) >> 8;
	TxMessage.Data[1]=float_to_uint(_angle, P_MIN, P_MAX, 16);
	TxMessage.Data[2]=float_to_uint(_angle_acc, V_MIN, V_MAX, 16) >> 8;
	TxMessage.Data[3]=float_to_uint(_angle_acc, V_MIN, V_MAX, 16);
	TxMessage.Data[4]=float_to_uint(_kp, KP_MIN, KP_MAX, 16) >> 8;
	TxMessage.Data[5]=float_to_uint(_kp, KP_MIN, KP_MAX, 16);
	TxMessage.Data[6]=float_to_uint(_kd, KD_MIN, KD_MAX, 16) >> 8;
	TxMessage.Data[7]=float_to_uint(_kd, KD_MIN, KD_MAX, 16); 
	
	uint8_t mail_box=0;
	mail_box = CAN_Transmit(CAN_CyberGear_num,&TxMessage);
	if((CAN_TransmitStatus(CAN_CyberGear_num, mail_box) == CAN_TxStatus_Failed)){
		CyberGear_send_error_timers++;
	}
	return;
}

void CyberGear_enable(uint8_t _master_id, uint8_t _id){
	CanTxMsg TxMessage;
	TxMessage.DLC=8;
	TxMessage.IDE=CAN_Id_Extended;
	TxMessage.RTR=CAN_RTR_Data;
	TxMessage.ExtId=(uint32_t)(3<<24) | (uint32_t)(_master_id<<8) | (_id);
	
	data_frame_fill(TxMessage, 0, 0, 0, 0);
	uint8_t mail_box=0;
	mail_box = CAN_Transmit(CAN_CyberGear_num,&TxMessage);
	if((CAN_TransmitStatus(CAN_CyberGear_num, mail_box) == CAN_TxStatus_Failed)){
		CyberGear_send_error_timers++;
	}
	return;	
}

void CyberGear_unable(uint8_t _master_id, uint8_t _id, char flag){
	CanTxMsg TxMessage;
	TxMessage.DLC=8;
	TxMessage.IDE=CAN_Id_Extended;
	TxMessage.RTR=CAN_RTR_Data;
	TxMessage.ExtId=(uint32_t)(4<<24) | (uint32_t)(_master_id<<8) | (_id);
	
	//正常运行时， data 区需清0；Byte[0]=1 时：清故障；
	//flag=1 有故障 flag=0 无故障
	if(flag) TxMessage.Data[0] = 1;
	else TxMessage.Data[0] = 0;
	
	for(int i=1; i<8; i++){
		TxMessage.Data[i] = 0;
	}
	
	uint8_t mail_box=0;
	mail_box = CAN_Transmit(CAN_CyberGear_num,&TxMessage);
	if((CAN_TransmitStatus(CAN_CyberGear_num, mail_box) == CAN_TxStatus_Failed)){
		CyberGear_send_error_timers++;
	}
	return;	
}

void CyberGear_set_zero_position(uint8_t _master_id, uint8_t _id){
	CanTxMsg TxMessage;
	TxMessage.DLC=8;
	TxMessage.IDE=CAN_Id_Extended;
	TxMessage.RTR=CAN_RTR_Data;
	TxMessage.ExtId=(uint32_t)(6<<24) | (uint32_t)(_master_id<<8) | (uint32_t)(_id);
	
	TxMessage.Data[0]=0x01;
	for(int i=1; i<8; i++){
		TxMessage.Data[i]=0;
	}

	uint8_t mail_box=0;
	mail_box = CAN_Transmit(CAN_CyberGear_num,&TxMessage);
	if((CAN_TransmitStatus(CAN_CyberGear_num, mail_box) == CAN_TxStatus_Failed)){
		CyberGear_send_error_timers++;
		CyberGear_set_zero_position(_master_id, _id);//如果发送失败 重复发送 直到成功
	}
	return;
	
}


void CyberGear_set_CAN_id(uint32_t _id){
	CanTxMsg TxMessage;
	TxMessage.DLC=8;
	TxMessage.IDE=CAN_Id_Extended;
	TxMessage.RTR=CAN_RTR_Data;
	TxMessage.ExtId=(uint32_t)_id;
	
	data_frame_fill(TxMessage, 0, 0, 0, 0);
	uint8_t mail_box=0;
	mail_box = CAN_Transmit(CAN_CyberGear_num,&TxMessage);
	if((CAN_TransmitStatus(CAN_CyberGear_num, mail_box) == CAN_TxStatus_Failed)){
		CyberGear_send_error_timers++;
	}
	return;	
}

void CyberGear_single_param_read(uint32_t _id){
	CanTxMsg TxMessage;
	TxMessage.DLC=8;
	TxMessage.IDE=CAN_Id_Extended;
	TxMessage.RTR=CAN_RTR_Data;
	TxMessage.ExtId=(uint32_t)_id;
	
	data_frame_fill(TxMessage, 0, 0, 0, 0);
	uint8_t mail_box=0;
	mail_box = CAN_Transmit(CAN_CyberGear_num,&TxMessage);
	if((CAN_TransmitStatus(CAN_CyberGear_num, mail_box) == CAN_TxStatus_Failed)){
		CyberGear_send_error_timers++;
	}
	return;
}

void CyberGear_single_param_write(uint32_t _id){
	CanTxMsg TxMessage;
	TxMessage.DLC=8;
	TxMessage.IDE=CAN_Id_Extended;
	TxMessage.RTR=CAN_RTR_Data;
	TxMessage.ExtId=(uint32_t)_id;
	
	data_frame_fill(TxMessage, 0, 0, 0, 0);
	uint8_t mail_box=0;
	mail_box = CAN_Transmit(CAN_CyberGear_num,&TxMessage);
	if((CAN_TransmitStatus(CAN_CyberGear_num, mail_box) == CAN_TxStatus_Failed)){
		CyberGear_send_error_timers++;
	}
	return;	
}


//void CAN1_RX0_IRQHandler(void)
//{
//	CanRxMsg Rx1Message;
//	CAN_Receive(CAN_num,CAN_FIFO0,&Rx1Message);
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
//		CyberGear.cur_angle = (float)(((Rx1Message.Data[0] << 8) | Rx1Message.Data[1])^0x8000)/32768*4*pi;
//		CyberGear.cur_angle_acc = (float)(((Rx1Message.Data[2] << 8) | Rx1Message.Data[3])^0x8000)/32768*30.0f;
//		CyberGear.cur_torque = (float)(((Rx1Message.Data[4] << 8) | Rx1Message.Data[5])^0x8000)/32768*12.0f;
//		CyberGear.cur_temperature = (float)((Rx1Message.Data[6] << 8) | Rx1Message.Data[7])/10.0f;
//	}
//	else if(((Rx1Message.ExtId & 0xFF) == master_can_id) && ((Rx1Message.ExtId & 0x1f000000) >> 24) == 0){
//		Rx1Message.ExtId = Rx1Message.ExtId & 0x00FFFF00;
//		CyberGear.id = (Rx1Message.ExtId & 0xFF00) >> 8;
//	}
//	return;
//}


void CyberGear_control(float _torque, float angle, float speed){
	CyberGear_motion_control_mode(0x7f, _torque, angle, speed, 10.0, 1.0);
	return;
}

void CyberGear_stop(void){
	char flag_error;
	if(CyberGear.Undervoltage||CyberGear.Overcurrent||CyberGear.Overtemperature||CyberGear.Encoder_error||CyberGear.HALL_error){
		flag_error=1;
	}
	CyberGear_unable(master_can_id, 0x7f, flag_error);
}

void CyberGear_set(void){
	CyberGear.cyberGear_control = CyberGear_control;
	CyberGear.cyberGear_stop = CyberGear_stop;
	
	CyberGear_Get_Device_Id(master_can_id, 0x7f);	
	CyberGear_enable(master_can_id, 0x7f);
	CyberGear_set_zero_position(master_can_id, 0x7f);
	
	return;
}

void CyberGear_rpm_0(void){
	CyberGear_control(5.0f, 0.0f, 0.0f);
}



