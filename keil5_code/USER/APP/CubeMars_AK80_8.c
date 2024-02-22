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
	
    //ʹ�����ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB_GPIOx, ENABLE);//ʹ��PORTAʱ��	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB_CANx, ENABLE);//ʹ��CAN1ʱ��	
	
    //��ʼ��GPIO
	GPIO_InitStructure.GPIO_Pin = CAN_GPIO_Pin_1| CAN_GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(CAN_GPIOx, &GPIO_InitStructure);//��ʼ��PA11,PA12
	
	//���Ÿ���ӳ������
	GPIO_PinAFConfig(CAN_GPIOx,CAN_GPIO_PinSource_1,GPIO_AF_CANx); //GPIOA11����ΪCAN1
	GPIO_PinAFConfig(CAN_GPIOx,CAN_GPIO_PinSource_2,GPIO_AF_CANx); //GPIOA12����ΪCAN1
	
  	//CAN��Ԫ����
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
  	CAN_InitStructure.CAN_Mode= mode;	 //ģʽ���� 
  	CAN_InitStructure.CAN_SJW=tsjw;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
  	CAN_Init(CAN_Ak_num, &CAN_InitStructure);   // ��ʼ��CAN1 
    
	//���ù�����
 	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //������0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
  	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
		
	
	CAN_ITConfig(CAN_Ak_num,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    

  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
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
		mail_box=CAN_Transmit(CAN_Ak_num, &TxMessage); //CAN �ڷ��� TxMessage ����
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

/*********************************************** �ŷ�ģʽ***********************************************/
//ռ�ձ�ģʽ
void comm_can_set_duty(uint8_t controller_id, float duty) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(duty * 100000.0f), &send_index);
	comm_can_transmit_eid(controller_id |
							((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}

/*
������ģʽ
������ֵΪ int32 ���ͣ� ��ֵ-60000-60000 ����-60-60A
*/
void comm_can_set_current(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0f), &send_index);
	comm_can_transmit_eid(controller_id |
						((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

/*
����ɲ��ģʽ
ɲ��������ֵΪ int32 ���ͣ� ��ֵ 0-60000 ���� 0-60A
*/
void comm_can_set_cb(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0f), &send_index);
	comm_can_transmit_eid(controller_id |
						((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}

/*
�ٶȻ�ģʽ
�ٶ���ֵΪ int32 �ͣ� ��Χ-100000-100000 ����-100000-100000 ����ת��
*/
void comm_can_set_rpm(uint8_t controller_id, float rpm) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)rpm, &send_index);
	comm_can_transmit_eid(controller_id |
						((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}


/*
λ�û�ģʽ
λ��Ϊ int32 �ͣ� ��Χ-360000000-360000000 ����λ��-36000�� ~36000��
*/
void comm_can_set_pos(uint8_t controller_id, float pos) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(pos * 10000.0f), &send_index);
	comm_can_transmit_eid(controller_id |
						((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}

//����ԭ��ģʽ
void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode) {
	uint8_t buffer;
	buffer=set_origin_mode;	//����ָ��Ϊ uint8_t �ͣ� 0 ����������ʱԭ��(�ϵ�����)�� 1 ���������������(�����Զ�����);
	comm_can_transmit_eid(controller_id |
						((uint32_t) CAN_PACKET_SET_ORIGIN_HERE << 8), &buffer, 1);
}

/*
λ���ٶȻ�ģʽ
���У� λ��Ϊ int32 �ͣ� ��Χ-360000000~360000000 ��Ӧ-λ��-36000�� ~36000�� 
���У� �ٶ�Ϊ int16 �ͣ� ��Χ-32768~32767 ��Ӧ-327680~-327680 ����ת��
���У� ���ٶ�Ϊ int16 �ͣ� ��Χ 0~32767�� ��Ӧ 0~327670�� 1 ��λ���� 10 ����ת��/s2
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
/*********************************************** �ŷ�ģʽ***********************************************/



/***********************************************�˿�ģʽ***********************************************/

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
	msg[0] = p_int>>8; //λ�ø� 8
	msg[1] = p_int&0xFF; //λ�õ� 8
	msg[2] = v_int>>4; //�ٶȸ� 8 λ
	msg[3] = ((v_int&0xF)<<4)|(kp_int>>8); //�ٶȵ� 4 λ KP �� 4 λ
	msg[4] = kp_int&0xFF; //KP �� 8 λ
	msg[5] = kd_int>>4; //Kd �� 8 λ
	msg[6] = ((kd_int&0xF)<<4)|(t_int>>8); //KP �� 4 λŤ�ظ� 4 λ
	msg[7] = t_int&0xff; //Ť�ص� 8 λ
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
		mail_box=CAN_Transmit(CAN_Ak_num, &TxMessage); //CAN �ڷ��� TxMessage ����
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
/***********************************************�˿�ģʽ***********************************************/

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
//	AK80_8.cur_position = (float)( pos_int * 0.1f); //���λ��
//	AK80_8.cur_speed = (float)( spd_int * 10.0f);//����ٶ�
//	AK80_8.cur_curenty = (float) ( cur_int * 0.01f);//�������
//	AK80_8.cur_temperature = Rx1Message.Data[6] ;//����¶�
//	AK80_8.error_code = Rx1Message.Data[7] ;//���������
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
//		AK80_8.cur_position = (float)( pos_int * 0.1f); //���λ��
//		AK80_8.cur_speed = (float)( spd_int * 10.0f);//����ٶ�
//		AK80_8.cur_curenty = (float) ( cur_int * 0.01f);//�������
//		AK80_8.cur_temperature = Rx1Message.Data[6] ;//����¶�
//		AK80_8.error_code = Rx1Message.Data[7] ;//���������

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
	int id = Rx1Message.Data[0]; //���� ID ��
	int p_int = (Rx1Message.Data[1]<<8)|Rx1Message.Data[2]; //���λ������
	int v_int = (Rx1Message.Data[3]<<4)|(Rx1Message.Data[4]>>4); //����ٶ�����
	int i_int = ((Rx1Message.Data[4]&0xF)<<8)|Rx1Message.Data[5]; //���Ť������
	int T_int = Rx1Message.Data[6] ;
	/// convert ints to floats ///
	float p = uint_to_float_ak(p_int, P_MIN, P_MAX, 16);
	float v = uint_to_float_ak(v_int, V_MIN, V_MAX, 12);
	float i = uint_to_float_ak(i_int, T_MIN, T_MAX, 12);
	float T =T_int;
	if(id == 0x7f){
		AK80_8.cur_position = p; //���� ID �Ŷ�ȡ��Ӧ����
		AK80_8.cur_speed = v;
		AK80_8.cur_curenty = i;
		AK80_8.cur_temperature = T-40; //�¶ȷ�Χ-40~215
	}
	return;	
}


