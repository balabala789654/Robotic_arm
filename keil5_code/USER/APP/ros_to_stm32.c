#include "ros_to_stm32.h"
#include "stm32f4xx.h"                  // Device header

void ros_to_stm32_init(int bound){
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_USART3); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_USART3); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
	USART_Init(USART3, &USART_InitStructure); 

	USART_Cmd(USART3, ENABLE);   
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); //接受中断
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE); //空闲中断

	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	
}

union FloatToBytes
{
	float value;
	char bytes[4];
};
union FloatToBytes linear;
union FloatToBytes angle;

static char flag=0;
static char i=0;
uint8_t usart3_buffers[10];

float twist_linear, twist_angle;
void USART3_IRQHandler(void)
{	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		if(USART_ReceiveData(USART3) == frame_head) 
			flag=1;
		else if(USART_ReceiveData(USART3) == frame_end) 
		{
			usart3_buffers[i++] = USART_ReceiveData(USART3);
			flag=0;
		}
		if(flag) 
			usart3_buffers[i++] = USART_ReceiveData(USART3);
//		usart3_buffers[i++] = USART_ReceiveData(USART3);
		if(i == 10) {
			i = 0;
			
			linear.bytes[0]=usart3_buffers[1];
			linear.bytes[1]=usart3_buffers[2];
			linear.bytes[2]=usart3_buffers[3];
			linear.bytes[3]=usart3_buffers[4];

			angle.bytes[0]=usart3_buffers[5];
			angle.bytes[1]=usart3_buffers[6];
			angle.bytes[2]=usart3_buffers[7];
			angle.bytes[3]=usart3_buffers[8];
			
			twist_linear = linear.value;
			twist_angle = angle.value;
		}
		USART_ClearFlag(USART3, USART_FLAG_RXNE);
	}
	else if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET){
		USART_ReceiveData(USART3);	 
		USART_ClearFlag(USART3, USART_FLAG_IDLE);
	}
}

float*  ros_to_stm32_output(void){
	static float ret[2];
	ret[0] = twist_linear;
	ret[1] = twist_angle;
	return ret;
}




