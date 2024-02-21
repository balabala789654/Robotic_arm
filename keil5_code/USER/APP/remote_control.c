#include "Remote_Control.h"
//#include "main.h"
#include "stm32f4xx.h"                  // Device header

RC_ctrl_t rc_ctrl;// static

//遥控器处理函数
static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t SBUS_rx_buf[2][SBUS_RX_BUF_NUM];

//初始化DMA，串口3
void remote_control_init(void)
{
    usart3_Init(SBUS_rx_buf[0], SBUS_rx_buf[1], SBUS_RX_BUF_NUM);
}

//串口中断   大疆官方是USART2
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(rc_USARTx, USART_IT_RXNE) != RESET)//检查指定的USART中断是否发生  接受中断
    {
        USART_ReceiveData(rc_USARTx);//清空空闲挂起标志
    }
    else if (USART_GetITStatus(rc_USARTx, USART_IT_IDLE) != RESET)//空闲中断
    {
        static uint16_t this_time_rx_len = 0;
        USART_ReceiveData(rc_USARTx);

        if(DMA_GetCurrentMemoryTarget(rc_DMAx_Streamx) == 0)		//大疆DMA1_Stream5
        {
            //重新设置DMA
            DMA_Cmd(rc_DMAx_Streamx, DISABLE);
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(rc_DMAx_Streamx);
            DMA_SetCurrDataCounter(rc_DMAx_Streamx, SBUS_RX_BUF_NUM);	//将 dma 内存指针重新定位到起始位置
            rc_DMAx_Streamx->CR |= DMA_SxCR_CT;	//启用当前选定的内存为内存 1
            //清DMA中断标志
            DMA_ClearFlag(rc_DMAx_Streamx, DMA_FLAG_TCIF4 | DMA_FLAG_HTIF4);
            DMA_Cmd(rc_DMAx_Streamx, ENABLE);

            if(this_time_rx_len == RC_FRAME_LENGTH)	//确保接收到完整的帧数据。
            {
                //处理遥控器数据
                SBUS_TO_RC(SBUS_rx_buf[0], &rc_ctrl);
                //记录数据接收时间
                //DetectHook(DBUSTOE);
            }
        }
        else
        {
            //重新设置DMA
            DMA_Cmd(rc_DMAx_Streamx, DISABLE);
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(rc_DMAx_Streamx);
            DMA_SetCurrDataCounter(rc_DMAx_Streamx, SBUS_RX_BUF_NUM);	//将 dma 内存指针重新定位到起始位置
            rc_DMAx_Streamx->CR &= ~(DMA_SxCR_CT);
            //清DMA中断标志
            DMA_ClearFlag(rc_DMAx_Streamx, DMA_FLAG_TCIF4 | DMA_FLAG_HTIF4);
            DMA_Cmd(rc_DMAx_Streamx, ENABLE);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                SBUS_TO_RC(SBUS_rx_buf[1], &rc_ctrl);
            }
        }
				USART_ClearITPendingBit(rc_USARTx, USART_IT_IDLE);
    }
}

static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff;        //!< Channel 2
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;		//控制
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

