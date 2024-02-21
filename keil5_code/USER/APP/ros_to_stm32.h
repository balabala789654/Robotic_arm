#ifndef _ROS_TO_STM32
#define _ROS_TO_STM32

#define frame_head 0x5a
#define frame_end 0xa5
void ros_to_stm32_init(int bound);
float*  ros_to_stm32_output(void);


#endif


