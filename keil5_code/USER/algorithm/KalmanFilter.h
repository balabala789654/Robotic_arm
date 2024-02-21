#ifndef _KALMANFILTER_H
#define _KALMANFILTER_H

#include "arm_math.h"

#define mat         arm_matrix_instance_f32 //float
#define mat_f64      arm_matrix_instance_f64 //double
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32//浮点矩阵转置
#define mat_inv     arm_mat_inverse_f32//矩阵的逆
#define mat_inv_f64 arm_mat_inverse_f64



typedef struct 
{
	mat X;//
	mat P;//
	mat F;//状态转移矩阵
	mat B;//控制矩阵
	mat U;//
	mat Q;//协方差矩阵
	mat R;//方差
	mat K;//卡尔曼增益
	mat H;//观测矩阵
	mat Z;//
	mat I;// 单位矩阵
	
}KALMAN_FILTER_MAT;


typedef struct
{
	float X_data[2];//状态
	float P_data[4];//协方差矩阵
	float F_data[4];//状态转移矩阵
	float Q_data[4];//状态转移协方差矩阵
	float H_data[2];//观测矩阵
	float B_data[2];//控制矩阵
	float R_data[1];
	float K_data[2];//kalman增益
	float Z_data[1];//传感器数据
	
}KALMAN_FILTER_DATA;




#endif // !_KALMAN_FILTER_H
