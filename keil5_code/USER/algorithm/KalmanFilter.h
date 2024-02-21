#ifndef _KALMANFILTER_H
#define _KALMANFILTER_H

#include "arm_math.h"

#define mat         arm_matrix_instance_f32 //float
#define mat_f64      arm_matrix_instance_f64 //double
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32//�������ת��
#define mat_inv     arm_mat_inverse_f32//�������
#define mat_inv_f64 arm_mat_inverse_f64



typedef struct 
{
	mat X;//
	mat P;//
	mat F;//״̬ת�ƾ���
	mat B;//���ƾ���
	mat U;//
	mat Q;//Э�������
	mat R;//����
	mat K;//����������
	mat H;//�۲����
	mat Z;//
	mat I;// ��λ����
	
}KALMAN_FILTER_MAT;


typedef struct
{
	float X_data[2];//״̬
	float P_data[4];//Э�������
	float F_data[4];//״̬ת�ƾ���
	float Q_data[4];//״̬ת��Э�������
	float H_data[2];//�۲����
	float B_data[2];//���ƾ���
	float R_data[1];
	float K_data[2];//kalman����
	float Z_data[1];//����������
	
}KALMAN_FILTER_DATA;




#endif // !_KALMAN_FILTER_H
