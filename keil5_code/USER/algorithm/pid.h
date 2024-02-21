#ifndef __PID_H
#define __PID_H
#include "stdint.h"

#define off 0
#define on 1

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    int mode;
    //PID ������
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //������
    float max_iout; //���������

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    float error[3]; //����� 0���� 1��һ�� 2���ϴ�
	int integral;	//���ֿ���
	float compare;

} PidType;

void PID_init(PidType *pid, uint8_t mode,float p, float i, float d, float max_out, float max_iout, float compare);
float PID_calc(PidType *pid, float ref, float set);
void pid_clear(PidType *pid);
void pid_reset(PidType *pid,float p,float i,float d);

#endif



