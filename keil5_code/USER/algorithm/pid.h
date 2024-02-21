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
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次
	int integral;	//积分开关
	float compare;

} PidType;

void PID_init(PidType *pid, uint8_t mode,float p, float i, float d, float max_out, float max_iout, float compare);
float PID_calc(PidType *pid, float ref, float set);
void pid_clear(PidType *pid);
void pid_reset(PidType *pid,float p,float i,float d);

#endif



