#include "pid.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }



void PID_init(PidType *pid, uint8_t mode,float p, float i, float d, float max_out, float max_iout, float compare)
{
    pid->mode = mode;//PID算法模式
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
	pid->compare = compare;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

float PID_calc(PidType *pid, float ref, float set)
{
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
	/**********************************积分分离判断***************************************/
	if(pid->error[0] >= pid->compare ||pid->error[0] <= -(pid->compare)) pid->integral = off;	//积分关闭
	else pid->integral = on;	//积分开启
	/*********************************************************************************/
    if (pid->mode == PID_POSITION)//位置式PID算法
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->integral * pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)//增量式PID算法
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->integral * pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return 0;
}

void pid_clear(PidType *pid)
{
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->set = pid->fdb = 0.0f;  
}
void pid_reset(PidType *pid,float p,float i,float d)
{
    pid->Kd=d;
    pid->Kp=p;
    pid->Ki=i;

    pid->Pout = pid->Iout = pid->Dout = 0;
}




