#include "Remote_Control.h"
#include "DJI_remote.h"
#include "math.h"

#define x_max 0.7f
#define x_min -0.7f

#define y_max 0.7f
#define y_min -0.7f

#define z_max 0.5f
#define z_min -0.4f

void DJI_remote_init(void){
	remote_control_init();
	return;
}

void max_min_check(float _max, float _min, float* _input){
	if(*_input >= _max) *_input = _max;
	else if(*_input <= _min) *_input = _min;
	return;
}

static float delta_step = 0.0001f;
float alpha = 0.0f;
float length = 0.390f;

void DJI_remote_control(float* _x, float* _y, float* _z, float* _angle){
	
	
	if(rc_ctrl.rc.s[1] == 2 && rc_ctrl.rc.s[0] == 2) return;
	
	if(rc_ctrl.rc.ch[2] < 0) alpha+=delta_step*3.0f;
	else if(rc_ctrl.rc.ch[2] > 0) alpha-=delta_step*3.0f;
	
	if(rc_ctrl.rc.ch[3] > 0) length+=delta_step;
	else if(rc_ctrl.rc.ch[3] < 0) length-=delta_step;
	
	if(rc_ctrl.rc.ch[1] > 0) *_z+=delta_step;
	else if(rc_ctrl.rc.ch[1] < 0) *_z-=delta_step;
	
	if(rc_ctrl.rc.s[1] == 3) *_angle=1.57f;
	else if(rc_ctrl.rc.s[1] == 1) *_angle=0.0f;
	
	*_x = cosf(alpha) * length;
	*_y = sinf(alpha) * length;
	
	max_min_check(x_max, x_min, _x);
	max_min_check(y_max, y_min, _y);
	max_min_check(z_max, z_min, _z);
	
	return;
}

