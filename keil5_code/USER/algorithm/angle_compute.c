#include "angle_compute.h"

double angle = 0;

double angle_com(RC_ctrl_t* RC)
{
	double x,y;
	x = RC->rc.ch[2];
	y = RC->rc.ch[3];
	
	
	angle = atan2(y,x);
	return angle;
}

