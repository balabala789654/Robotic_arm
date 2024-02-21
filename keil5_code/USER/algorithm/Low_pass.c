#include "Low_pass.h"

static float Low_pass_output_last[10]={0, 0, 0, 0 ,0, 0, 0, 0, 0 ,0};
static float p = 0.005f;

void Low_pass(float _input, float* _output, char _i){
	 *_output = (_input*p)+(Low_pass_output_last[_i]*(1-p));
	Low_pass_output_last[_i] = *_output;
	return;
}

