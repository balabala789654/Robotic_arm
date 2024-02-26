# 介绍

<p>祝NCIST风暴战队在rc中取得好成绩:-)</p>

##  机械臂中的pipper准则

机械臂逆运动学求解中一般分为两种 **数值解**与**解析解**，然而区分机械臂是否有解析解便要用到**pipper准则**

**pipper准则如下**

``三个相邻关节轴相交于一点``

``三个相邻关节轴相互平行``

而在这个机械臂中符合pipper准则，因此存在解析解，所以采用解析解对机械臂逆运动学进行求解，避免复杂的数值解。

##  逆运动学解析

逆运动学解析为<u>已知机械臂末端坐标从而解析各个关节的旋转角度</u>

###  核心公式：余弦定理

![余弦定理](.\assets\pic\1.png)



在该机械臂逆运动学解析中只需要用到**两次余弦定理**即可。

##  代码讲解

###  Matlab

在Matlab仿真文件夹中存放着**arm_4.m**与**arm_4_inv.m**文件，前者是***四轴机械臂运动学仿真文件***，后者为***四轴机械臂逆运动学仿真文件***

**arm_4_inv.m**中的

```c
l1 = 0.410; l2 = 0.390; l3 = 0.100;
```

**l1**，**l2**，**l3**分别为三节臂的长度

`````c
x = input('x: '); y = input('y: '); z = input('z: ');
angle = input('angle: ');
`````

输入机械臂末端坐标与最后关节与横坐标的夹角



```c
hypotenuse = sqrt(joint_position(1)^2 + joint_position(2)^2 + joint_position(3)^2)


var_1 = (hypotenuse^2 + l1^2 - l2^2)/(2*hypotenuse*l1)
var_2 = (l1^2 + l2^2 - hypotenuse^2)/(2*l1*l2)

if var_1>1 || var_2>1
    disp("机械臂无法到达该坐标")
    continue
end
alpha = atan(joint_position(3)/sqrt(joint_position(1)^2 + joint_position(2)^2))
beta = acos(var_1)
theta_2 = pi/2 - acos(var_2)    
theta_1 = pi/2 -(alpha + beta)
theta_3 = angle - theta_1 - theta_2
theta_ori = atan2(y, x)
```

**hypotenuse**为末端到坐标系原点的距离

**var_1**与**var_2**为两次余弦定理的计算

**alpha**，**beta**，**theta_1**，**theta_2**，**theta_3**，**theta_ori**为各个角度的计算，其中**theta_1**，**theta_2**，**theta_3**，**theta_ori**为四个电机的角度

###  keil5

keil5中的机械臂逆运动学解析程序在**arm_4.h**文件中

```c
void Arm_4_init(void){
	arm.l1 = 0.410f;
	arm.l2 = 0.390f;
	arm.l3 = 0.100f;
	arm.alpha = 0;
	arm.beta = 0;
	arm.theta_1 = 0;
	arm.theta_2 = 0;
	arm.theta_3 = 0;
	arm.theta_ori = 0;
	arm.AK80_8_int_angle = 100.0f;
	arm.CyberGear_init_angle = deg_to_rad(-150.0f);
	arm.DJI_2006_init_angle = deg_to_rad(120.0f);
	
	AK80_8_set();
	CyberGear_set();
	DJI_6020_set();
}
```

该**Arm_4_init()**函数功能为初始化，`arm.l1 = 0.410f;` `arm.l2 = 0.390f;` `arm.l3 = 0.100f;`为三节臂长，如果之后需要改进机械臂的臂长， 只需要改变这三个参数即可。

```c
	arm.AK80_8_int_angle = 100.0f;
	arm.CyberGear_init_angle = deg_to_rad(-150.0f);
	arm.DJI_2006_init_angle = deg_to_rad(120.0f);
```

这三个参数为电机的初始角度，目的是为了使机械臂在上电后运动到**初始姿态**，机械臂的**初始姿态**为

![机械臂的初始姿态](.\assets\pic\2.png)

如果在之后的使用中发现机械臂并没有在上电后保持该**初始姿态**，则需调节参数，参数为**角度制**

```c
	AK80_8_set();
	CyberGear_set();
	DJI_6020_set();
```

这三个函数为角度初始化函数

```c
void joint_position_cal(float _x, float _y, float _z, float _angle){
	if(_angle == 1.57f){
		arm.joint_position[0] = _x;
		arm.joint_position[1] = _y;
		arm.joint_position[2] = _z + arm.l3;
		arm.joint_position[3] = _angle;
		return;
	}
	else if(_angle == 0){
		float gamma = atan2(_y, _x);
		float delta_x = cosf(gamma) * arm.l3;
		float delta_y = sinf(gamma) * arm.l3;
		
		arm.joint_position[0] = _x - delta_x;
		arm.joint_position[1] = _y - delta_y;	
		arm.joint_position[2] = _z;
		arm.joint_position[3] = _angle;
		return;
	}
	else return;
}

void alpha_cal(float _x, float _y, float _z, float* _alpha){
	float hypotenuse = sqrtf(powf(_x, 2)+powf(_y, 2));
	*_alpha = atanf(_z/hypotenuse);
	return;
}

char beta_cal(float _x, float _y, float _z, float* _beta){
	float hypotenuse = sqrtf(powf(_x, 2)+powf(_y, 2)+powf(_z, 2));
	float var = (float)(powf(hypotenuse,2)+powf(arm.l1,2)-powf(arm.l2,2))/(2*hypotenuse*arm.l1);
	if(var > 1){
		return 0;
	}
	else {
		*_beta = acosf(var);
		return 1;
	}
}

char theta_cal(float _x, float _y, float _z){
	float var;
	float hypotenuse = sqrtf(powf(_x, 2)+powf(_y, 2)+powf(_z, 2));
	
	alpha_cal(_x, _y, _z, &arm.alpha);
	if(beta_cal(_x, _y, _z, &arm.beta)){
		var = (powf(arm.l1,2)+powf(arm.l2,2)-powf(hypotenuse,2))/(2*arm.l1*arm.l2);
		if(var > 1){
			return 0;
		}
		else {
			arm.theta_ori = atan2f(_y, _x);
			arm.theta_1 = pi/2 - (arm.alpha + arm.beta);
			arm.theta_2 = pi/2 - acosf(var);
			arm.theta_3 = arm.joint_position[3] - arm.theta_1 - arm.theta_2;
			return 1;
		}
	}
	else {
		return 0;
	}
}
```

**joint_position_cal()**函数, **alpha_cal()**函数, **beta_cal()**函数, **theta_cal()**函数, 为机械臂逆运动学解析最主要的四个函数，具体作用自己体会。

```c
void Arm_4_control(float _x, float _y, float _z, float _angle){
	arm.end_effector_position[0] = _x;
	arm.end_effector_position[1] = _y;
	arm.end_effector_position[2] = _z;
	arm.end_effector_position[3] = _angle;
	
	joint_position_cal(arm.end_effector_position[0], arm.end_effector_position[1], 		  arm.end_effector_position[2], arm.end_effector_position[3]);
	theta_cal(arm.joint_position[0], arm.joint_position[1], arm.joint_position[2]);
	
	
	Ak80_8_control_servo(arm.AK80_8_int_angle + rad_to_deg(arm.theta_1));
	CyberGear_control(5.0f, arm.CyberGear_init_angle + arm.theta_2, 0.0f);
	DJI_motor_control(arm.theta_ori, arm.DJI_2006_init_angle + arm.theta_3);
}
```

**Arm_4_control()**函数为**接口函数**， 四个参数为**机械臂末端坐标**与**最后关节与横坐标的夹角**， 在实际机械臂使用中， 只需要对该函数传入参数即可

目前可支持大疆遥控器控制，控制逻辑为把遥控器的拨杆看作极坐标系的输入，在**DJI_remote.c**文件中，把**极坐标系**转化成**笛卡尔坐标系**，转化后在把得到的四个参数传递到**Arm_4_control()**函数中，代码如下

```c
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
```

