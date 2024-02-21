clc;
clear;
close all;

l1 = 0.410; l2 = 0.390; l3 = 0.100;
% l1 = 1; l2 = 1; l3 = 1;
% l1 = 2; l2 = 2; l3 = 1;
% 创建 Link 对象，定义机器人的关节
L(1) = Link('revolute', 'd', 0, 'a', 0, 'alpha', pi/2, 'offset', pi);
L(2) = Link('revolute', 'd', 0, 'a', l1, 'alpha', 0, 'offset', pi/2);
L(3) = Link('revolute', 'd', 0, 'a', l2, 'alpha', 0, 'offset', pi/2);
L(4) = Link('revolute', 'd', 0, 'a', l3, 'alpha', 0, 'offset', 0);
% 创建 SerialLink 对象，表示串联机器人
robot = SerialLink(L, 'name', 'my_robot');

% robot.teach
theta_1 = 0; theta_2 = 0; theta_3 = 0;theta_ori=0;
alpha = 0;
beta = 0;
% robot.plot([0 0 0 0])

while true
    x = input('x: '); y = input('y: '); z = input('z: ');
    angle = input('angle: ');
    
    if angle == pi/2
        joint_position = [x y z+l3]
    elseif angle == 0
        gamma = atan2(y,x)
        delta_x = cos(gamma) * l3
        delta_y = sin(gamma) * l3
        joint_position = [x-delta_x y-delta_y z]
    else 
        continue
    end
    
    %斜边
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
    
    T = robot.fkine([theta_ori theta_1 theta_2 theta_3])
    position = transl(T)  % 获取位置信息
    robot.plot([theta_ori theta_1 theta_2 theta_3])
%     robot.plot([0 0 0 0])
    pause(1)
end
