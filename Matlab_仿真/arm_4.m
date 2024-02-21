clc;
clear;

% 创建 Link 对象，定义机器人的关节
L(1) = Link('revolute', 'd', 1, 'a', 0, 'alpha', pi/2, 'offset', 0);
L(2) = Link('revolute', 'd', 0, 'a', 1, 'alpha', 0, 'offset', pi/2);
L(3) = Link('revolute', 'd', 0, 'a', 1, 'alpha', 0, 'offset', 0);
L(4) = Link('revolute', 'd', 0, 'a', 1, 'alpha', 0, 'offset', 0);
% 创建 SerialLink 对象，表示串联机器人
robot = SerialLink(L, 'name', 'my_robot');

robot.teach

l1 = 1;
l2 = 1;
l3 = 1;
l4 = 1;

while true
    theta = robot.getpos()

    theta_1 = theta(1);
    theta_2 = -theta(2);
    theta_3 = -theta(3);
    theta_4 = -theta(4);

%     theta_1 = deg2rad(theta_1);
%     theta_2 = deg2rad(theta_2);
%     theta_3 = deg2rad(theta_3);
%     theta_4 = deg2rad(theta_4);

    mat_1 = [0 sin(theta_2)*cos(theta_1) sin(theta_2+theta_3)*cos(theta_1) sin(theta_2+theta_3+theta_4)*cos(theta_1);
            0 sin(theta_2)*sin(theta_1) sin(theta_2+theta_3)*sin(theta_1) sin(theta_2+theta_3+theta_4)*sin(theta_1);
            1 cos(theta_2) cos(theta_2+theta_3) cos(theta_2+theta_3+theta_4)];
    x = mat_1 * [l1 l2 l3 l4]';
    disp(x)
    pause(1)
end




