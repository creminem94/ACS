% Assignment 15
% Implement the Parallel Force/Position Control
KD = [50;50;20;10;10;10];
KP = [1000;300;20;50;50;50];
Md = diag([0.01;0.1;0.1;1;1;1]);
KI = 4;
KF = 5;

invMd = inv(Md);
values_loader;
g_q = [0;0;-g*m3];

K = diag([0 0 15 0 0 0]);
fd = [0 0 -0.5 0 0 0]';
% simulink trajectory
qi = [0 0 0]';
dqi = [0;0;0];
qf = [0.2 pi/2 -0.1]';
dqf = 0;
dqm = 0.1;
ddqm = 0.1;
dddqm = 0.5;
ti = 0;
tf = 10;
alpha = 0.4;
beta = 0.4;
Ts = 0.001;

xi = getK(qi);
xd = getK(qf);

xr = xd;
%env is orthogonal to z direction and a little above to desired position
%the robot must go down and collide with env
xr(3) = xr(3)+0.05;

open('simulink_models\parallel_force_pos_control.slx');
sim('simulink_models\parallel_force_pos_control.slx');
%%
KI = 0;
sim('simulink_models\parallel_force_pos_control.slx');