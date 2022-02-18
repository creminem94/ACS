% Assignment 6
% Design the Joint Space PD control law with gravity compensation
% What happens if g(q) is not taken into account?
% What happens if the gravity term is set constant and equal to g(qd ) within the control law?
% What happens if qd is not constant (e.g. qd (t) = hat_qd + delta sin(wt))?

%joint space pd control with gravity compensation
KD = [100;5;5];
KP = [1000;50;50];

% g_q = [0;0;0]; %without gravity compensation it just take a little bit more to reach steady state
values_loader;
g_q = [0;0;-g*m3];
addNoise = 0;

% simulink trajectory
qi = [0 0 0]';
qd = [0.2 pi 0.2]';
dqi = [0 0 0]';
Ts = 0.001;
open('simulink_models\joint_space_pd_w_g_comp.slx');
sim('simulink_models\joint_space_pd_w_g_comp.slx');

%% g(q) not taken into account
addNoise = 0;
g_q = zeros(3,1);
sim('simulink_models\joint_space_pd_w_g_comp.slx');

%% fixed to a wrong value
addNoise = 0;
g_q = [0;0;-g*m3*1.5];
sim('simulink_models\joint_space_pd_w_g_comp.slx');

%% qd not const 
g_q = [0;0;-g*m3];
addNoise = 1;
sim('simulink_models\joint_space_pd_w_g_comp.slx');