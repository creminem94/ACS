% Assignment 8
% Implement in Simulink the Adaptive Control law for the a 1-DoF link under gravity

KD = 30;
KP = 100;
lambda = 10;
Ktheta = inv(diag([100 100 100]));
A = 1;

Ts = 0.001;

open('simulink_models\joint_space_adaptive.slx');
sim('simulink_models\joint_space_adaptive.slx');