he = [0 0 0 0 0 0]';

KD = [200;40;10;10;10;10];
KP = [1000;50;50;50;50;50];
values_loader;
g_q = [0;0;-g*m3];

% simulink trajectory
qi = [0 pi/3 0]';
dqi = [0;0;0];
qf = [0.2 pi -0.1]';
Ts = 0.001;

xd = getK(qf);%[-0.3; 0.55;0.2;0;1.7;0];

% Rd = [
% 1 0 0
% 0 1 0
% 0 0 1
% ];
% 
% Td = [Rd Od
%     zeros(1,3) 1];

% open('simulink_models\compliance_control.slx');
% sim('simulink_models\compliance_control.slx');