he = [0 0 0 0 0 0]';

KD = [10;10;10;10;10;10];
KP = [100;100;100;100;100;100];
values_loader;
g_q = [0;0;-g*m3];

% simulink trajectory
qi = [0.1 pi/2 0.1]';
dqi = [0;0;0];
qf = [0 pi/2 -0.1]';
% myRobot.setAllConfig(qf);
% myRobot.show;
Ts = 0.001;

xd = getK(qf);
xr = xd;
%env is orthogonal to z direction and a little above to desired position
%the robot must go down and collide with env
xr(3) = xr(3)+0.05;

K = diag([1 1 1000 1 1 1]);

open('simulink_models\compliance_control.slx');
sim('simulink_models\compliance_control.slx');