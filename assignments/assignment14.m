% Assignment 14
% Implement the Force Control with Inner Position Loop.

KD = [10;10;15;10;10;10];
KP = [50;50;20;50;50;50];
Md = diag([1;1;0.1;1;1;1]);
KI = 0;
KF = 0.5;

invMd = inv(Md);
values_loader;
g_q = [0;0;-g*m3];

K = diag([1 1 10 1 1 1]);

fd = [0 0 -1 0 0 0]';

qi = [0.1 pi/2 0.2]';
dqi = [0;0;0];
qf = [0.2 pi/3 -0.2]';

Ts = 0.001;

xi = getK(qi);
xf = getK(qf);

xr = xf;
%env is orthogonal to z direction and a little above to desired position
%the robot must go down and collide with env
xr(3) = xr(3)+0.05;

open('simulink_models\force_control.slx');
sim('simulink_models\force_control.slx');

%%
KD = [10;10;15;10;10;10];
KP = [50;50;20;50;50;50];
Md = diag([1;1;0.1;1;1;1]);
KI = 1;
KF = 0.5;
sim('simulink_models\force_control.slx');
