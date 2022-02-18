% Assignment 12
% Implement the impedance control in the operational space.

KD = [200;40;10;10;10;10];
KP = [1000;200;50;50;50;50];
Md = diag([0.3;0.2;0.1;1;1;1]);
invMd = inv(Md);
values_loader;
g_q = [0;0;-g*m3];

K = diag([1 1 10 1 1 1]);

% simulink trajectory
qi = [0.1 pi/2 0.1]';
dqi = [0;0;0];
qf = [0.2 pi/3 -0.1]';
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
xf = getK(qf);

xr = xf;
%env is orthogonal to z direction and a little above to desired position
%the robot must go down and collide with env
xr(3) = xr(3)+0.05;

TimeValues = [ti:Ts:tf];
DimValues = 6;

DataPositions = [];
DataVelocities = [];
DataAccelerations = [];

for i=1:DimValues
    fprintf("\nEvaluating qi=%f,qf=%f\n",xi(i),xf(i));
    traj = doubleStrajectory(xi(i),xf(i),0,0,dqm,ddqm,dddqm,ti,tf,alpha,beta,Ts);

    DataPositions(i, :) = traj.q;
    DataVelocities(i, :) = traj.dq;
    DataAccelerations(i, :) = traj.ddq;
end
clear xd;
xd.time=TimeValues;
xd.signals.values=DataPositions';
xd.signals.dimensions=DimValues;

dxd.time=TimeValues;
dxd.signals.values=DataVelocities';
dxd.signals.dimensions=DimValues;

ddxd.time=TimeValues;
ddxd.signals.values=DataVelocities';
ddxd.signals.dimensions=DimValues;
open('simulink_models\impedance_control.slx');
sim('simulink_models\impedance_control.slx');
