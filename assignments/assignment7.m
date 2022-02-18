% Assignment 7
% Design the Joint Space Inverse Dynamics Control law

KD = [20;50;20];
KP = [80;300;100];
clear qd
wrongModel = 0;

% simulink trajectory
qi = [0 0 0]';
qf = [0.2 pi 0.2]';
dqi = [0 0 0]';
dqf = 0;
dqm = 0.1;
ddqm = 0.1;
dddqm = 0.5;
ti = 0;
tf = 10;
alpha = 0.4;
beta = 0.4;
Ts = 0.001;

TimeValues = [ti:Ts:tf];
DimValues = 3;

DataPositions = [];
DataVelocities = [];
DataAccelerations = [];

for i=1:DimValues
    fprintf("\nEvaluating qi=%f,qf=%f\n",qi(i),qf(i));
    traj = doubleStrajectory(qi(i),qf(i),dqi(i),dqf,dqm,ddqm,dddqm,ti,tf,alpha,beta,Ts);

    DataPositions(i, :) = traj.q;
    DataVelocities(i, :) = traj.dq;
    DataAccelerations(i, :) = traj.ddq;
end

qd.time=TimeValues;
qd.signals.values=DataPositions';
qd.signals.dimensions=DimValues;

dqd.time=TimeValues;
dqd.signals.values=DataVelocities';
dqd.signals.dimensions=DimValues;

ddqd.time=TimeValues;
ddqd.signals.values=DataVelocities';
ddqd.signals.dimensions=DimValues;

open('simulink_models\joint_space_inv_dyn.slx');
sim('simulink_models\joint_space_inv_dyn.slx');

% notes: gravity term error will affect steady state error, coriolis and
% centrifugal term error will affect transient. Try to remove one of the 2
% from N to see the effect

% settling time piccolo = alti gain

%%
% Check that in the nominal case the dynamic behaviour is equivalent to the one of a
% set of stabilized double integrators
KD = [20;20;20];
KP = [100;100;100];
wrongModel = 0;
sim('simulink_models\joint_space_inv_dyn.slx');
%%
% Check the behavior of the control law when the ^B; ^C; ^g used within the controller are
% different than the “true ones” B;C; g (e.g. slightly modify the masses, the frictions, ...)
KD = [20;20;20];
KP = [100;100;100];
wrongModel = 1;
sim('simulink_models\joint_space_inv_dyn.slx');
%% What happens to the torque values when the settling time of the equivalent second
% order systems is chosen very small?
KD = [40;40;40];
KP = [500;500;500];
wrongModel = 0;
% set step response
sim('simulink_models\joint_space_inv_dyn.slx',1);