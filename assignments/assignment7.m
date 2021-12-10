he = [0 0 0 0 0 0]';

KD = [10;5;10];
KP = [50;100;50];

% simulink trajectory
qi = [0 0 0]';
qf = [0.1 pi 0.2]';
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