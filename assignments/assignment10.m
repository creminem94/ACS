he = [0 0 0 0 0 0]';

KD = [200;10;10;10;10;10];
KP = [1000;50;50;50;50;50];
g_q = [0;0;-g*m3];

% simulink trajectory
qi = [0 pi/3 0]';
qf = [0.2 pi -0.1]';
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

xi = getK(qi);
xf = getK(qf);

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

xd.time=TimeValues;
xd.signals.values=DataPositions';
xd.signals.dimensions=DimValues;

dxd.time=TimeValues;
dxd.signals.values=DataVelocities';
dxd.signals.dimensions=DimValues;

ddxd.time=TimeValues;
ddxd.signals.values=DataVelocities';
ddxd.signals.dimensions=DimValues;
open('simulink_models\operational_space_pd_w_g_comp.slx');
sim('simulink_models\operational_space_pd_w_g_comp.slx');