%%

syms dB a2 q1 q2 q3 m1 L1 s1 m2 L2 ro ri m3 L3 s3 real;

DH = [
    0 -pi/2 dB 0
    0 pi/2 q1+L1/2 0
    a2 0 0 q2
    0 0 q3-L3/2 0
    0 pi 0 0
    ];
links = [
    Link("box", m1, s1, s1, L1, [0;0;-L1/2]) 
    Link("cyl", m2, ro, ri, L2, [-L2/2;0;0])
    Link("box", m3, s3, s3, L3, [0;0;-L3/2])
];
myRobot = MyRobot('PRP.urdf', DH, links);
% myRobot.show;
% myRobot.details;

%% 

he = [0 0 0 0 0 0]';

%joint space pd control with gravity compensation
KD = [10;5;10];
KP = [50;100;50];

% g_q = [0;0;0]; %without gravity compensation it just take a little bit more to reach steady state
values_loader;
g_q = [0;0;-g*m3];

g_q_c = g_q; %[0;0;-g*m3*1.1];

%joint space inv dyn control
Kd_inv = [10;5;10];
Kp_inv = [50;100;50];

%% simulink trajectory

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

%%
myRobot.setValues(myRobot.B)
myRobot.setValues(myRobot.B_RNE)
% 
myRobot.setValues(myRobot.C*myRobot.dq)
myRobot.setValues(myRobot.C_RNE)

% myRobot.G_RNE
% myRobot.G
% myRobot.setValues(myRobot.G_RNE)  
% myRobot.setValues(myRobot.G)

% myRobot.setAllConfig([0, pi/3, 0]);
% myRobot.setValues(myRobot.C)
% myRobot.setValues(myRobot.B)
% myRobot.setValues(myRobot.G)
% 
% myRobot.setValues(myRobot.TAU)
% myRobot.setValues(myRobot.TAU_RNE)

% 
% myRobot.setAllConfig([0, 0, 0]);
% 
% T = myRobot.getTransform(5);
% J = myRobot.geometricJacobian(5);

% myRobot.inertiaCoM
% myRobot.translatedInertia
% myRobot.partialJacobian

% 
% myRobot.setValues(T)
% myRobot.toolboxT
% myRobot.setValues(J)
% myRobot.toolboxJg

%assignment 2
% k = myRobot.kineticEnergy(dq);
% u = myRobot.potentialEnergy;
% myRobot.setValues(k)
% myRobot.setValues(u)

%assignment 3
% myRobot.TAUi(1, q, dq, ddq)


%% TODO, 
% - verificare le multiple inverse se fattibili
% - sistemare overleaf
