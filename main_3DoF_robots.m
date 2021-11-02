%%
syms dB q1 a2 q2 q3 real;
syms m1 L1 s1 m2 L2 ro ri m3 L3 s3 real;
syms dq1 dq2 dq3 real;
syms ddq1 ddq2 ddq3 real;

DH = [
    0 -pi/2 dB 0
    0 pi/2 q1+L1/2 0
    a2 0 0 q2
    0 0 q3-L3/2 0
    0 pi 0 0
    ];
myRobot = MyRobot('PRP.urdf', DH);
% myRobot.show;
% myRobot.details;


links = [
    Link("box", m1, s1, s1, L1, [0;0;-L1/2]) 
    Link("cyl", m2, ro, ri, L2, [-L2/2;0;0])
    Link("box", m3, s3, s3, L3, [0;0;-L3/2])
];

myRobot.setLinks(links);
myRobot.setAllConfig([0, 0, 0]);

T = myRobot.getTransform(5);
J = myRobot.geometricJacobian(5);

% myRobot.inertiaCoM
% myRobot.translatedInertia
% myRobot.partialJacobian

q = [q1; q2; q3];
dq = [dq1; dq2; dq3];
ddq = [ddq1; ddq2; ddq3];
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
myRobot.TAUi(3, q, dq, ddq)
%% TODO, 
% - verificare le multiple inverse se fattibili
%% assignment 1 TODO sistemare overleaf
% toolbox computation
tform = getTransform(robot,config,'ee','base_link')
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0.25 0.25 0.25 1 1 1];
[configSoln,solnInfo] = ik('ee', tform, weights, config);
geoJacob = geometricJacobian(robot,config,'ee');

dB = 0.5;
a2 = 0.3;
d1Offset = 0.35;
d3Offset = -0.2;

% manual computation

d1 = config(1).JointPosition+d1Offset;
theta2 = config(2).JointPosition;
d3 = config(3).JointPosition+d3Offset;
[rows cols] = size(DH);
tformManual = subs(Ti(:,:,rows))

geoJacobManual = [
    0 -a2*sin(theta2) 0
    1 a2*cos(theta2) 0
    0 0 1
    0 0 0
    0 0 0
    0 1 0
];

% manually computed inverse kinematics
Px = tform(1,4);
Py = tform(2,4);
Pz = tform(3,4);
subs([dB a2 d1]);
d3Man = Pz - dB;
d1Man = Py - sqrt(a2^2-Px^2);
theta2Man = atan2(Py-d1Man, Px);
ikManConfig = homeConfiguration(robot);
ikManConfig(1).JointPosition = d1Man-d1Offset;
ikManConfig(2).JointPosition = theta2Man;
ikManConfig(3).JointPosition = d3Man-d3Offset;
figure;
show(robot, ikManConfig);
xlim([-0.5 0.5])
ylim([-0.5 0.8])
zlim([0 0.8])

