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
myRobot.overallInertia

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
% myRobot.TAUi(1, q, dq, ddq)
%% TODO, 
% - verificare le multiple inverse se fattibili
% - sistemare overleaf
