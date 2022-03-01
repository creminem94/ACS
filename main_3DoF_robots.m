clear all;
close all;
addpath methods assignments classes simulink_models;
syms dB a2 q1 q2 q3 m1 L1 s1 m2 L2 ro ri m3 L3 s3 real;

DH = [
    0 -pi/2 dB 0
    0 pi/2 q1+L1/2 0
    a2 0 0 q2
    0 0 q3-L3/2 0
    0 pi 0 0
    ];

links = [
    Link("box", m1, s1, s1, L1, [0;-L1/2;0]) 
    Link("cyl", m2, ro, ri, L2, [-L2/2;0;0])
    Link("box", m3, s3, s3, L3, [0;0;L3/2])
];
myRobot = MyRobot('PRP.urdf', DH, links);

% for visual simulation it needs a variable
rbt = importrobot('PRP.urdf');

%% todo
%verificare derivata jacobiana rispetto al tempo, moltiplicare per dq

