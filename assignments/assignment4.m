% Assignemnt 4
% Compute the RNE formulation


zero = zeros(3,1);
syms g real;
g0 = [0;-g;0];
tau = RNE(myRobot, myRobot.q, myRobot.dq, myRobot.ddq, g0);
G = RNE(myRobot, myRobot.q, zero, zero, g0);
C = RNE(myRobot,myRobot.q, myRobot.dq, zero, zero);
N = myRobot.N;
B = zeros(N,N, 'sym');
assume(B, 'real');
for i = 1:N
    ei = zeros(N, 1);
    ei(i) = 1;
    B(:,i) = RNE(myRobot, myRobot.q, zero, ei, zero);
end

disp('B RNE');
disp(B);
disp('C RNE * dq');
disp(C);
disp('G RNE');
disp(G);
disp('TAU RNE');
disp(tau);

myRobot.setTrajectoryPoint([0.1;pi/3;-0.1], [0.1;0.1;0.1]);
disp('Set configuration [0.1;pi/3;-0.1] to see some values');
disp('B RNE');
disp(myRobot.setValues(B, false, true));
disp('C RNE * dq');
disp(myRobot.setValues(C, false, true));
disp('G RNE');
disp(myRobot.setValues(G, false, true));
disp('TAU RNE');
disp(myRobot.setValues(tau, false, true));
