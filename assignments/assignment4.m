% Assignemnt 4
% Compute the RNE formulation

disp('B RNE');
disp(myRobot.B_RNE);
disp('C RNE * dq');
disp(myRobot.C_RNE);
disp('G RNE');
disp(myRobot.G_RNE);
disp('TAU RNE');
disp(myRobot.TAU_RNE);

myRobot.setTrajectoryPoint([0.1;pi/3;-0.1], [0.1;0.1;0.1]);
disp('Set configuration [0.1;pi/3;-0.1] to see some values');
disp('B RNE');
disp(myRobot.setValues(myRobot.B_RNE, false, true));
disp('C RNE * dq');
disp(myRobot.setValues(myRobot.C_RNE, false, true));
disp('G RNE');
disp(myRobot.setValues(myRobot.G_RNE, false, true));
disp('TAU RNE');
disp(myRobot.setValues(myRobot.TAU_RNE, false, true));