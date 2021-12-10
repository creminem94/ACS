disp('B');
disp(myRobot.B);
disp('C');
disp(myRobot.C);
disp('G');
disp(myRobot.G);
disp('TAU');
disp(myRobot.TAU);

myRobot.setTrajectoryPoint([0.1;pi/3;-0.1], [0.1;0.1;0.1]);
disp('Set configuration [0.1;pi/3;-0.1] to see some values');
disp('B');
disp(myRobot.setValues(myRobot.B, false, true));
disp('C');
disp(myRobot.setValues(myRobot.C, false, true));
disp('G');
disp(myRobot.setValues(myRobot.G, false, true));
disp('TAU');
disp(myRobot.setValues(myRobot.TAU, false, true));