disp('B op');
disp(myRobot.Ba);
disp('C op * xd');
disp(myRobot.Ca_xd);
disp('G op');
disp(myRobot.Ga);

myRobot.setTrajectoryPoint([0.1;pi/3;-0.1], [0.1;0.1;0.1]);
disp('Set configuration [0.1;pi/3;-0.1] to see some values');
disp('B op');
disp(myRobot.setValues(myRobot.Ba, false, true));
disp('C op * xd');
disp(myRobot.setValues(myRobot.Ca_xd, false, true));
disp('G op');
disp(myRobot.setValues(myRobot.Ga, false, true));