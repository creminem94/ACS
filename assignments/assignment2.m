disp('Kinetic energy');
disp(myRobot.kineticEnergy);
disp('Potential Energy');
disp(myRobot.potentialEnergy);

myRobot.setTrajectoryPoint([0.1;pi/3;-0.1], [0.1;0.1;0.1]);
disp('Set configuration [0.1;pi/3;-0.1] to see some values');
disp('Kinetic energy');
disp(myRobot.setValues(myRobot.kineticEnergy, false, true));
disp('Potential Energy');
disp(myRobot.setValues(myRobot.potentialEnergy, false, true));
