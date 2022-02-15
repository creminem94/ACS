% Assignment 1
% DH table
% direct kinematics
% inverse kinematics
% Jacobians (geometric and analytical)
% By hand, and cross-checking with Robotics toolbox

% myRobot.show;
disp('DH Table');
disp(myRobot.DH);
disp('Direct Kinematics');
disp(myRobot.allT(:,:,end));
disp('Inverse Kinematics');
disp(ik());
disp('Geometric Jacobian');
disp(myRobot.J);
disp('Analytical Jacobian');
disp(myRobot.Ja);

myRobot.setAllConfig([0.1;pi/3;-0.1]);
disp('Set configuration [0.1;pi/3;-0.1] to cross check with toolbox');
disp('Direct kinematics computed');
disp(myRobot.setValues(myRobot.allT(:,:,end), true));
disp('Direct kinematics toolbox');
disp(myRobot.toolboxT);
disp('Inverse kinematics computed')
disp(myRobot.setValues(myRobot.inverseKinematics', true));
disp('Inverse kinematics toolbox');
disp(myRobot.toolboxIk);
disp('Geometric Jacobian computed')
disp(myRobot.setValues(myRobot.J, true));
disp('Geometric Jacobian toolbox');
disp(myRobot.toolboxJg);
