function sys = joint_space_dyn_lagrangian(t,x,u, myRobot, he)
    N = myRobot.N;
    tau = u;
    myRobot.q = x(1:N);
    myRobot.dq = x(N+1:2*N);
    myRobot.computeLagrangian;
    B = myRobot.setValues(myRobot.B);
    C = myRobot.setValues(myRobot.C);
    G = myRobot.setValues(myRobot.G);
    J = myRobot.setValues(myRobot.J);
    ddq = inv(B) * (tau - C*qdot - G - J'*he);
    sys = [myRobot.dq; ddq];
end

