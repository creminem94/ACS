function n = getN(q, dq)
%myRobot.setValues(myRobot.C*myRobot.dq+myRobot.G) 
q1 = q(1);q2 = q(2);q3 = q(3);
dq1 = dq(1);dq2 = dq(2);dq3 = dq(3);

n = [
-0.06*dq2^2*sin(q2)
                  0
               0.98
];
end

