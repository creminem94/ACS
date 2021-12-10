function j = getInvJa(q)
%myRobot.setValues(pinv(myRobot.Ja))
q2 = q(2);
j = [
[(0.09*cos(q2)*sin(q2))/(0.09*sin(q2)^2 + 1.0), 1.0,   0, -(0.3*cos(q2))/(0.09*sin(q2)^2 + 1.0), 0, 0]
[        -(0.3*sin(q2))/(0.09*sin(q2)^2 + 1.0),   0,   0,              1/(0.09*sin(q2)^2 + 1.0), 0, 0]
[                                            0,   0, 1.0,                                     0, 0, 0]
];
end

