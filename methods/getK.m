function k = getK(q)
%myRobot.setValues(myRobot.allT(:,:,end))
q1 = q(1); q2 = q(2); q3 = q(3);

T = [
[cos(q2),      sin(q2),    0,             0.3*cos(q2)]
[sin(q2), -1.0*cos(q2),    0, q1 + 0.3*sin(q2) + 0.35]
[      0,            0, -1.0,                q3 + 0.3]
[      0,            0,    0,                     1.0]
];
p = T(1:3, 4);
phi = [
    atan2(T(2,3), T(1,3));
    atan2(sqrt(T(1,3)^2+T(2,3)^2),T(3,3));
    atan2(T(3,2), -T(3,1));
];
k = [p;phi];
end

