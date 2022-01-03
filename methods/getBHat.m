function B = getBHat(q)
%myRobot.B_RNE
q2 = q(2);
B = [
[         0.9,                                    0.09*cos(q2),    0]
[0.09*cos(q2), 0.02025*cos(q2)^2 + 0.02025*sin(q2)^2 + 0.02041,    0]
[           0,                                               0, 0.15]
];
end

