function B = getB(q)
%myRobot.B_RNE
q1 = q(1);q2 = q(2);q3 = q(3);
B = [
[                            0.3*cos(q2)^2 + 0.3*sin(q2)^2 + 0.3,                                        cos(q2)*(0.09*cos(q2) + 0.06) - 0.09*sin(q2)^2,   0]
[0.3*cos(q2)*(0.3*cos(q2) + 0.3) - 0.09*sin(q2)^2 - 0.03*cos(q2), 0.027*sin(q2)^2 - 0.009*cos(q2) + (0.3*cos(q2) + 0.3)*(0.09*cos(q2) + 0.06) + 0.01361,   0]
[                                                              0,                                                                                     0, 0.1]
];
end

