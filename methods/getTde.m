function Tde = getTde(q, Td)
% get transform from desired frame to end effector frame
q1 = q(1); q2 = q(2); q3 = q(3);
Te = [
[cos(q2),      sin(q2),    0,             0.3*cos(q2)]
[sin(q2), -1.0*cos(q2),    0, q1 + 0.3*sin(q2) + 0.35]
[      0,            0, -1.0,                q3 + 0.3]
[      0,            0,    0,                     1.0]
];
Tde = Td\Te;
end

