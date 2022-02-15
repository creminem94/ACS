function Ja = getJa(q)
q2 = q(2);
Ja = [
[  0, -0.3*sin(q2),   0]
[1.0,  0.3*cos(q2),   0]
[  0,            0, 1.0]
[  0,          1.0,   0]
[  0,            0,   0]
[  0,            0,   0]
];
end
