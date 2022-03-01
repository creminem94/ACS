function j = getDiffJa(q, dq)
q2 = q(2);
dq2 = dq(2);
j = [
[0, -0.3*cos(q2)*dq2, 0]
[0, -0.3*sin(q2)*dq2, 0]
[0,           0, 0]
[0,           0, 0]
[0,           0, 0]
[0,           0, 0]    
];
end

