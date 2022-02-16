function dJad = getDiffJad(dxd, q, Td)
q2 = q(2);
Tde = getTde(q, Td);
angles = tform2eul(Tde, 'ZYZ');
phi =   angles(1);
theta = angles(2);
T = [
    0, -sin(phi), cos(phi)*sin(theta)
    0,  cos(phi), sin(phi)*sin(theta)
    1,         0,          cos(theta)
];
Ta = [
eye(3) zeros(3,3)
zeros(3,3) T
];
wd = dxd(4:6);
dphi = wd(1);
dtheta = wd(2);
dT = [
0, -cos(phi)*dphi, -sin(phi)*sin(theta)*dphi+cos(phi)*cos(theta)*dtheta
0, -sin(phi)*dphi, cos(phi)*sin(theta)*dtheta+sin(phi)*cos(theta)*dtheta
0, 0, -sin(theta)*dtheta
];
dTa = [
    zeros(3,6)
    zeros(3,3) dT
];
invTa = pinv(Ta);
invDTa = -invTa*dTa*invTa;
J = [
[0, -0.3*sin(q2), 0]
[1,  0.3*cos(q2), 0]
[0,           0, 1]
[0,           0, 0]
[0,           0, 0]
[0,           1, 0]    
];
dJ = [
[0, -0.3*cos(q2), 0]
[0, -0.3*sin(q2), 0]
[0,           0, 0]
[0,           0, 0]
[0,           0, 0]
[0,           0, 0]    
];
Rdt = Td(1:3, 1:3)';
swd=[0 -wd(3) wd(2)
    wd(3) 0 -wd(1)
    -wd(2) wd(1) 0 ];
RdMat = [Rdt zeros(3,3);zeros(3,3) Rdt];
dJad = invTa*([Rdt*swd zeros(3,3);zeros(3,3) Rdt*swd]*J+RdMat*dJ)-invDTa*RdMat*J;
end

