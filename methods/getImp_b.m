function b = getImp_b(xTilde,Td,dxd)
Rd = Td(1:3,1:3);
od_dot = dxd(1:3);
wd = dxd(4:6);
ode_d = -xTilde(1:3);
wd_d = Rd*wd;
phi = -xTilde(4);
theta = -xTilde(5);
T = [
0, -sin(phi), cos(phi)*sin(theta)
0,  cos(phi), sin(phi)*sin(theta)
1,         0,          cos(theta)
];
b = [
Rd'*od_dot+cross(wd_d,ode_d)
pinv(T)*wd_d
];
end

