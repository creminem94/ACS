q = qi;
Rd = eul2rotm(xd(4:6)');
t = xd(1:3);
Td = [Rd t
    zeros(1,3) 1];
xTilde = getK(q);
jj = getJad(q, xTilde, Td);
