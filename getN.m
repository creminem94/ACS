function n = getN(q, dq)
%myRobot.setValues(myRobot.C_RNE+myRobot.G_RNE) 
q1 = q(1);q2 = q(2);q3 = q(3);
dq1 = dq(1);dq2 = dq(2);dq3 = dq(3);
he = zeros(6,1);
fex = he(1);fey = he(2);fez = he(3);
uex = he(4);uey = he(5);uez = he(6);

n = [
fex*sin(q2) - 1.0*fey*cos(q2) - 1.0*sin(q2)*(0.06*dq2^2 - 1.0*fex + 0.09*dq2^2*cos(q2)) - 1.0*cos(q2)*(0.09*sin(q2)*dq2^2 + fey)
0.6*fey - 2.0*uez + 0.009*dq2^2*sin(q2) + 0.3*sin(q2)*(0.06*dq2^2 - 1.0*fex + 0.09*dq2^2*cos(q2)) - 0.3*fex*sin(q2) - 1.0*(0.09*sin(q2)*dq2^2 + fey)*(0.3*cos(q2) + 0.3) - 1.0*fey*(0.3*cos(q2) + 0.3)
0.98 - 2.0*fez
];
end

