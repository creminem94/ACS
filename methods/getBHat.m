function B = getBHat(q)
%myRobot.B_RNE
q1 = q(1);q2 = q(2);q3 = q(3);
he = zeros(6,1);
fex = he(1);fey = he(2);fez = he(3);
uex = he(4);uey = he(5);uez = he(6);
B = [
sin(q2)*(fex + 0.6*sin(q2)) - 1.0*cos(q2)*(fey - 0.3*cos(q2)) + 0.3, cos(q2)*(0.1*cos(q2) - 1.0*fey + 0.06) + sin(q2)*(fex - 0.09*sin(q2)),fex*sin(q2) - 1.0*fey*cos(q2)
0.*fey - 1.0*uez - 0.06*cos(q2) - 0.3*sin(q2)*(fex + 0.3*sin(q2)) - 1.0*(0.3*cos(q2) + 0.3)*(fey - 0.3*cos(q2)), 0.3*fey - 1.0*uez - 0.009*cos(q2) - 0.3*sin(q2)*(fex - 0.09*sin(q2)) + (0.5*cos(q2) + 0.5)*(0.09*cos(q2) - 1.0*fey + 0.06) + 0.01361, 0.3*fey - 1.0*uez - 0.3*fex*sin(q2) - 1.0*fey*(0.3*cos(q2) + 0.3)
-1.0*fez,-1.0*fez,0.1 - 1.0*fez
];
end

