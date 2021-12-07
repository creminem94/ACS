function ddq = fwdDyn(q, dq, tau, he)
%myRobot.setValues(myRobot.fwdDynDdq) 
q1 = q(1);q2 = q(2);q3 = q(3);
dq1 = dq(1);dq2 = dq(2);dq3 = dq(3);
fex = he(1);fey = he(2);fez = he(3);
uex = he(4);uey = he(5);uez = he(6);
tau1 = tau(1);tau2 = tau(2);tau3 = tau(3);

ddq = [
((0.06*sin(q2)*dq2^2 - 1.0*fey + tau1)*(0.162*cos(q2)^2 + 0.162*sin(q2)^2 + 0.1633))/(0.054*cos(q2)^2 + 0.0972*sin(q2)^2 + 0.09798) - (0.72*cos(q2)*(tau2 - 1.0*uez - 0.3*fey*cos(q2) + 0.3*fex*sin(q2)))/(0.054*cos(q2)^2 + 0.0972*sin(q2)^2 + 0.09798)
(7.2*(tau2 - 1.0*uez - 0.3*fey*cos(q2) + 0.3*fex*sin(q2)))/(0.054*cos(q2)^2 + 0.0972*sin(q2)^2 + 0.09798) - (0.72*cos(q2)*(0.06*sin(q2)*dq2^2 - 1.0*fey + tau1))/(0.054*cos(q2)^2 + 0.0972*sin(q2)^2 + 0.09798)
10.0*tau3 - 10.0*fez - 9.8    
];

end