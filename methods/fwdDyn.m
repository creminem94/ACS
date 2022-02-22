function ddq = fwdDyn(q, dq, tau)
%myRobot.setValues(myRobot.fwdDyn) 
q2 = q(2); dq2 = dq(2);
tau1 = tau(1);tau2 = tau(2);tau3 = tau(3);

%notdiced a difference only in compliance control along y direction
%rne
% ddq = [
% (6.0*(0.06*cos(q2) - 0.18*cos(q2)^2 + 0.18*sin(q2)^2)*(tau2 + 0.0045*dq2^2*sin(q2) + 0.3*sin(q2)*(0.03*dq2^2 - 0.09*dq2^2*cos(q2)) + 0.09*dq2^2*sin(q2)*(0.3*cos(q2) - 0.15)))/(0.1564*cos(q2)^2 - 0.0648*cos(q2) + 0.1672*sin(q2)^2 - 0.1296*cos(q2)*sin(q2)^2 + 0.3888*cos(q2)^2*sin(q2)^2 + 0.06997) + ((tau1 - 1.0*sin(q2)*(0.03*dq2^2 - 0.09*dq2^2*cos(q2)) + 0.09*dq2^2*cos(q2)*sin(q2))*(0.324*cos(q2)^2 - 0.216*cos(q2) + 0.324*sin(q2)^2 + 0.2332))/(0.1564*cos(q2)^2 - 0.0648*cos(q2) + 0.1672*sin(q2)^2 - 0.1296*cos(q2)*sin(q2)^2 + 0.3888*cos(q2)^2*sin(q2)^2 + 0.06997)
%                   (6.0*(tau1 - 1.0*sin(q2)*(0.03*dq2^2 - 0.09*dq2^2*cos(q2)) + 0.09*dq2^2*cos(q2)*sin(q2))*(0.06*cos(q2) - 0.18*cos(q2)^2 + 0.18*sin(q2)^2))/(0.1564*cos(q2)^2 - 0.0648*cos(q2) + 0.1672*sin(q2)^2 - 0.1296*cos(q2)*sin(q2)^2 + 0.3888*cos(q2)^2*sin(q2)^2 + 0.06997) + (12.0*(0.3*cos(q2)^2 + 0.3*sin(q2)^2 + 0.3)*(tau2 + 0.0045*dq2^2*sin(q2) + 0.3*sin(q2)*(0.03*dq2^2 - 0.09*dq2^2*cos(q2)) + 0.09*dq2^2*sin(q2)*(0.3*cos(q2) - 0.15)))/(0.1564*cos(q2)^2 - 0.0648*cos(q2) + 0.1672*sin(q2)^2 - 0.1296*cos(q2)*sin(q2)^2 + 0.3888*cos(q2)^2*sin(q2)^2 + 0.06997)
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       10.0*tau3 - 9.8
% ];


%lag
ddq = [
((0.06*sin(q2)*dq2^2 + tau1)*(0.162*cos(q2)^2 + 0.162*sin(q2)^2 + 0.1792))/(0.054*cos(q2)^2 + 0.0972*sin(q2)^2 + 0.1075) - (0.72*tau2*cos(q2))/(0.054*cos(q2)^2 + 0.0972*sin(q2)^2 + 0.1075)
                                         (7.2*tau2)/(0.054*cos(q2)^2 + 0.0972*sin(q2)^2 + 0.1075) - (0.72*cos(q2)*(0.06*sin(q2)*dq2^2 + tau1))/(0.054*cos(q2)^2 + 0.0972*sin(q2)^2 + 0.1075)
                                                                                                                                                                             10.0*tau3 - 9.8    
];

end