function q = linearTrajectory(qi,qf,ti,tf,Ts)
    deltaT = tf-ti;
    a0 = qi;
    a1 = (qf-qi)/deltaT;
    t = ti:Ts:tf;
    q = a0+a1*(t-ti);
end

