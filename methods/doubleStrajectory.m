function [traj,separators] = doubleStrajectory(qiH,qfH,dqiH,dqfH,dqmH,ddqmH,dddqmH,ti,tf,alpha,beta,Ts)

    %sign normalization
    sigma = 1;
    if (qfH < qiH)
        sigma = -1; 
    end
    qi = sigma*qiH;
    qf = sigma*qfH;
    dqi = sigma*dqiH;
    dqf = sigma*dqfH;
    dqm = (sigma+1)/2*dqmH-(sigma-1)/2*dqmH;
    ddqm = (sigma+1)/2*ddqmH-(sigma-1)/2*ddqmH;
    dddqm = (sigma+1)/2*dddqmH-(sigma-1)/2*dddqmH;
    
    if isnan(tf) 
        %check feasbility
        diffV = abs(dqf-dqi);
        rapVel = sqrt(diffV/dddqm);
        rapAcc = ddqm/dddqm;
        tj = min(rapVel,rapAcc);
        fprintf("tj*: %f\n", tj);
        if tj == rapAcc
            fprintf("tj* corresponds to acceleration term\n");
            checkCondition = 0.5*(dqi+dqf)*(tj+diffV/dddqm);
        else 
            fprintf("tj* corresponds to velocity term\n");
            checkCondition = tj*(dqi+dqf);
        end

        if qf-qi < checkCondition
            error("Trajectory is not feasible: delta q is smaller than check condition(%f)",checkCondition);
        end

        %case1 dqLim = dqm
        fprintf("Assuming dqLim = dqmax\n");
        dqLim = dqm;
        ddqmReached = (dqm-dqi)*dddqm >= ddqm^2;
        if ~ddqmReached
            fprintf("Max acceleration not reached\n");
            Tj1 = sqrt((dqm - dqi)/dddqm);
            ta = 2*Tj1;
            ddqaLim = dddqm*Tj1;
        else
            fprintf("max acceleration reached\n");
            Tj1 = ddqm/dddqm;
            ta = Tj1 + (dqm - dqi)/ddqm;
            ddqaLim = ddqm;
        end

        ddqMinReached = (dqm-dqf)*dddqm >= ddqm^2;
        if ~ddqMinReached
            fprintf("Min acceleration not reached\n");
            Tj2 = sqrt((dqm-dqf)/dddqm);
            td = 2*Tj2;
            ddqdLim = -dddqm*Tj2;
        else
            fprintf("Min acceleration reached\n");
            Tj2 = ddqm/dddqm;
            td = Tj2+(dqm-dqf)/ddqm;
            ddqdLim = -ddqm;
        end

        tv = (qf-qi)/dqm-ta/2*(1+dqi/dqm)-td/2*(1+dqf/dqm);

        %case 2 dqLim < dqMax
        if tv <= 0
            fprintf("tv less then 0 (%f), dqLim < dqmax\n",tv);
            fprintf("Assuming max/min acceleration reached\n");
            Tj = ddqm/dddqm;
            Tj1 = Tj;
            Tj2 = Tj;
            D = ddqm^4/dddqm^2+2*(dqi^2+dqf^2)+ddqm*(4*(qf-qi)-2*ddqm/dddqm*(dqi+dqf));
            ta = (ddqm^2/dddqm-2*dqi+sqrt(D))/(2*ddqm);
            td = (ddqm^2/dddqm-2*dqf+sqrt(D))/(2*ddqm);
            if ta < 2*Tj
                if ta <= 0
                    ta = 0;
                    Tj1 = 0;
                else
                    fprintf("Max acceleretion is not reached\n");
                    fprintf("ta:%f,Tj:%f\n",ta,Tj);
                    error("decrease max acceleration");
                end
            end
            if td < 2*Tj
                if td <= 0
                    td = 0;
                    Tj2 = 0;
                else
                    fprintf("Min acceleretion is not reached\n");
                    fprintf("td:%f,Tj:%f\n",td,Tj);
                    error("decrease max acceleration");
                end
            end
            ddqaLim = dddqm*Tj;
            ddqdLim = -dddqm*Tj;
            dqLim = dqi+(ta-Tj1)*ddqaLim;
            tv = 0;
        end

        dddqMin = -dddqm;

        tf = ta+tv+td+ti;
        deltaT = tf-ti;
    else 
        %deltaT given
        deltaT = tf-ti;
        ta = alpha*deltaT;
        td = ta;
        Tj1 = beta*ta;
        Tj2 = Tj1;
        deltaQ = qf-qi;
        dqLim = deltaQ/((1-alpha)*deltaT);
        ddqaLim = deltaQ/(alpha*(1-alpha)*(1-beta)*deltaT^2);
        ddqdLim = -ddqaLim;
        dddqm = deltaQ/(alpha^2*beta*(1-alpha)*(1-beta)*deltaT^3);
        dddqMin = -dddqm;
        tv = deltaT-ta-td;
    end
    
    fprintf("final params:\n");
    fprintf("Tj1:%f\nTj2:%f\nta:%f\ntd:%f\ntv:%f\ndqMax:%f\ndqLim:%f\n",Tj1,Tj2,ta,td,tv,dqm,dqLim);
    fprintf("ddqMax:%f\nddqaLim:%f\nddqdLim:%f\n",ddqm,ddqaLim,ddqdLim);
    fprintf("dddqMax:%f\ndddqMin:%f\n",dddqm,dddqMin);
    
    separators = [];
    
    %acceleration phase
    ta1 = ti:Ts:Tj1+ti;
    tVar = ta1-ti;
    qa1 = qi+dqi*tVar+dddqm*tVar.^3/6;
    dqa1 = dqi+dddqm*tVar.^2/2;
    ddqa1 = dddqm*tVar;
    dddqa1 = ones(1,length(ta1))*dddqm;
    fprintf("range a1[%f,%f]\n",ti,Tj1+ti);
    
    separators(end+1) = Tj1+ti;
    
    ta2 = Tj1+ti+Ts:Ts:ta-Tj1+ti;
    tVar = ta2-ti;
    qa2 = qi+dqi*tVar+ddqaLim*(3*tVar.^2-3*Tj1*tVar+Tj1^2)/6;
    dqa2 = dqi+ddqaLim*(tVar-Tj1/2);
    ddqa2 = ones(1,length(ta2))*ddqaLim;
    dddqa2 = zeros(1,length(ta2));
    fprintf("range a2[%f,%f]\n",Tj1+ti+Ts,ta-Tj1+ti);
    
    separators(end+1) = ta-Tj1+ti;
    
    ta3 = ta-Tj1+ti+Ts:Ts:ta+ti;
    tVar = ta3-ti;
    qa3 = qi+(dqLim+dqi)*ta/2-dqLim*(ta-tVar)-dddqMin*(ta-tVar).^3/6;
    dqa3 = dqLim+dddqMin*(ta-tVar).^2/2;
    ddqa3 = -dddqMin*(ta-tVar);
    dddqa3 = ones(1,length(ta3))*dddqMin;
    fprintf("range a3[%f,%f]\n",ta-Tj1+ti+Ts,ta+ti);
    
    separators(end+1) = ta+ti;
    
    %TODO check if there are problems when tv=0
    %constant phase
    tc = ta+ti+Ts:Ts:ta+tv+ti;
    tVar = tc-ti;
    qc = qi+(dqLim+dqi)*ta/2+dqLim*(tVar-ta);
    dqc = ones(1,length(tc))*dqLim;
    ddqc = zeros(1,length(tc));
    dddqc = zeros(1,length(tc));
    fprintf("range c[%f,%f]\n",ta+ti+Ts,ta+tv+ti);
    
    separators(end+1) = ta+tv+ti;
    
    %deceleration phase 
    
    td1 = deltaT-td+ti+Ts:Ts:deltaT-td+Tj2+ti;
    tVar = td1-ti;
    qd1 = qf-(dqLim+dqf)*td/2+dqLim*(tVar-deltaT+td)-dddqm*(tVar-deltaT+td).^3/6;
    dqd1 = dqLim-dddqm*(tVar-deltaT+td).^2/2;
    ddqd1 = -dddqm*(tVar-deltaT+td);
    dddqd1 = ones(1,length(td1))*dddqMin;
    fprintf("range d1[%f,%f]\n",deltaT-td+ti+Ts,deltaT-td+Tj2+ti);
    
    separators(end+1) = deltaT-td+Tj2+ti;
    
    td2 = deltaT-td+Tj2+Ts+ti:Ts:deltaT-Tj2+ti;
    tVar = td2-ti;
    qd2 = qf-(dqLim+dqf)*td/2+dqLim*(tVar-deltaT+td)+ddqdLim/6*(3*(tVar-deltaT+td).^2-3*Tj2*(tVar-deltaT+td)+Tj2^2);
    dqd2 = dqLim+ddqdLim*(tVar-deltaT+td-Tj2/2);
    ddqd2 = ones(1,length(td2))*ddqdLim;
    dddqd2 = zeros(1,length(td2));
    fprintf("range d2[%f,%f]\n",deltaT-td+Tj2+Ts+ti,deltaT-Tj2+ti);
    
    separators(end+1) = deltaT-Tj2+ti;
    
    td3 = deltaT-Tj2+Ts+ti:Ts:deltaT+ti;
    tVar = td3-ti;
    qd3 = qf-dqf*(deltaT-tVar)-dddqm*(deltaT-tVar).^3/6;
    dqd3 = dqf+dddqm*(deltaT-tVar).^2/2;
    ddqd3 = -dddqm*(deltaT-tVar);
    dddqd3 = ones(1,length(td3))*dddqm;
    fprintf("range d3[%f,%f]\n",deltaT-Tj2+Ts+ti,deltaT+ti);
    
    traj.t = [ta1,ta2,ta3,tc,td1,td2,td3];
    traj.q = sigma*[qa1,qa2,qa3,qc,qd1,qd2,qd3];
    traj.dq = sigma*[dqa1,dqa2,dqa3,dqc,dqd1,dqd2,dqd3];
    traj.ddq = sigma*[ddqa1,ddqa2,ddqa3,ddqc,ddqd1,ddqd2,ddqd3];
    traj.dddq = sigma*[dddqa1,dddqa2,dddqa3,dddqc,dddqd1,dddqd2,dddqd3];
end