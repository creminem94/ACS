classdef MyRobot < handle
    
    properties
        robot
        config
        DH
        links
        N %number of joints
        allT
        innerT
        B
        C
        G
        q
        dq
        ddq
        TAU
        TAU_RNE
        trajectoryPoint
        fwdDynDdq
        J
    end
    
    methods
        function obj = MyRobot(urdfFile, DH, links)
            obj.robot = importrobot(urdfFile);
            obj.config = homeConfiguration(obj.robot);
            obj.DH = DH;
            obj.N = length(links);
            obj.q = sym('q', [3 1], 'real');
            obj.dq = sym('dq', [3 1], 'real');
            obj.ddq = sym('ddq', [3 1], 'real');
            obj.computeDirectKinematics;
            obj.computeJ;
            obj.setLinks(links);
        end
        
        % setters
        
        function setSingleConfig(obj, jointIdx, jointValue)
            obj.config(jointIdx).JointPosition = jointValue;
        end
        
        function setAllConfig(obj,jointValues)
            for i=1:length(jointValues)
               obj.setSingleConfig(i, jointValues(i));
            end
        end
        
        function setLinks(obj, links)
            obj.links = links;
            for i=1:obj.N
                obj.links(i).setPosition(i, obj, obj.q(i), obj.dq(i), obj.ddq(i));
            end
            obj.computeLagrangian;
            obj.computeRNE;
        end
        
        % getters
        
        function names = frameNames(obj)
            names = obj.config.JointName;
        end
        
        function computeDirectKinematics(obj)
            [rows cols] = size(obj.DH);
            Ti = sym(zeros(4,4,rows));
            obj.innerT = sym(zeros(4,4,rows));
            for i=1:rows
                a = obj.DH(i, 1);
                alpha = obj.DH(i, 2);
                d = obj.DH(i, 3);
                theta = obj.DH(i, 4);
                T = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta)
                     sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta)
                     0 sin(alpha) cos(alpha) d
                     0 0 0 1];
                obj.innerT(:,:, i) = T;
                if i > 1
                    Ti(:,:,i) = Ti(:,:,i-1) * T;
                else 
                    Ti(:,:,i) = T;
                end
                %threshold = 1e-6;
                %Ti(i,:,:) = sym_matrix_round_zeros(Ti(i,:,:), threshold);
            end
            obj.allT = Ti;
        end
        
        function config = inverseKinematics(obj)
            [config1, config2] = ik;
            T = obj.allT(:,:,end);
            Px = T(1,4);
            Py = T(2,4);
            Pz = T(3,4);
            config = subs(config);
            config = obj.setValues(config);
        end
        
        function T = getTransform(obj, frameIdx)
            T = obj.allT(:,:,frameIdx);
        end
        
        function J = geometricJacobian(obj, frameIdx)
            totalCols = min([obj.N, frameIdx]);
            J = sym(zeros(6, totalCols));
            for i=1:totalCols
                type = obj.robot.Bodies{i}.Joint.Type;
                Z = obj.jacobianZ(i);
                if type == "prismatic"
                    col = [Z;0;0;0];
                else
                    P = obj.diffP(i, frameIdx);
                    C = cross(Z,P);
                    col = [C;Z];
                end
                J(:, i) = col;
            end
        end
        
        function computeJ(obj)
            obj.J = obj.geometricJacobian(5);
        end
        
        function t = Ta(obj)
            syms phi theta real;
            T = [0 -sin(phi) cos(phi)*sin(theta)
                0 cos(phi) sin(phi)*sin(theta)
                1 0 cos(theta)];
            t = [eye(3) zeros(3)
                  zeros(3) T];
        end
        
        function ja = Ja(obj)
            Ta = obj.Ta;
            ja = inv(Ta)*obj.J;
        end
        
        function Ic = inertiaCoM(obj)
            Ic = syms(zeros(3,3,obj.N));
            for i=1:obj.N
                Ic(:,:,i) = obj.links(i).inertiaCoM;
            end
        end
        
        function It = translatedInertia(obj)
            It = syms(zeros(3,3,obj.N));
            for i=1:obj.N
                It(:,:,i) = obj.links(i).translatedInertia;
            end
        end
       
        %print/plot methods
        
        function show(obj, figNum)
            if nargin == 2
                figure(figNum);
            else
                figure;
            end
            show(obj.robot,obj.config);
            xlim([-0.5 0.5]);
            ylim([-0.8 0.8]);
            zlim([0 0.8]);
        end
        
        function details(obj)
            showdetails(obj.robot)
        end
    
        function Z = jacobianZ(obj, frameIdx)
            T = obj.getTransform(frameIdx);
            Z = T(1:3,3);
        end
        function P = diffP(obj, frameIdx, refIdx)
            T1 = obj.getTransform(frameIdx);
            T2 = obj.getTransform(refIdx);
            P = T2(1:3, 4) - T1(1:3, 4);
        end
        
        function computeB(obj)
            B = sym(zeros(obj.N,obj.N));
            for i=1:obj.N
                l = obj.links(i);
                B = B + l.overallInertia;
            end
            obj.B = B;
        end
        function k = kineticEnergy(obj)
            k = simplify(0.5*obj.dq'*obj.B*obj.dq);
            k = vpa(k, 4);
        end
        function u = potentialEnergy(obj)
            u = 0;
            for i = 1:obj.N
                u = u + obj.links(i).potentialEnergy;
            end
            u = vpa(u, 4);
        end
        
        function cijk = CIJK(obj, i, j, k)
            B = obj.B;
            %dBij/dqk
            dk = diff(B(i,j), obj.q(k));
            %dBik/dqj
            dj = diff(B(i,k), obj.q(j));
            %dBjk/dqi
            di = diff(B(j,k), obj.q(i));
            cijk = (dk+dj-di)/2;
        end
        
        function cij = CIJ(obj, i, j)
            cij = 0;
            for k = i:obj.N
                cij = cij + obj.CIJK(i,j,k)*obj.dq(k);
            end
        end
        
        function computeC(obj)
            C = sym(zeros(obj.N,obj.N));
            for i = 1:obj.N
                for j = 1:obj.N
                    C(i, j) = obj.CIJ(i, j);
                end
            end
            obj.C = C;
        end
        
        function computeG(obj)
            G = sym(zeros(obj.N, 1));
            syms g real;
            g0 = [0;0;g];
            for i = 1:obj.N
                g = 0;
                for j = 1:obj.N
                    lj = obj.links(j);
                    Jp = lj.partialJacobian;
                    g = g + lj.mass*g0'*Jp(1:3, i);
                end
                G(j) = g;
            end
            obj.G = -G;
        end
        
        function computeLagrangian(obj)
            obj.computeB;
            obj.computeC;
            obj.computeG;
            obj.TAU = obj.B * obj.ddq + obj.C*obj.dq + obj.G;
            
            
        end
        
        function setTrajectoryPoint(obj, q, dq)
            obj.trajectoryPoint.q = q;
            if nargin == 2
                dq = zeros(obj.N, 1);
            end
            
            obj.trajectoryPoint.dq = dq;
        end
               
        function valueVar = setValues(obj, var, useJointPosition, useTrajectoryPoint)
            values_loader;
            if nargin == 3 && useJointPosition
                var = subs(var, obj.q, [obj.config.JointPosition]');
            end
            if nargin == 4 && useTrajectoryPoint
                var = subs(var, [obj.q; obj.dq], [obj.trajectoryPoint.q; obj.trajectoryPoint.dq]);
            end
            valueVar = subs(var);
            valueVar = vpa(valueVar, 4);
        end
        
        function T = toolboxT(obj)
            T = getTransform(obj.robot,obj.config,'ee','base_link');
        end
        
        function Jg = toolboxJg(obj)
            Jg = geometricJacobian(obj.robot,obj.config,'ee');
        end
        
        function ik = toolboxIk(obj)
            tform = obj.toolboxT;
            ik = inverseKinematics('RigidBodyTree', obj.robot);
            weights = [0.25 0.25 0.25 1 1 1];
            [configSoln,solnInfo] = ik('ee', tform, weights, obj.config);
            ik = [configSoln.JointPosition];
        end
        
        function computeRNE(obj)
            obj.TAU_RNE = sym(zeros(3, 1));
            for i = 1:obj.N
                j = obj.N - i + 1;
                obj.TAU_RNE(j) = obj.links(j).tauI;
            end
            tau = sym('tau', [obj.N 1]);
            syms fex fey fez uex uey uez real;
            he = [fex fey fez uex uey uez]';
            obj.fwdDynDdq = inv(obj.B_RNE) * (tau - obj.C_RNE - obj.G_RNE - obj.J'*he);
        end
        
        function ddq = getFwdDynDdq(obj)
            syms fex fey fez uex uey uez real;
            he = [fex fey fez uex uey uez]';
            tau = sym('tau', [obj.N 1]);
            ddq = inv(obj.B) * (tau - obj.C*obj.dq - obj.G_RNE - obj.J'*he);
        end
        
        function G = G_RNE(obj)
            tau = obj.TAU_RNE;
            dq1 = 0; dq2 = 0; dq3 = 0; ddq1 = 0; ddq2 = 0; ddq3 = 0;
            G = subs(tau);
        end
        
        function C = C_RNE(obj)
            tau = obj.TAU_RNE;
            ddq1 = 0; ddq2 = 0; ddq3 = 0; g = 0;
            C = subs(tau);
        end
        
        function B = B_RNE(obj)
            tau = obj.TAU_RNE;
            B = sym(zeros(obj.N, obj.N));
            dq1 = 0; dq2 = 0; dq3 = 0; g = 0;
            fex = 0;fey = 0;fez = 0;
            uex = 0;uey = 0;uez = 0;
            for i = 1:obj.N
                ddq1 = 0; ddq2 = 0; ddq3 = 0;
                eval(strcat('ddq',num2str(i), '=1;')); %ddqi = 1;
                B(1:obj.N, i) = subs(tau);
            end
        end
        
        %operationalSpace
        
        function B = Ba(obj)
            Ja = obj.Ja;
            B = pinv(Ja')*obj.B*pinv(Ja);
        end
        
        function C = Ca_xd(obj)
            Ja = obj.Ja;
            dJa = diff(Ja);
            C = pinv(Ja')*obj.C*obj.dq-obj.Ba*dJa*obj.dq;
        end
        
        function G = Ga(obj)
            G = pinv(obj.Ja')*obj.G;
        end
        
        
    end
end

