classdef MyRobot < handle
    
    properties
        robot
        config
        DH
        links
        N %number of joints
    end
    
    methods
        function obj = MyRobot(urdfFile, DH)
            obj.robot = importrobot(urdfFile);
            obj.config = homeConfiguration(obj.robot);
            obj.DH = DH;
        end
        
        % setters
        
        function setSingleConfig(obj, jointIdx, jointValue)
            obj.config(jointIdx).JointPosition = jointValue;
        end
        
        function setAllConfig(obj,jointValues)
            for i=1:length(jointValues)
               obj.setSingleConfig(i, jointValues(i)) 
            end
        end
        
        function setLinks(obj, links)
            obj.links = links;
            obj.N = length(links);
            for i=1:obj.N
                obj.links(i).setPosition(i, obj);
            end
        end
        
        % getters
        
        function names = frameNames(obj)
            names = obj.config.JointName;
        end
        
        function Ti = directKinematics(obj)
            [rows cols] = size(obj.DH);
            Ti = sym(zeros(4,4,rows));
            for i=1:rows
                a = obj.DH(i, 1);
                alpha = obj.DH(i, 2);
                d = obj.DH(i, 3);
                theta = obj.DH(i, 4);
                T = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta)
                     sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta)
                     0 sin(alpha) cos(alpha) d
                     0 0 0 1];

                if i > 1
                    Ti(:,:,i) = Ti(:,:,i-1) * T;
                else 
                    Ti(:,:,i) = T;
                end
                %threshold = 1e-6;
                %Ti(i,:,:) = sym_matrix_round_zeros(Ti(i,:,:), threshold);
            end
        end
        
        function T = getTransform(obj, frameIdx)
            Ti = obj.directKinematics();
            T = Ti(:,:,frameIdx);
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
        function Jp = partialJacobian(obj)
            Jp = sym(zeros(6,obj.N, obj.N));
            for i=1:obj.N
                Jp(:,:,i) = obj.links(i).partialJacobian(obj, i);
            end
        end
        function B = overallInertia(obj)
            B = sym(zeros(obj.N,obj.N));
            for i=1:obj.N
                l = obj.links(i);
                B = B + l.overallInertia;
            end
        end
        function k = kineticEnergy(obj, dq)
            k = 0.5*dq'*obj.overallInertia*dq;
            k = vpa(k, 4);
        end
        function u = potentialEnergy(obj)
            u = 0;
            for i = 1:obj.N
                u = u + obj.links(i).potentialEnergy;
            end
            u = vpa(u, 4);
        end
        
        function cijk = CIJK(obj, i, j, k, q)
            B = obj.overallInertia;
            %dBij/dqk
            dk = diff(B(i,j), q(k));
            %dBik/dqj
            dj = diff(B(i,k), q(j));
            %dBjk/dqi
            di = diff(B(j,k), q(i));
            cijk = dk+dj-di;
        end
        
        function cij = CIJ(obj, i, j, q, dq)
            cij = 0;
            for k = i:obj.N
                cij = cij + obj.CIJK(i,j,k, q)*dq(k);
            end
        end
        
        function tauI = TAUi(obj, i, q, dq, ddq)
            syms g real;
            g0 = [0;0;g];
            tauI = 0;
            B = obj.overallInertia;            
            for j = 1:obj.N
                lj = obj.links(j);
                Jp = lj.partialJacobian;
                g = lj.mass*g0'*Jp(1:3, i);
                tauI = tauI + B(i,j)*ddq(j) + obj.CIJ(i, j, q, dq)*dq(j) + g;
            end
        end
        
        function valueVar = setValues(obj, var)
            values_loader;
            for i = 1:obj.N
                i_s = num2str(i);
                eval(strcat('q',i_s, '=obj.config(',i_s,').JointPosition;'));
            end
            valueVar = subs(var);
        end
        
        function toolboxT(obj)
            getTransform(obj.robot,obj.config,'ee','base_link')
        end
        
        function toolboxJg(obj)
            geometricJacobian(obj.robot,obj.config,'ee')
        end
    end
end

