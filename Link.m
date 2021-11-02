classdef Link < handle
    %LINK Summary of this class goes here
    %   Detailed explanation goes here
    properties
        type %box or cyl
        mass
        a
        b
        c_h
        tvec
        pli %position of CoM wrt base frame
        pj %position of the previous joint wrt base frame
        idx %index of the joint
        robot
    end
    
    methods
        function obj = Link(type, mass, a, b, c_h, tvec)
            % type ("box" or "cyl")
            % box case: a(length) b(height) c_h(c,width)
            % cyl case: a(outer radius) b(inner radius) c_h(h,length)
            % tvec (translation vector)
            obj.type = type;
            obj.mass = mass;
            obj.a = a;
            obj.b = b;
            obj.c_h = c_h;
            obj.tvec = tvec;
        end
        
        function Ic = inertiaCoM(obj)
            %Inertia matrix wrt center of mass
            m = obj.mass;
            c = obj.c_h;
            h = obj.c_h;
            if obj.type == "box"
                Ic = [1/12*m*(obj.b^2+c^2)
                      1/12*m*(obj.a^2+c^2)
                      1/12*m*(obj.a^2+obj.b^2)];
            else
                Ic = [1/2*m*(obj.a^2+obj.b^2)
                      1/2*m*(3*(obj.a^2+obj.b^2)+h^2)
                      1/2*m*(3*(obj.a^2+obj.b^2)+h^2)];
            end
            Ic = diag(Ic);
        end
        
        function It = translatedInertia(obj)
           It = obj.inertiaCoM + obj.mass*(obj.tvec'*obj.tvec*eye(3)-obj.tvec*obj.tvec');
        end
        
        function setPosition(obj, idx, robot)
            T = robot.getTransform(idx+1);
            R = T(1:3,1:3);
            obj.pli = R*obj.tvec+T(1:3, 4);
            obj.pj = T(1:3, 4);
            obj.idx = idx;
            obj.robot = robot;
        end
        
        function Jp = partialJacobian(obj)
            Jp = sym(zeros(6, length(obj.robot.links)));
            for i=1:obj.idx
                Z = obj.robot.jacobianZ(i);
                if obj.type == "box"
                    col = [Z;0;0;0];
                else
                    C = cross(Z, obj.pli-obj.pj);
                    col = [C;Z];
                end
                Jp(:, i) = col;
            end
        end
        
        function u = potentialEnergy(obj)
            syms g real;
            g0 = [0;0;g];
            u = -obj.mass*g0'*obj.pli;
        end
        
        function B = overallInertia(obj)
            m = obj.mass;
            J = obj.partialJacobian;
            Jp = J(1:3, :);
            Jo = J(4:6, :);
            T = obj.robot.getTransform(obj.idx+1);
            R = T(1:3,1:3);
            B = m*(Jp'*Jp)+Jo'*R*obj.translatedInertia*R'*Jo;
        end
    end
end

