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
        qi
        dqi
        ddqi
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
%                 Ic = [1/12*m*(obj.b^2+c^2)
%                       1/12*m*(obj.a^2+c^2)
%                       1/12*m*(obj.a^2+obj.b^2)];
               Ic = [1/12*m*(obj.b^2+obj.a^2)
                  1/12*m*(obj.a^2+c^2)
                  1/12*m*(obj.b^2+c^2)];
            else
                Ic = [1/2*m*(obj.a^2+obj.b^2)
                      1/2*m*(3*(obj.a^2+obj.b^2)+h^2)
                      1/2*m*(3*(obj.a^2+obj.b^2)+h^2)];
            end
            Ic = diag(Ic);
        end
        
        function It = translatedInertia(obj)
            T = obj.robot.getTransform(obj.idx);
            r = -T(1:3,1:3)'*obj.tvec;
            It = obj.inertiaCoM + obj.mass*(r'*r*eye(3)-r*r');
        end
        
        function setPosition(obj, idx, robot, qi, dqi, ddqi)
            T = robot.getTransform(idx+1); %CoM is computed from the end of the link
            R = T(1:3,1:3);
            obj.pli = R*obj.tvec+T(1:3, 4);
            T2 = robot.getTransform(idx);
            obj.pj = T2(1:3, 4); %pj-1
            obj.idx = idx;
            obj.robot = robot;
            obj.qi = qi;
            obj.dqi = dqi;
            obj.ddqi = ddqi;
        end
        
        function Jp = partialJacobian(obj)
            Jp = sym(zeros(6, obj.robot.N));
            for i=1:obj.idx
                Z = obj.robot.jacobianZ(i);
                if obj.robot.links(i).type == "box"
                    col = [Z;0;0;0];
                else
                    C = cross(Z, obj.pli-obj.robot.links(i).pj);
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
        
        function R = Rim1_i(obj)
            T = obj.robot.innerT(:,:,obj.idx+1);
            R = T(1:3,1:3);
        end
        
        function R = Ri_ip1(obj)
            T = obj.robot.innerT(:,:,obj.idx+2);
            R = T(1:3,1:3);
        end
        
        function r = ri_im1_i(obj)
            T = obj.robot.innerT(:,:,obj.idx+1);
            r = T(1:3, 4);
        end
        
        function r = ri_ic_i(obj)
            r = -obj.tvec;
        end
        
        function l = prevLink(obj)
            l = obj.robot.links(obj.idx-1);
        end
        
        function l = nextLink(obj)
            l = obj.robot.links(obj.idx+1);
        end
        
        function w = w_im1_im1(obj)
            if obj.idx == 1
                w = [0 0 0]';
            else
                w = obj.prevLink.wi_i;
            end
        end
        
        function w = wi_i(obj)
            R = obj.Rim1_i;
            w = R'*obj.w_im1_im1;
            if obj.type == "cyl"
                w = w + R'*obj.dqi*[0;0;1];
            end
        end
        
        function dw = dw_im1_im1(obj)
            if obj.idx == 1
                dw = [0;0;0];
            else
                dw = obj.prevLink.dwi_i;
            end
        end
        
        function dw = dwi_i(obj)
            R = obj.Rim1_i;
            dw = R'*obj.dw_im1_im1;
            if obj.type == "cyl"
                dw = R'*(obj.ddqi*[0;0;1]+cross(obj.dqi*obj.w_im1_im1, [0;0;1]));
            end
        end
        
        function d = ddpim1_im1(obj)
            syms g real;
            baseT = obj.robot.getTransform(obj.idx);
            R = baseT(1:3,1:3);
            
            if obj.idx == 1
                d = R*[0; 0; g]; %how gravity is oriented on first joint
            else
                d = obj.prevLink.ddpi_i;
            end
        end
        
        function o = ddpi_i(obj)
            r = obj.ri_im1_i;
            R = obj.Rim1_i;
            o = R'*obj.ddpim1_im1+cross(obj.dwi_i, r);
            o = o + cross(obj.wi_i, cross(obj.wi_i, r));
            if obj.type == "box"
                o = o + R'*obj.ddqi*[0;0;1];
                o = o + cross(2*obj.dqi*obj.wi_i, R'*[0;0;1]);
            end
        end
        
        function o = ddpi_ci(obj)
            o = obj.ddpi_i+cross(obj.dwi_i,obj.ri_ic_i)+cross(obj.wi_i, cross(obj.wi_i, obj.ri_ic_i));
        end
        
        function f = fip1_ip1(obj)
            syms fex fey fez real;
            if obj.idx == obj.robot.N
%                 f = sym([fex; fey; fez]);
                f = zeros(3,1);
            else
                f = obj.nextLink.fi_i;
            end
        end
        
        function u = uip1_ip1(obj)
            syms uex uey uez real;
            if obj.idx == obj.robot.N
%                 u = sym([uex;uey;uez]);
                u = zeros(3,1);
            else
                u = obj.nextLink.ui_i;
            end
        end
        
        function f = fi_i(obj)
            f = obj.Ri_ip1*obj.fip1_ip1+obj.mass*obj.ddpi_ci;
        end
        
        function u = ui_i(obj)
            R = obj.Ri_ip1;
            I = obj.translatedInertia;
            u = cross(-obj.fi_i, obj.ri_im1_i+obj.ri_ic_i) + R*obj.uip1_ip1;
            u = u + cross(R*obj.fip1_ip1, obj.ri_ic_i) + I*obj.dwi_i;
            u = u + cross(obj.wi_i, I*obj.wi_i);
        end
        
        function tau = tauI(obj)
            if obj.type == "box"
                f = obj.fi_i;
            else
                f = obj.ui_i;
            end
            tau = f'*obj.Rim1_i'*[0;0;1];
            
            %missing motor dynamics and frictions
        end
    end
end

