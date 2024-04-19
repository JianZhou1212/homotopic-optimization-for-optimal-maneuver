classdef DriverModel < handle

    properties (SetAccess = public)
        lf;
        lr;
        l;
        w;
        l_x1;
        l_x2;
        l_x3;
        l_x4;
        l_y1;
        l_y3;
        l_y2;
        l_y4;
        m;
        Izz;
        g;
        mu_y1;
        mu_y2;
        mu_y3;
        mu_y4;
        B_y1;
        B_y2;
        B_y3;
        B_y4;
        C_y1;
        C_y2;
        C_y3;
        C_y4;
        E_y1;
        E_y2;
        E_y3;
        E_y4;
        F_z1;
        F_z2;
        F_z3;
        F_z4;
        F_x01_max;
        F_x02_max;
        F_x03_max;
        F_x04_max;
        N;
        p;
        delta_min;
        delta_dot_min;
        delta_max;
        delta_dot_max;
        Xa;
        Ya;
        R_xi;
        R_xo;
        R_yi;
        R_yo;
        Xp0;
        Yp0;
        phi0;
        vx0;
        vy0;
        r0;
        wf0;
        wr0;
        del0;
        X_tf;
        Y_tf;
        D_control;
        D_state;
        ref_x;
        ref_y;
        ref_heading;
        ref_curvature;
        ref_steering;
        s0;
        vnorm;
        plan;
    end
    
    methods (Access = public)
        function obj = DriverModel(parameters, path)
            obj.lf = parameters.lf;
            obj.lr = parameters.lr;
            obj.l = parameters.lr;
            obj.w = parameters.w;
            obj.l_x1 = parameters.lf;
            obj.l_x2 = parameters.lf;
            obj.l_x3 = -parameters.lr;
            obj.l_x4 = -parameters.lr;
            obj.l_y1 = parameters.w/2;
            obj.l_y3 = parameters.w/2;
            obj.l_y2 = -parameters.w/2;
            obj.l_y4 = -parameters.w/2;
            obj.m = parameters.m;
            obj.Izz = parameters.Izz;
            obj.g = parameters.g;
            obj.mu_y1 = parameters.mu_y1;
            obj.mu_y2 = parameters.mu_y2;
            obj.mu_y3 = parameters.mu_y3;
            obj.mu_y4 = parameters.mu_y4;
            obj.B_y1 = parameters.B_y1;
            obj.B_y2 = parameters.B_y2;
            obj.B_y3 = parameters.B_y3;
            obj.B_y4 = parameters.B_y4;
            obj.C_y1 = parameters.C_y1;
            obj.C_y2 = parameters.C_y2;
            obj.C_y3 = parameters.C_y3;
            obj.C_y4 = parameters.C_y4;
            obj.E_y1 = parameters.E_y1;
            obj.E_y2 = parameters.E_y2;
            obj.E_y3 = parameters.E_y3;
            obj.E_y4 = parameters.E_y4;
            obj.F_z1 = parameters.F_z1;
            obj.F_z2 = parameters.F_z2;
            obj.F_z3 = parameters.F_z3;
            obj.F_z4 = parameters.F_z4;
            obj.F_x01_max = parameters.F_x01_max;
            obj.F_x02_max = parameters.F_x02_max;
            obj.F_x03_max = parameters.F_x03_max;
            obj.F_x04_max = parameters.F_x04_max;
            obj.N = parameters.N;
            obj.p = parameters.p;
            obj.delta_min = parameters.delta_min;
            obj.delta_dot_min = parameters.delta_dot_min;
            obj.delta_max = parameters.delta_max;
            obj.delta_dot_max = parameters.delta_dot_max;
            obj.Xa = parameters.Xa;
            obj.Ya = parameters.Ya;
            obj.R_xi = parameters.R_xi;
            obj.R_xo = parameters.R_xo;
            obj.R_yi = parameters.R_yi;
            obj.R_yo = parameters.R_yo;
            obj.Xp0 = parameters.Xp0;
            obj.Yp0 = parameters.Yp0;
            obj.phi0 = parameters.phi0;
            obj.vx0 = parameters.vx0;
            obj.vy0 = parameters.vy0;
            obj.r0 = parameters.r0;
            obj.wf0 = parameters.wf0;
            obj.wr0 = parameters.wr0;
            obj.del0 = parameters.del0;
            obj.X_tf = parameters.X_tf;
            obj.Y_tf = parameters.Y_tf;
            obj.D_control = parameters.D_control;
            obj.D_state = parameters.D_state;
            obj.ref_x         = parameters.ref_x;
            obj.ref_y         = parameters.ref_y;
            obj.ref_heading   = parameters.ref_heading;
            obj.ref_curvature = parameters.ref_curvature;
            obj.ref_steering  = parameters.ref_steering;
            obj.s0 = parameters.s0;
            obj.vnorm = parameters.vnorm;
            obj.plan = path;
           

        end

        function x_dot = vehicle_model(obj, x, u)

            phi = x(3);
            r   = x(4);
            vx  = x(5);
            vy  = x(6);

            F_x1  = u(1);
            F_x2  = u(2);
            F_x3  = u(3);
            F_x4  = u(4);
            delta = u(5);

            delta_1 = delta;
            delta_2 = delta;
            [delta_3, delta_4] = deal(0);

            alpha_1 = delta_1 - atan((vy + obj.lf*r)/vx);
            alpha_2 = delta_2 - atan((vy + obj.lf*r)/vx);
            alpha_3 = -atan((vy - obj.lr*r)/vx);
            alpha_4 = -atan((vy - obj.lr*r)/vx);
            
            F_y01 = obj.mu_y1*obj.F_z1*sin(obj.C_y1*atan(obj.B_y1*alpha_1 - obj.E_y1*(obj.B_y1*alpha_1 - atan(obj.B_y1*alpha_1))));
            F_y02 = obj.mu_y2*obj.F_z2*sin(obj.C_y2*atan(obj.B_y2*alpha_2 - obj.E_y2*(obj.B_y2*alpha_2 - atan(obj.B_y2*alpha_2))));
            F_y03 = obj.mu_y3*obj.F_z3*sin(obj.C_y3*atan(obj.B_y3*alpha_3 - obj.E_y3*(obj.B_y3*alpha_3 - atan(obj.B_y3*alpha_3))));
            F_y04 = obj.mu_y4*obj.F_z4*sin(obj.C_y4*atan(obj.B_y4*alpha_4 - obj.E_y4*(obj.B_y4*alpha_4 - atan(obj.B_y4*alpha_4))));
            
            F_y1 = F_y01*sqrt(1 - (F_x3/obj.F_x01_max)^2); 
            F_y2 = F_y02*sqrt(1 - (F_x3/obj.F_x02_max)^2);
            F_y3 = F_y03*sqrt(1.05 - (F_x3/obj.F_x03_max)^2);
            F_y4 = F_y04*sqrt(1.05 - (F_x4/obj.F_x04_max)^2);
    

            MATRIX = [1 0; 0 1; -obj.l_y1 obj.l_x1]*[cos(delta_1) -sin(delta_1); sin(delta_1) cos(delta_1)]*[F_x1; F_y1] + ...
                     [1 0; 0 1; -obj.l_y2 obj.l_x2]*[cos(delta_2) -sin(delta_2); sin(delta_2) cos(delta_2)]*[F_x2; F_y2] + ...
                     [1 0; 0 1; -obj.l_y3 obj.l_x3]*[cos(delta_3) -sin(delta_3); sin(delta_3) cos(delta_3)]*[F_x3; F_y3] + ...
                     [1 0; 0 1; -obj.l_y4 obj.l_x4]*[cos(delta_4) -sin(delta_4); sin(delta_4) cos(delta_4)]*[F_x4; F_y4];

            Fx = MATRIX(1);
            Fy = MATRIX(2);
            Mz = MATRIX(3);

            Xp_dot = vx*cos(phi) - vy*sin(phi);
            Yp_dot = vx*sin(phi) + vy*cos(phi);
            phi_dot = r;
            r_dot   = Mz/obj.Izz;
            vx_dot  = Fx/obj.m + vy*r;
            vy_dot  = Fy/obj.m - vx*r;
            
            x_dot = [Xp_dot; Yp_dot; phi_dot; r_dot; vx_dot; vy_dot];
        end

        function [tf, U, X] = carsimulate(obj, K3, K4, K5, tf)

            X  = ones(obj.D_state - 1, obj.N + 1);
            U  = ones(obj.D_control, obj.N);

            X0 = [obj.Xp0; obj.Yp0; obj.phi0; obj.r0; obj.vx0; obj.vy0]; 
            dt = tf/obj.N; 
            X(:, 1) = X0;
            for k = 1:obj.N
                p_car = [X(1, k) X(2, k)];
                [si, d] = obj.plan.project(p_car, obj.s0, 2, 20);
                obj.s0 = si;

                Fx3 = - K3*(X(4, k) - obj.vnorm);
                Fx4 = - K4*(X(4, k) - obj.vnorm);
                del = obj.ref_steering(k) - K5*d;

                U_k = [0 0 Fx3 Fx4 del]';

                k1 = obj.vehicle_model(X(:, k),          U_k);
                k2 = obj.vehicle_model(X(:,k) + dt/2*k1, U_k);
                k3 = obj.vehicle_model(X(:,k) + dt/2*k2, U_k);
                k4 = obj.vehicle_model(X(:,k) + dt*k3,   U_k);
                x_next   = X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
                X(:,k+1) = x_next; 
                U(:, k) = U_k;
            end
        end

    end
end