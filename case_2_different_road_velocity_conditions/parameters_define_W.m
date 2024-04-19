clc
clear
close all

parameters_W = struct;

parameters_W.lf = 1.3;
parameters_W.lr = 1.5;
parameters_W.l = parameters_W.lf + parameters_W.lr;
parameters_W.w = 0.8;
parameters_W.l_x1 = parameters_W.lf;
parameters_W.l_x2 = parameters_W.lf;
parameters_W.l_x3 = -parameters_W.lr;
parameters_W.l_x4 = -parameters_W.lr;
parameters_W.l_y1 = parameters_W.w/2;
parameters_W.l_y3 = parameters_W.w/2;
parameters_W.l_y2 = -parameters_W.w/2;
parameters_W.l_y4 = -parameters_W.w/2;
parameters_W.m = 2100;
parameters_W.Izz = 3900;
parameters_W.g = 9.82;
parameters_W.mu_y1 = 0.885;
parameters_W.mu_y2 = 0.885;
parameters_W.mu_y3 = 0.911;
parameters_W.mu_y4 = 0.911;
parameters_W.B_y1 = 10.7;
parameters_W.B_y2 = 10.7;
parameters_W.B_y3 = 11.3;
parameters_W.B_y4 = 11.3;
parameters_W.C_y1 = 1.07;
parameters_W.C_y2 = 1.07;
parameters_W.C_y3 = 1.07;
parameters_W.C_y4 = 1.07;
parameters_W.E_y1 = -2.14;
parameters_W.E_y2 = -2.14;
parameters_W.E_y3 = -1.97;
parameters_W.E_y4 = -1.97;
parameters_W.F_z1 = parameters_W.m*parameters_W.g*(parameters_W.lr)/parameters_W.l/2;
parameters_W.F_z2 = parameters_W.m*parameters_W.g*(parameters_W.lr)/parameters_W.l/2;
parameters_W.F_z3 = parameters_W.m*parameters_W.g*(parameters_W.lf)/parameters_W.l/2;
parameters_W.F_z4 = parameters_W.m*parameters_W.g*(parameters_W.lf)/parameters_W.l/2;
parameters_W.mu_x1 = 1.06;
parameters_W.mu_x2 = 1.06;
parameters_W.mu_x3 = 1.07;
parameters_W.mu_x4 = 1.07;
parameters_W.F_x01_max = parameters_W.mu_x1*parameters_W.F_z1;
parameters_W.F_x02_max = parameters_W.mu_x2*parameters_W.F_z2;
parameters_W.F_x03_max = parameters_W.mu_x3*parameters_W.F_z3;
parameters_W.F_x04_max = parameters_W.mu_x4*parameters_W.F_z4;
parameters_W.F_x01_min = -parameters_W.mu_x1*parameters_W.F_z1;
parameters_W.F_x02_min = -parameters_W.mu_x2*parameters_W.F_z2;
parameters_W.F_x03_min = -parameters_W.mu_x3*parameters_W.F_z3;
parameters_W.F_x04_min = -parameters_W.mu_x4*parameters_W.F_z4;
parameters_W.p = 4;
parameters_W.delta_min = -pi/4;
parameters_W.delta_dot_min = -pi/3;
parameters_W.delta_max = -parameters_W.delta_min;
parameters_W.delta_dot_max = -parameters_W.delta_dot_min;
parameters_W.Xa = 0;
parameters_W.Ya = 0;
parameters_W.R_xi = 2.5;
parameters_W.R_xo = 7.5;
parameters_W.Xp0  = -5;
parameters_W.Yp0  = 0;
parameters_W.phi0 = pi/2;
parameters_W.vx0 = 20/3.6;
parameters_W.vy0 = 0;
parameters_W.r0  = 0;
parameters_W.wf0 = 0;
parameters_W.wr0 = 0;
parameters_W.del0 = 0;
parameters_W.X_tf = 5;
parameters_W.Y_tf = 0;
parameters_W.D_control = 5;
parameters_W.D_state = 7;
parameters_W.N = 60;
parameters_W.R_yo = 135;
parameters_W.R_ytrue = parameters_W.R_yo - 5;
parameters_W.R_y0 = 5;
parameters_W.tf_guess_ini = 3;

save 'parameters_W.mat' parameters_W;
