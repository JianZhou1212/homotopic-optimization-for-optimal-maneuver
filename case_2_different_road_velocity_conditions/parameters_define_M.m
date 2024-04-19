clc
clear
close all

parameters_M = struct;

parameters_M.lf = 1.3;
parameters_M.lr = 1.5;
parameters_M.l = parameters_M.lf + parameters_M.lr;
parameters_M.w = 0.8;
parameters_M.l_x1 = parameters_M.lf;
parameters_M.l_x2 = parameters_M.lf;
parameters_M.l_x3 = -parameters_M.lr;
parameters_M.l_x4 = -parameters_M.lr;
parameters_M.l_y1 = parameters_M.w/2;
parameters_M.l_y3 = parameters_M.w/2;
parameters_M.l_y2 = -parameters_M.w/2;
parameters_M.l_y4 = -parameters_M.w/2;
parameters_M.m = 2100;
parameters_M.Izz = 3900;
parameters_M.g = 9.82;
parameters_M.mu_y1 = 0.935;
parameters_M.mu_y2 = 0.935;
parameters_M.mu_y3 = 0.961;
parameters_M.mu_y4 = 0.961;
parameters_M.B_y1 = 8.86;
parameters_M.B_y2 = 8.86;
parameters_M.B_y3 = 9.3;
parameters_M.B_y4 = 9.3;
parameters_M.C_y1 = 1.19;
parameters_M.C_y2 = 1.19;
parameters_M.C_y3 = 1.19;
parameters_M.C_y4 = 1.19;
parameters_M.E_y1 = -1.21;
parameters_M.E_y2 = -1.21;
parameters_M.E_y3 = -1.11;
parameters_M.E_y4 = -1.11;
parameters_M.F_z1 = parameters_M.m*parameters_M.g*(parameters_M.lr)/parameters_M.l/2;
parameters_M.F_z2 = parameters_M.m*parameters_M.g*(parameters_M.lr)/parameters_M.l/2;
parameters_M.F_z3 = parameters_M.m*parameters_M.g*(parameters_M.lf)/parameters_M.l/2;
parameters_M.F_z4 = parameters_M.m*parameters_M.g*(parameters_M.lf)/parameters_M.l/2;
parameters_M.mu_x1 = 1.2;
parameters_M.mu_x2 = 1.2;
parameters_M.mu_x3 = 1.2;
parameters_M.mu_x4 = 1.2;
parameters_M.F_x01_max = parameters_M.mu_x1*parameters_M.F_z1;
parameters_M.F_x02_max = parameters_M.mu_x2*parameters_M.F_z2;
parameters_M.F_x03_max = parameters_M.mu_x3*parameters_M.F_z3;
parameters_M.F_x04_max = parameters_M.mu_x4*parameters_M.F_z4;
parameters_M.F_x01_min = -parameters_M.mu_x1*parameters_M.F_z1;
parameters_M.F_x02_min = -parameters_M.mu_x2*parameters_M.F_z2;
parameters_M.F_x03_min = -parameters_M.mu_x3*parameters_M.F_z3;
parameters_M.F_x04_min = -parameters_M.mu_x4*parameters_M.F_z4;
parameters_M.p = 4;
parameters_M.delta_min = -pi/4;
parameters_M.delta_dot_min = -pi/3;
parameters_M.delta_max = -parameters_M.delta_min;
parameters_M.delta_dot_max = -parameters_M.delta_dot_min;
parameters_M.Xa = 0;
parameters_M.Ya = 0;
parameters_M.R_xi = 2.5;
parameters_M.R_xo = 7.5;
parameters_M.Xp0  = -5;
parameters_M.Yp0  = 0;
parameters_M.phi0 = pi/2;
parameters_M.vx0 = 40/3.6;
parameters_M.vy0 = 0;
parameters_M.r0  = 0;
parameters_M.wf0 = 0;
parameters_M.wr0 = 0;
parameters_M.del0 = 0;
parameters_M.X_tf = 5;
parameters_M.Y_tf = 0;
parameters_M.D_control = 5;
parameters_M.D_state = 7;
parameters_M.N = 60;
parameters_M.R_yo = 135;
parameters_M.R_ytrue = parameters_M.R_yo - 5;
parameters_M.R_y0 = 5;
parameters_M.tf_guess_ini = 1;

save 'parameters_M.mat' parameters_M;