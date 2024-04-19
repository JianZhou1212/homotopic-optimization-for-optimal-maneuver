clc
clear
close all
load("parameters.mat");
R_true = parameters.R_yi;
R_x = (parameters.R_xi + parameters.R_xo)/2;
p = parameters.p;
N = parameters.N;
L = parameters.l;

[x_values, y_values] = generate_circle_points(R_x, R_true, p, N);

track  = [x_values; y_values]';
path   = SplinePath(track);
s_grid = linspace(0, path.length, N);
track_out     = path.p(s_grid);

ref_x = track_out(1, :);
ref_y = track_out(2, :);

ref_heading = ones(1, N);
for i = 1:1:N - 1
    ref_heading(i) = atan((ref_y(i + 1) - ref_y(i))/(ref_x(i + 1) - ref_x(i)));
end
ref_heading(end) = -ref_heading(1);
ref_curvature = path.c(s_grid);
ref_steering = L*ref_curvature;

parameters.ref_x = ref_x;
parameters.ref_y = ref_y;
parameters.ref_heading = ref_heading;
parameters.ref_curvature = ref_curvature;
parameters.ref_steering = ref_steering;
parameters.s0 = 0;
parameters.vnorm = 1*parameters.vx0;

K_force = 0.15;
K3 = K_force;
K4 = K_force;
K5 = 0.125;
tf = 14;

Driver = DriverModel(parameters, path);

[tf_guess, U_guess, X_guess] = Driver.carsimulate(K3, K4, K5, tf);

plot(X_guess(1, :), X_guess(2, :), 'k');
hold on
plot(ref_x, ref_y, 'b');

dt = tf_guess/N;

X_guess(7, :) = [0 U_guess(end, :)];
U_guess(5, :) = gradient(U_guess(5, :))/dt;

parameters.U_guess  = U_guess;
parameters.X_guess  = X_guess;
parameters.tf_guess = tf_guess;
OptimalControllerMinTime = MinTimeCoGDoubleTrack(parameters);

% RUN
tic
[U_opt, X_opt, tf_opt] = OptimalControllerMinTime.solve( );
toc
time = toc;

hold on
plot(X_opt(1, :), X_opt(2, :))
save('U_guess.mat', 'U_guess');
save('X_guess.mat', 'X_guess');
save('tf_guess.mat', 'tf_guess');
save('U_opt.mat', 'U_opt');
save('X_opt.mat', 'X_opt');
save('tf_opt.mat', 'tf_opt');
save('Time_Com.mat', 'time');



