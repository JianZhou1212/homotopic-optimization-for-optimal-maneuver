clc
clear
close all

load('parameters.mat');

Total_loop = 4;

U_sol    = cell(1, Total_loop);
X_sol    = cell(1, Total_loop);
tf_sol   = cell(1, Total_loop);
Time_sol = cell(1, Total_loop);
%%
% stepwise configuration
Y_tf = [parameters.R_yi - 10; parameters.R_yi; parameters.R_yi - 2.5; parameters.R_yi - 15; parameters.R_yi - 20;  parameters.Y_tf];
X_tf = [parameters.Xp0;       0;               parameters.X_tf - 1.0; parameters.X_tf + 1.0;parameters.X_tf + 2.25; parameters.X_tf];
%%
for loop = 1:6
    loop

    %parameters.N = N(loop);
    parameters.Y_tf = Y_tf(loop);
    parameters.X_tf = X_tf(loop);

    if loop ~= 1
        parameters.U_guess  = U_sol{loop - 1};
        parameters.X_guess  = X_sol{loop - 1};
        parameters.tf_guess = tf_sol{loop-1};
        OptimalControllerMinTime = MinTimeCoGDoubleTrack(parameters);
    else
        OptimalControllerMinTime = MinTimeCoGDoubleTrackInitialization(parameters);
    end

    % RUN
    tic
    [U_opt, X_opt, tf_opt] = OptimalControllerMinTime.solve( );
    toc
    time = toc;

    U_sol{loop}    = U_opt;
    X_sol{loop}    = X_opt;
    tf_sol{loop}   = tf_opt;
    Time_sol{loop} = time;

end
%%
save('U_sol.mat', 'U_sol');
save('X_sol.mat', 'X_sol');
save('tf_sol.mat', 'tf_sol');
save('Time_sol.mat', 'Time_sol');

