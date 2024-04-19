clc
clear
close all

load('parameters_W.mat');

Total_loop = 1;
Ri_Homo_W = cell(1, Total_loop);

U_Homo_sol_W    = cell(1, Total_loop);
X_Homo_sol_W    = cell(1, Total_loop);
tf_Homo_sol_W   = cell(1, Total_loop);
Time_Homo_sol_W = cell(1, Total_loop);

N_iteration = 25;

for loop = 1:(N_iteration + 1)
    loop

   gamma_i = (loop - 1)/N_iteration;

    parameters_W.R_yi = (1 - gamma_i)*parameters_W.R_y0 + gamma_i*parameters_W.R_ytrue;
    
    Ri_Homo_W{loop} = parameters_W.R_yi;

    if loop ~= 1
        parameters_W.U_guess  = U_Homo_sol_W{loop - 1};
        parameters_W.X_guess  = X_Homo_sol_W{loop - 1};
        parameters_W.tf_guess = tf_Homo_sol_W{loop-1};
        OptimalControllerMinTime = MinTimeCoGDoubleTrack(parameters_W);
    else
        OptimalControllerMinTime = MinTimeCoGDoubleTrackInitialization(parameters_W);
    end
    
    % RUN Homotopic approach
    tic
    [U_opt, X_opt, tf_opt] = OptimalControllerMinTime.solve( );
    toc
    time = toc;
    disp('Above the information for Homotopic')

    U_Homo_sol_W{loop}    = U_opt;
    X_Homo_sol_W{loop}    = X_opt;
    tf_Homo_sol_W{loop}   = tf_opt;
    Time_Homo_sol_W{loop} = time;

end

save('U_Homo_sol_W.mat', 'U_Homo_sol_W');
save('X_Homo_sol_W.mat', 'X_Homo_sol_W');
save('tf_Homo_sol_W.mat', 'tf_Homo_sol_W');
save('Time_Homo_sol_W.mat', 'Time_Homo_sol_W');
save('Ri_Homo_W.mat', 'Ri_Homo_W');