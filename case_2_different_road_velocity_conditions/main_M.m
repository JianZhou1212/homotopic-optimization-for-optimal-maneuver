clc
clear
close all

load('parameters_M.mat');

Total_loop = 1;
Ri_Homo_M = cell(1, Total_loop);

U_Homo_sol_M    = cell(1, Total_loop);
X_Homo_sol_M    = cell(1, Total_loop);
tf_Homo_sol_M   = cell(1, Total_loop);
Time_Homo_sol_M = cell(1, Total_loop);

N_iteration = 25;

for loop = 1:(N_iteration + 1)
    loop

     gamma_i = (loop - 1)/N_iteration;

    parameters_M.R_yi = (1 - gamma_i)*parameters_M.R_y0 + gamma_i*parameters_M.R_ytrue;
    
    Ri_Homo_M{loop} = parameters_M.R_yi;

    if loop ~= 1
        parameters_M.U_guess  = U_Homo_sol_M{loop - 1};
        parameters_M.X_guess  = X_Homo_sol_M{loop - 1};
        parameters_M.tf_guess = tf_Homo_sol_M{loop-1};
        OptimalControllerMinTime = MinTimeCoGDoubleTrack(parameters_M);
    else
        OptimalControllerMinTime = MinTimeCoGDoubleTrackInitialization(parameters_M);
    end
    

    % RUN Homotopic approach
    tic
    [U_opt, X_opt, tf_opt] = OptimalControllerMinTime.solve( );
    toc
    time = toc;
    disp('Above the information for Homotopic')

    U_Homo_sol_M{loop}    = U_opt;
    X_Homo_sol_M{loop}    = X_opt;
    tf_Homo_sol_M{loop}   = tf_opt;
    Time_Homo_sol_M{loop} = time;

end

save('U_Homo_sol_M.mat', 'U_Homo_sol_M');
save('X_Homo_sol_M.mat', 'X_Homo_sol_M');
save('tf_Homo_sol_M.mat', 'tf_Homo_sol_M');
save('Time_Homo_sol_M.mat', 'Time_Homo_sol_M');
save('Ri_Homo_M.mat', ['Ri_Homo_M']);