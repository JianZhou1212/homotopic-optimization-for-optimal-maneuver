clc
clear
close all

load('parameters_D.mat');

Total_loop = 1;
Ri_Homo_D = cell(1, Total_loop);

U_Homo_sol_D    = cell(1, Total_loop);
X_Homo_sol_D    = cell(1, Total_loop);
tf_Homo_sol_D   = cell(1, Total_loop);
Time_Homo_sol_D = cell(1, Total_loop);

N_iteration = 25;

for loop = 1:(N_iteration + 1)
    loop

    gamma_i = (loop - 1)/N_iteration;

    parameters_D.R_yi = (1 - gamma_i)*parameters_D.R_y0 + gamma_i*parameters_D.R_ytrue;
    
    Ri_Homo_D{loop} = parameters_D.R_yi;

    if loop ~= 1
        parameters_D.U_guess  = U_Homo_sol_D{loop - 1};
        parameters_D.X_guess  = X_Homo_sol_D{loop - 1};
        parameters_D.tf_guess = tf_Homo_sol_D{loop-1};
        OptimalControllerMinTime = MinTimeCoGDoubleTrack(parameters_D);
    else
        OptimalControllerMinTime = MinTimeCoGDoubleTrackInitialization(parameters_D);
    end
    

    % RUN Homotopic approach
    tic
    [U_opt, X_opt, tf_opt] = OptimalControllerMinTime.solve( );
    toc
    time = toc;
    disp('Above the information for Homotopic')

    U_Homo_sol_D{loop}    = U_opt;
    X_Homo_sol_D{loop}    = X_opt;
    tf_Homo_sol_D{loop}   = tf_opt;
    Time_Homo_sol_D{loop} = time;

end

save('U_Homo_sol_D.mat', 'U_Homo_sol_D');
save('X_Homo_sol_D.mat', 'X_Homo_sol_D');
save('tf_Homo_sol_D.mat', 'tf_Homo_sol_D');
save('Time_Homo_sol_D.mat', 'Time_Homo_sol_D');
save('Ri_Homo_D.mat', 'Ri_Homo_D');

