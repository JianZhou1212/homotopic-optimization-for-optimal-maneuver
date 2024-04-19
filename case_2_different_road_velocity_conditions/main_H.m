clc
clear
close all

load('parameters_H.mat');

Total_loop = 1;
Ri_Homo_H = cell(1, Total_loop);

U_Homo_sol_H    = cell(1, Total_loop);
X_Homo_sol_H    = cell(1, Total_loop);
tf_Homo_sol_H   = cell(1, Total_loop);
Time_Homo_sol_H = cell(1, Total_loop);

N_iteration = 25;

for loop = 1:(N_iteration + 1)
    loop

    gamma_i = (loop - 1)/N_iteration;

    parameters_H.R_yi = (1 - gamma_i)*parameters_H.R_y0 + gamma_i*parameters_H.R_ytrue;
    
    Ri_Homo_H{loop} = parameters_H.R_yi;
    
    if loop ~= 1
        parameters_H.U_guess  = U_Homo_sol_H{loop - 1};
        parameters_H.X_guess  = X_Homo_sol_H{loop - 1};
        parameters_H.tf_guess = tf_Homo_sol_H{loop-1};
        OptimalControllerMinTime = MinTimeCoGDoubleTrack(parameters_H);
    else
        OptimalControllerMinTime = MinTimeCoGDoubleTrackInitialization(parameters_H);
    end
    

    % RUN Homotopic approach
    tic
    [U_opt, X_opt, tf_opt] = OptimalControllerMinTime.solve( );
    toc
    time = toc;
    disp('Above the information for Homotopic')

    U_Homo_sol_H{loop}    = U_opt;
    X_Homo_sol_H{loop}    = X_opt;
    tf_Homo_sol_H{loop}   = tf_opt;
    Time_Homo_sol_H{loop} = time;

end

save('U_Homo_sol_H.mat', 'U_Homo_sol_H');
save('X_Homo_sol_H.mat', 'X_Homo_sol_H');
save('tf_Homo_sol_H.mat', 'tf_Homo_sol_H');
save('Time_Homo_sol_H.mat', 'Time_Homo_sol_H');
save('Ri_Homo_H.mat', 'Ri_Homo_H');
