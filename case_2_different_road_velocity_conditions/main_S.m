clc
clear
close all

load('parameters_S.mat');


Total_loop = 1;
Ri_Homo_S = cell(1, Total_loop);

U_Homo_sol_S    = cell(1, Total_loop);
X_Homo_sol_S    = cell(1, Total_loop);
tf_Homo_sol_S   = cell(1, Total_loop);
Time_Homo_sol_S = cell(1, Total_loop);

N_iteration = 25;

for loop = 1:(N_iteration + 1)
    loop

     gamma_i = (loop - 1)/N_iteration;

    parameters_S.R_yi = (1 - gamma_i)*parameters_S.R_y0 + gamma_i*parameters_S.R_ytrue;
    
    Ri_Homo_S{loop} = parameters_S.R_yi;

    if loop ~= 1
        parameters_S.U_guess  = U_Homo_sol_S{loop - 1};
        parameters_S.X_guess  = X_Homo_sol_S{loop - 1};
        parameters_S.tf_guess = tf_Homo_sol_S{loop-1};
        OptimalControllerMinTime = MinTimeCoGDoubleTrack(parameters_S);
    else
        OptimalControllerMinTime = MinTimeCoGDoubleTrackInitialization(parameters_S);
    end
    
    % RUN Homotopic approach
    tic
    [U_opt, X_opt, tf_opt] = OptimalControllerMinTime.solve( );
    toc
    time = toc;
    disp('Above the information for Homotopic')

    U_Homo_sol_S{loop}    = U_opt;
    X_Homo_sol_S{loop}    = X_opt;
    tf_Homo_sol_S{loop}   = tf_opt;
    Time_Homo_sol_S{loop} = time;

end

save('U_Homo_sol_S.mat', 'U_Homo_sol_S');
save('X_Homo_sol_S.mat', 'X_Homo_sol_S');
save('tf_Homo_sol_S.mat', 'tf_Homo_sol_S');
save('Time_Homo_sol_S.mat', 'Time_Homo_sol_S');
save('Ri_Homo_S.mat', 'Ri_Homo_S');


