clc
clear
close all

load('parameters.mat');

Total_loop = 1;
Ri_Homo_8 = cell(1, Total_loop);

U_Homo_sol_8    = cell(1, Total_loop);
X_Homo_sol_8    = cell(1, Total_loop);
tf_Homo_sol_8   = cell(1, Total_loop);
Time_Homo_sol_8 = cell(1, Total_loop);

N_iteration = 6;

for loop = 1:(N_iteration + 1)
    loop

    gamma_i = (loop - 1)/N_iteration;

    parameters.R_yi = (1 - gamma_i)*parameters.R_y0 + gamma_i*parameters.R_ytrue;

    Ri_Homo_8{loop} = parameters.R_yi;

    if loop ~= 1
        parameters.U_guess  = U_Homo_sol_8{loop - 1};
        parameters.X_guess  = X_Homo_sol_8{loop - 1};
        parameters.tf_guess = tf_Homo_sol_8{loop-1};
        OptimalControllerMinTime = MinTimeCoGDoubleTrack(parameters);
    else
        OptimalControllerMinTime = MinTimeCoGDoubleTrackInitialization(parameters);
    end
    
    % RUN Homotopic approach
    tic
    [U_opt, X_opt, tf_opt] = OptimalControllerMinTime.solve( );
    toc
    time = toc;
    disp('Above the information for Homotopic')

    U_Homo_sol_8{loop}    = U_opt;
    X_Homo_sol_8{loop}    = X_opt;
    tf_Homo_sol_8{loop}   = tf_opt;
    Time_Homo_sol_8{loop} = time;

end

save('U_Homo_sol_8.mat', 'U_Homo_sol_8');
save('X_Homo_sol_8.mat', 'X_Homo_sol_8');
save('tf_Homo_sol_8.mat', 'tf_Homo_sol_8');
save('Time_Homo_sol_8.mat', 'Time_Homo_sol_8');
save('Ri_Homo_8.mat', 'Ri_Homo_8');


