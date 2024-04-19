clc
clear
close all

load('parameters_L.mat');

Total_loop = 1;
Ri_Homo_L = cell(1, Total_loop);

U_Homo_sol_L    = cell(1, Total_loop);
X_Homo_sol_L    = cell(1, Total_loop);
tf_Homo_sol_L   = cell(1, Total_loop);
Time_Homo_sol_L = cell(1, Total_loop);

N_iteration = 25;

for loop = 1:(N_iteration + 1)
    loop

   gamma_i = (loop - 1)/N_iteration;

    parameters_L.R_yi = (1 - gamma_i)*parameters_L.R_y0 + gamma_i*parameters_L.R_ytrue;
    
    Ri_Homo_L{loop} = parameters_L.R_yi;

    if loop ~= 1
        parameters_L.U_guess  = U_Homo_sol_L{loop - 1};
        parameters_L.X_guess  = X_Homo_sol_L{loop - 1};
        parameters_L.tf_guess = tf_Homo_sol_L{loop-1};
        OptimalControllerMinTime = MinTimeCoGDoubleTrack(parameters_L);
    else
        OptimalControllerMinTime = MinTimeCoGDoubleTrackInitialization(parameters_L);
    end
    

    % RUN Homotopic approach
    tic
    [U_opt, X_opt, tf_opt] = OptimalControllerMinTime.solve( );
    toc
    time = toc;
    disp('Above the information for Homotopic')

    U_Homo_sol_L{loop}    = U_opt;
    X_Homo_sol_L{loop}    = X_opt;
    tf_Homo_sol_L{loop}   = tf_opt;
    Time_Homo_sol_L{loop} = time;

end

save('U_Homo_sol_L.mat', 'U_Homo_sol_L');
save('X_Homo_sol_L.mat', 'X_Homo_sol_L');
save('tf_Homo_sol_L.mat', 'tf_Homo_sol_L');
save('Time_Homo_sol_L.mat', 'Time_Homo_sol_L');
save('Ri_Homo_L.mat', 'Ri_Homo_L');