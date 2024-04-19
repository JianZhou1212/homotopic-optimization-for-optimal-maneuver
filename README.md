# homotopic-optimization-for-optimal-maneuver
This is the MATLAB code for the article
```
@ainproceedings{zhou2024homotopic,
  title={Homotopic Optimization for Autonomous Vehicle Maneuvering},
  author={Jian Zhou, Arvind Balachandran, Bj\"orn Olofsson, Lars Nielsen, and Erik Frisk},
  booktitle={2024 35th IEEE Intelligent Vehicles Symposium (IV)},
  year={2024},
  pages={}} 
```
which has been accepted for publication at the 35th IEEE Intelligent Vehicles Conference (IEEE IV).

The authors are from the Department of Electrical Engineering, Linköping University, Sweden, and the Department of Automatic Control, Lund University, Sweden.

Contact: Jian Zhou (jian.zhou@liu.se).
## Packages for running the code
To run the code you need to install:

**CasADi**: https://web.casadi.org/

**HSL Solver**: https://licences.stfc.ac.uk/product/coin-hsl

Note: Installing the HSL package can be a bit comprehensive, but the solvers just speed up the solutions. You can comment out the places where the HSL solver is used, i.e., options.ipopt.linear_solver = 'ma57', and just use the default linear solver of CasADi.

## Introduction to the files
The article includes three case studies. The implementation details of the code are given below.

In the folder **case_1_change_homotopy_iteration_n**:

(1) The file `MinTimeCoGDoubleTrackInitialization.m` defines the optimization problem for initializing the homotopic optimization.

(2) The file `MinTimeCoGDoubleTrack.m` defines the optimization problem during the homotopy iterations.

(3) The file `parameters_define.m` defines the static parameters.

(4) The files `main_1.m` to `main_11.m` contain the cases for implementing the homotopic optimization with different iterations $n$. For example, in `main_1.m`, $n=120$. The only difference in these files is just different values of $n$.

(5) To implement the method, first run `parameters_define.m`, then you can choose any case from `main_1.m` to `main_11.m` to get the results with different $n$ values.

In the folder **case_2_different_road_velocity_conditions**:

(1) The file `MinTimeCoGDoubleTrackInitialization.m` defines the optimization problem for initializing the homotopic optimization.

(2) The file `MinTimeCoGDoubleTrack.m` defines the optimization problem during the homotopy iterations.

(3) The files `parameters_define_D.m`, `parameters_define_W.m`, and `parameters_define_S.m` define the static parameters on the dry, wet, and snow roads, respectively. The files `parameters_define_H.m`, `parameters_define_M.m`, and `parameters_define_L.m` define the static parameters with high initial speed, middle initial speed, and low initial speed, respectively.

(4) The files `main_D.m`... `main_L.m` implement the homotopic optimization method corresponding to the parameters defined in `parameters_define_D.m`...`parameters_define_L.m`. For example, `main_D.m` implements the problem subject to the dry road with the static parameters defined in `parameters_define_D.m`.

(5) To implement the method, first run the file `parameters_define_X.m` to define the parameters, then run the main file `main_X.m`. 

In the folder **case_3_compare_with_stepwise_initialization**:

(1) It contains three sub-folders, corresponding to the implementation of the stepwise initialization for the three problems listed in Table III of the paper.

(2) For instance, in the sub-folder **problem 1**, the implementation of Problem 1 in Table III of the paper by the stepwise initialization is provided, where the file `MinTimeCoGDoubleTrackInitialization.m` defines the optimization problem for initializing the optimization. The file `MinTimeCoGDoubleTrack.m` defines the optimization problem for solving the maneuvering problem, where the initial guess is obtained stepwisely. The file `parameters_define.m` defines the static parameters, and `main.m` is the main file for running the method. Normally, the stepwise initialization does not get convergence for this problem. The files in another two sub-folders are interpreted in the same way.

(3) To implement the method, first run `parameters_define.m`, then run `main.m`.

In the folder **case_4_compare_with_driver_model_initialization**:
(1) It contains three sub-folders, corresponding to the implementation of the driver-model-based initialization for the three problems listed in Table III of the paper.

(2) For instance, in the sub-folder **problem 1**, the implementation of Problem 1 in Table III of the paper by the driver-model-based initialization is provided. The file `MinTimeCoGDoubleTrack.m` defines the optimization problem for solving the maneuvering problem, where the initial guess is obtained by the driver model. The file `parameters_define.m` defines the static parameters, and `main.m` is the main file for running the method. The files `SplinePath.m`, `generate_circle_points.m`, and `DriverModel.m` together find the initial guess of the problem by the driver model. Normally, the driver-model-based initialization does not get convergence for this problem. The files in another two sub-folders are interpreted in the same way.

(3) To implement the method, first run `parameters_define.m`, then run `main.m`.


