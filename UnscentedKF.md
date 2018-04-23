# ExtendedKF project
## GitHub Repo
[CarND-Extended-Kalman-Filter-Project](https://github.com/QuantumCoherence/CarND-Extended-Kalman-Filter-Project)

This project was completed on Ubuntu 16.04.

The ecbuild subfolder contains all ECLIPSE project files, including compiled binary.

The build subfolder contains all the binary and makefile for compiling the project at the prompt with no formal IDE. 

**Compiling on your system: CMakelists.txt**

Use the CMakeLists.txt in the repo root folder to create the make file and compile on your system.

## RMSE Result
The file rmse.txt in the subfolder build contains the RMSE values for p_x, p_y, vx and vy, when run with the Dataset 1.

Here below a plot of the RMSE values
![RMSE Plot](https://github.com/QuantumCoherence/CarND-Extended-Kalman-Filter-Project/blob/master/RMSE_result.jpg?raw=true)

The file EKF_result.png is a screendump of the simulator at the end of the Dataset 1 run.

![Simulator Screen Dump](https://github.com/QuantumCoherence/CarND-Extended-Kalman-Filter-Project/blob/master/EKF%20result.png?raw=true)

## EKF convergence tuning

**Paramters used for tuning**

	1. Initial State Vector Values
    2. Process Covariance Initial Values 
    3. Acceleration Noise values

*Initial State Vector Values*

Using the first measurement combined with the value of 5 for the Vx component and 0 for Vy, helped keeping the intitial error low.

*Process Covariance Initial Values*

Using the values of 50 for p_x and p_y , and 500 for the Vx and Vy, accelerated the initial convergence but then delayed the stabilziation of the error values. However, 

*Acceleration Noise values*

Increasing the noise value to 15 kept the error below the minimal required threshold, very rapidly.

**Remarks**

The largest errors occur at the begining of the turn, which is when acceleration changes the most. A larger noise level of the random model for the unknown acceleration helped better modeling the error of the position and veclocity estimations.