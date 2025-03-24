# Readme
## Description
This dataset contains the flight data used for the experimental validation of the 3D relative localization algorithm presented in the paper "Three-dimensional Relative Localization and Swarming with Wireless Ranging".

The data was recorded in the Cyberzoo test arena at the Faculty of Aerospace Engineering at TU Delft. 

## Experimental Data
### Folder structure
The compressed ```data.zip``` folder contains 4 subfolders for the individual experiments:
- ```velocity_model```: Data used for tuning & testing the horizontal velocity model for the Flapper Drones. 
- ```velocity```: On-board testing of the horizontal velocity estimates.
- ```relative_localization```: Relative localization with two drone in independent, manual flight.
- ```leader_follower```: Leader-Follower experiment with the leader being flown manually.

### File names
File names begin with the type of drone used for the experiment, ```cf``` for Crazyflies or ```fd``` for Flapper Drones. The drone type is followed by the experiment name and a number for distinguishing different runs of the same experiments.

### Data file content
The data is provided as .csv files with subsets of the following column headers (the exact data recorded depends on the experiments): 
#### General
- ```timeTick```: Timestamp (on-board) in [ms].
- ```otX```, ```otY```, ```otZ```: Position recorded from the Optitrack system in [m]. The number (0 or 1) distinguishes the drone in experiments with two drones.
- ```otRoll```, ```otPitch```, ```otYaw```: Attitude recorded from the Optitrack system in [deg].
#### Velocity model identification only
- ```roll```, ```pitch```, ```yaw```: On-board atttitude estimate in [deg]
- ```accX```, ```accY```, ```accZ```: On-board accelerometer measurements in [G] (1G=9.81m/s)
- ```baro```: On-board altitude measurement from barometer in [m]
- ```m1```,  ```m2```,  ```m3```,  ```m4```: Motor commands (m1: left wing flapping, m2: pitch servo, m3: yaw servo, m4: right wing flapping)
- ```thrust```: Thrust command
- ```vbat```: Battery voltage in [V]
#### Velocity estimation
- ```stateX```, ```stateY```, ```stateZ```: On-board position estimate (Odometry) in [m]
- ```stateVX```, ```stateVY```, ```stateVZ```: On-board horizontal velocity estimate in [m/s]
#### Relative localization & Leader-Follower
- ```rlX1```, ```rlY1```, ```rlZ1```, ```rlYaw1```: Relative position of drone 1 estimated on-board drone 0 in [m].
- ```PX1```, ```PY1```, ```PZ1```, ```PYaw1```: Diagonal of the relative position ekf covariance matrix P for the relative position of drone 1 estimated on-board drone 0 in [m2].
- ```dist1```: Distance to drone 1 measured by drone 0 using UWB in [m].
- ```vx```, ```vy```, ```vz```, ```r```: Inputs for the relative position ekf on drone 0 - horizontal velocity (in [m/s]) and yaw rate (in [deg/s]).

## Analysis
Python scripts are included that were used to evaluate the data and create the plots in the paper. To use the scripts with the provided data, extract the data folder to the same location as the evaluation scripts, keeping the data folders internal structure.

#### Analysis Scripts
- ```flapper_velocity_model_identification.py```: Train and test the velocity model on the provided data. Outputs the training and testing mean RMSE for vx, vy and vz as well as plots for one training and one testing file. 
- ```velocity_model_plots.py```: Compare horizontal velocity estimates for Crazyflie, Flapper Drone Level flight and Flapper Drone with large vz. Plots one example trajectory from each experiment and a Boxplot for performance comparison.
- ```relative_localization_plots.py```: Compare relative localization for Crazyflie and Flapper Drones in free flight and in Leader-Follower scenario. Plots one example trajectory for each experiment and a Boxplot for performance comparison.

#### Included Files
- ```analysis_utils.py```: Contains the ```LogReader``` class used to read and preprocess the log data as well as common functions for data processing and evaluation.
- ```plot_settings.py```: Code and functions for uniform formatting of the plots.


## Code references

Data was logged using the Crazyflie-Suite interface: https://github.com/tudelft/crazyflie-suite

The on-board code for performing the experiments can be found here:

(cf) https://github.com/tudelft/crazyflie-firmware/tree/cf_swarm3d

(fd) https://github.com/tudelft/crazyflie-firmware/tree/flapper_swarm3d
