+--------------------------------------------------------------------+
| Index of included datasets:
+--------------------------------------------------------------------+

1) BAL15_set:
Balanced DelFly Nimble responses to 15-degree magnitude quasi-step pitch maneuvers.
2) BAL30_set:
Balanced DelFly Nimble responses to 30-degree magnitude quasi-step pitch maneuvers.
3) BAL45_set:
Balanced DelFly Nimble responses to 45-degree magnitude quasi-step pitch maneuvers.
4) BAL60_set:
Balanced DelFly Nimble responses to 60-degree magnitude quasi-step pitch maneuvers.
5) BH30_set:
Bottom-heavy DelFly Nimble responses to 30-degree magnitude quasi-step pitch maneuvers.
6) FH30_set:
Front-heavy DelFly Nimble responses to 30-degree magnitude quasi-step pitch maneuvers.
7) CVR2_control_set:
First control set of balanced DelFly Nimble responses to 40-degree magnitude quasi-step pitch maneuvers. This should be the control for CVR2_Kd600_set, CVR2_Kd800_set, CVR2_Kp3900_set, CVR2_Kp11700_set.
8) CVR2_Kd600_set:
Balanced DelFly Nimble responses to 40-degree magnitude quasi-step pitch maneuvers with a rate feedback gain of 600.
9) CVR2_Kd800_set:
Balanced DelFly Nimble responses to 40-degree magnitude quasi-step pitch maneuvers with a rate feedback gain of 800.
10) CVR2_Kp3900_set:
Balanced DelFly Nimble responses to 40-degree magnitude quasi-step pitch maneuvers with an attitude feedback gain of 3900.
11) CVR2_Kp11700_set:
Balanced DelFly Nimble responses to 40-degree magnitude quasi-step pitch maneuvers with an attitude feedback gain of 11700.

+--------------------------------------------------------------------+
| Contents of each dataset:
+--------------------------------------------------------------------+

1) struct 'CL_errors':
Contains the calculated NRMS errors and correlation coefficients per each longitudinal state.
2) cell 'final_err_cell' contains, per each maneuver, auxiliary data used for calculating errors between simulation and OptiTrack-measured states.
3) struct 'final_sims':
Contains, per each maneuver, a sub-struct that contains all of the relevant simulation states;
4) struct 'testpars':
Contains the parameters used in the simulations. Contains some deprecated parameters as well, related to earlier versions of the simulation.
5) cell 'testsids':
Contains, per each maneuver, a struct with Optitrack measurements and estimates, onboard measurements and estimates, as well as a time vector to go with the rest of the data.

+--------------------------------------------------------------------+
| Simulation model:
+--------------------------------------------------------------------+
'run_sims.m' is the main script that can be used to select a dataset in order to run the simulation and print some example plots.

+--------------------------------------------------------------------+
| In case of questions e-mail karlmartin.kajak@gmail.com.
+--------------------------------------------------------------------+