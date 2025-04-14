clc; clearvars;
%% Purpose of this script
% The purpose of this script is to calculate the parameters necessary to
% fit the flight data curve to the curve of the model

%% Import the data
i = 1;

addpath('support_files')
load dataset_revision.mat
exp_title='PITCH_STEPS';
NEXP=[102 101 103 94 104];
legend_list={'pitch 15\circ','pitch 30\circ','pitch 45\circ','pitch 60\circ','pitch 75\circ'};

colors=viridis(length(NEXP));
colors_light=colors;

nSigma=1;

Nexp=NEXP(i);
    
eval(['data=experiment' num2str(Nexp) ';'])
assign_repetition_variables

%% Interpolate existing dihedral
valid = ~isnan(DIHEDavg);
x_val = TIME(valid);
y_val = DIHEDavg(valid);
y_valsmooth = smoothdata(y_val, 1, "sgolay", "omitmissing");
f_dihed = griddedInterpolant(x_val, y_valsmooth, "spline", "linear");

DIHEDinterp = f_dihed(TIME);

%% Filter the dihedral
fs = 360; %Hz
fc = 10; % Hz

order = 3;

[b, a] = butter(order, fc/(fs/2), 'low'); 

DIHEDinterp = filtfilt(b, a, DIHEDinterp);

%% Differential equation parameters

m = 0.0296;

g = 9.80665;

lw = 0.081;

%% Prepare inputs for the model

% Compute first derivative of ld

dt = mean(diff(TIME));  % Assuming uniform sampling

ld = lw*sin(DIHEDinterp*pi/180);
f_ldd = gradient(ld) / dt;
f_ldd = griddedInterpolant(TIME, f_ldd, 'spline', 'linear');

%% Set up the nonlinear least squares solving

% Parametrize the pitch angle theta
theta_smooth = smoothdata(PITCHavg*pi/180, 1, "sgolay", "omitnan");
f_theta = griddedInterpolant(TIME, theta_smooth, "spline", "linear");

% Take the derivative to find thetad
dt = mean(diff(TIME));
thetad_est = gradient(theta_smooth) / dt;
f_thetad = griddedInterpolant(TIME, thetad_est, "spline", "linear");


ud_odefunc = @(t, u, bx, lz) -sin(f_theta(t))*g -  bx/m*(u - lz * f_thetad(t) + f_ldd(t));


%% Define residuals function

function residuals = simulate_ode_and_get_residuals(bx, lz, t_data, x0, x_target, f_ode)

    % Example: you could include offset in the ODE or adjust target
    ode_func = @(t, x) f_ode(t, x, bx, lz);
    [~, x_sol] = ode45(ode_func, t_data, x0);
    x_sim = x_sol(:);
    
    % Maybe apply offset to x_target
    residuals = x_sim - (x_target(:));
end

%% Least squares analysis

% Initial guess for parameters
p0 = [0.05, 0.0271];  % initial guesses for [bx, lz]

x0 = -0.178;

x_target = VELxavg;

residual_fun = @(p) simulate_ode_and_get_residuals(p(1), p(2), TIME, x0, x_target, ud_odefunc);


opts = optimoptions('lsqnonlin', ...
    'Display','iter', ...
    'StepTolerance', 1e-12, ...
    'FunctionTolerance', 1e-8, ...
    'OptimalityTolerance', 1e-8);

%p_est = lsqnonlin(residual_fun, p0, [], [], opts);

%p_est;



%% Solve ODE with parameters by Matej
bx = 0.07220;

lz = 0.02710;

u0 = 0.178;

ud_odefunc = @(t, u) -sin(f_theta(t))*g -  2*bx/m*(u - lz * f_thetad(t) + f_ldd(t));

[t_ode, u_sol] = ode45(ud_odefunc, TIME, u0);

ud = zeros(size(u_sol));
for i = 1:length(t_ode)
    ud(i,:) = ud_odefunc(t_ode(i), u_sol(i,:)')';  % Make sure dimensions match
end



figure;
subplot(3,1,1);
plot(TIME, DIHEDinterp, 'LineWidth', 2);
title('Interpolated and Filtered DIHED(t)');
xlabel('Time (s)');
ylabel('DIHED (rad)');

subplot(3,1,2); hold on;
plot(t_ode, u_sol, 'LineWidth', 2);
plot(TIME, VELxavg, DisplayName='Flight Data')
title('Solution of the ODE');
xlabel('Time (s)');
ylabel('u(t)');

subplot(3,1,3); hold on;
plot(t_ode, ud, 'b')
plot(TIME, mean(ACCx))
legend('Model', 'Flight Data')
xlabel('Time')
ylabel('Value')
