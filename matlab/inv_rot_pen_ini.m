clc, clear variables
matlab_dir = fileparts(mfilename('fullpath'));
addpath(matlab_dir, fullfile(matlab_dir, 'app'));
%% Open GUI

% run GPA_nucleo_UART_exported.m
% Firmware controls the pendulum locally; Simulink UART exchange is not used.
% Reset the board with the pendulum hanging downward and the arm at its zero.
% GUI setpoint: arm position [rad] relative to this startup zero.
% Blue B1 button: INIT -> ARMED; upright capture -> BALANCE.
% User LED blinks while ARMED and is solid while BALANCE.
% GPA excitation/update is prepared but commented out in realtime_thread.
% Press B1 again to cancel/disarm. Leaving the outer angle band returns to INIT.
% Button rearming does not reset the encoders; hardware reset starts a new zero.
% App I/O fields Out / In1 / In2 show current [A] / theta1 [rad] / theta2 [rad].
% Six logged columns: i_des [A], current [A], theta1 [rad], theta2 upright error
% [rad], omega1 [rad/s], omega2 [rad/s]. No averaging is applied.
% GPA is not active yet. Once finalized, select frequencies below 5000 Hz.
% GUI defaults: GPA upper frequency 4990 Hz, logging downsampling 20.
% At 10 kHz, 1000 logged rows cover approximately 2 sec at downsampling 20.


%% Model and controller parameters

% Sampling time
Ts = 1 / 10e3; % sec, matching realtime_thread in src/main.cpp
f_pwm = 20e3; % Hz, hardware PWM is independent of the control sampling time

% Max. current
i_max = 0.5; % A, saturation BEFORE friction compensation as in Simulink
i_friction = 0.0205; % A; maximum resulting command is +/-0.5205 A

% Enable- and Disable Angle in rad
phi2_enable = 0.1745329252; % rad, 10 deg: capture band after button arming
phi2_disable = 0.5235987756; % rad, 30 deg: disable and require a new button press

% Differentiating Filter
s = zpk('s');
Tf = 1 / (2*pi*100);
G_diff = c2d(s / (Tf*s + 1), Ts, 'tustin');

% Physical parameters
param = get_parameter();

% Encoder notch filter (Tustin with prewarping), matching IIR_Filter.
f_notch = 680; % Hz
d_notch = 0.6;
w_notch = 2*pi*f_notch;
G_notch = c2d((s^2 + w_notch^2) / ...
    (s^2 + 2*d_notch*w_notch*s + w_notch^2), Ts, ...
    c2dOptions('Method', 'tustin', 'PrewarpFrequency', w_notch));

% Current controller, matching realtime_thread and PID_Cntrl.
supply_voltage = 24; % V
offset_voltage = 2; % V, motor deadzone compensation
kp_i = 2.5;
tn_i = 0.0013 / 4.5320; % sec, existing firmware value (L / R)
ki_i = kp_i / tn_i;
enable_current_rolloff = false; % match ENABLE_CURRENT_ROLLOFF in C++
tau_ro_i = 1 / (2*pi*3e3);
C_i_no_rolloff = pid(kp_i, ki_i, 0, 0, Ts, 'IFormula', 'BackwardEuler');
G_rolloff = c2d(1 / (tau_ro_i*s + 1), Ts, 'tustin');
C_i_rolloff = C_i_no_rolloff * G_rolloff;
C_i = C_i_no_rolloff;
if enable_current_rolloff
    C_i = C_i_rolloff;
end

% Linear R/L check only: no back-EMF, PWM ripple, deadzone, limits or computation delay.
G_i = c2d(1 / (param.L*s + param.R), Ts, 'zoh');
G_i_cl_no_rolloff = feedback(G_i * C_i_no_rolloff, 1);
G_i_cl_rolloff = feedback(G_i * C_i_rolloff, 1);

% Pendulum up
theta0 = [0; pi];
[A, B] = linearize_furuta_equilibrium(theta0, param);
B = B * param.km; % Account for Current Input

% State space controller
Q = diag([1 10 0.001 0.001]);
r = 0.5 * 10;
[K, ~, P] = lqr(A, B, Q, r) % Input is current [A]; do not divide by km again

% D = 0.8;
% P_des = [ 6 * [exp(1i*(pi + acos(D))) ; ...
%                exp(1i*(pi - acos(D)))]; ...
%          28 * [exp(1i*(pi + acos(D))) ; ...
%                exp(1i*(pi - acos(D)))]];
% K = acker(A, B, P_des)

% Prefilter
sys_cl = ss(A - B*K, B, eye(4), 0);
dc = dcgain( sys_cl );
V = 1 / dc(1)


%% Values for realtime_thread.cpp

% State vector x = [theta1; shortangle(theta2-pi); omega1; omega2].
% theta2 is continuous for differentiation; wrap only the upright angle error.
% Control law: i = V*w - K*x, with w in rad (no turns-to-rad conversion).
% Copy these values when changing the controller, as in mini_cuboid_ini.
fprintf('Matrix<float, 1, 4> K(%1.9ff, %1.9ff, %1.9ff, %1.9ff);\n', K);
fprintf('const float V = %1.9ff;\n', V);
