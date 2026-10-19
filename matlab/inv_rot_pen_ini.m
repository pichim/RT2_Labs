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
% GPA is not active yet. Once finalized at 1 kHz, select frequencies below
% 500 Hz; the unchanged app's default upper frequency of 1000 Hz is too high.


%% Model and controller parameters

% Sampling time
Ts = 1e-3; % sec, matching realtime_thread in src/main.cpp
Ts_fast = 50e-6; % sec, existing 20 kHz current controller

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
