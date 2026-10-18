clc, clear all
addpath app/
%% Open Gui

% run GPA_nucleo_UART_exported.m

% You might want to use the following code later.

% fprintf('Matrix<float, 1, 2> K(%1.4ff, %1.4ff);\n', K(1), K(2));

% fprintf('m_A << %1.3ff, %1.3ff, %1.3ff, %1.3ff;\n', A(1,1), A(1,2), A(2,1), A(2,2));
% fprintf('m_B << %1.3ff, %1.3ff;\n', B(1), B(2));
% fprintf('m_C << %1.3ff, %1.3ff;\n', C(1,1), C(1,2));
% fprintf('m_H << %1.3ff, %1.3ff;\n', H(1), H(2));


%% Intro to C++ and Mbed

% Parameters
R1 = 4.7e3;  % Ohm
R2 = R1;
C1 = 470e-9; % F
C2 = C1;

% Transfer function
s = tf('s');
a = R1*R2*C1*C2;
b = R1*C1 + R1*C2 + R2*C2;
G = 1 / (a*s^2 + b*s + 1);

% --- P1, AUFGABE 1.9 ---
load G_est_00 % save G_est_00 G_est
G_est_500Hz = G_est;
load G_est_01 % save G_est_01 G_est
G_est_5kHz = G_est;
load G_est_02 % save G_est_02 G_est
G_est_10kHz = G_est;

figure(1)
bode(G_est_500Hz, G_est_5kHz, G_est_10kHz, G)
legend('Location', 'best')

% --- P1, AUFGABE 1.11 ---
load data_00.mat % save data_00 data

y_sim = lsim(G, data.values(:,1), data.time);

figure(2)
plot(data.time, [data.values(:,1:3), y_sim]), grid on
ylabel('Voltage (V)'), xlabel('Time (sec)')
legend('Input', ...
    'Output 1 measured', ...
    'Output 2 measured', ...
    'Output 2 simulated', ...
    'Location', 'best')

% --- P1, AUFGABE 1.12 ---
load data_01.mat % save data_01 data

figure(3)
subplot(211)
plot(data.time, data.values(:,1:3)), grid on
ylabel('Voltage (V)')
legend('Setpoint', ...
    'Output 1 measured', ...
    'Control Output 2 measured', ...
    'Location', 'best')
subplot(212)
plot(data.time, data.values(:,4)), grid on
ylabel('Voltage (V)'), xlabel('Time (sec)')
legend('Control Input measured', ...
    'Location', 'best')


%% State space controller and full-state observer

% --- P2, AUFGABE 1.1 ---
% Physical state-space model (x1=uC1, x2=uC2)
A = [-(R1+R2)/(R1*R2*C1),  1/(R2*C1);
               1/(R2*C2), -1/(R2*C2)];
B = [1/(R1*C1);
            0];
C = [0 1];
sys = ss(A, B, C, 0);

% --- P2, AUFGABE 1.2 ---
K = place(A, B, 2 * [-500 + 500*1i; -500 - 500*1i])
fprintf('Matrix<float, 1, 2> K(%1.4ff, %1.4ff);\n', K(1), K(2));

% --- P2, AUFGABE 1.5 ---
V = -1 / (C * (A - B*K)^-1 * B)

load data_02.mat % save data_02 data

figure(4)
subplot(211)
plot(data.time, data.values(:,1:3)), grid on
ylabel('Voltage (V)')
legend('Setpoint', ...
    'Output 1 measured', ...
    'Control Output 2 measured', ...
    'Location', 'best')
subplot(212)
plot(data.time, data.values(:,4)), grid on
ylabel('Voltage (V)'), xlabel('Time (sec)')
legend('Control Input measured', ...
    'Location', 'best')

load data_03.mat % save data_03 data

figure(5)
subplot(211)
plot(data.time, data.values(:,1:3)), grid on
ylabel('Voltage (V)')
legend('Setpoint', ...
    'Output 1 measured', ...
    'Control Output 2 measured', ...
    'Location', 'best')
subplot(212)
plot(data.time, data.values(:,4)), grid on
ylabel('Voltage (V)'), xlabel('Time (sec)')
legend('Control Input measured', ...
    'Location', 'best')

% --- P2, AUFGABE 2.1 ---
H = place(A.', C.', 3 * [-500 + 500*1i; -500 - 500*1i])'
fprintf('m_A << %1.3ff, %1.3ff, %1.3ff, %1.3ff;\n', A(1,1), A(1,2), A(2,1), A(2,2));
fprintf('m_B << %1.3ff, %1.3ff;\n', B(1), B(2));
fprintf('m_C << %1.3ff, %1.3ff;\n', C(1,1), C(1,2));
fprintf('m_H << %1.3ff, %1.3ff;\n', H(1), H(2));

% --- P2, AUFGABE 2.5 & 2.6 ---

% Measured values for state-space controller
load data_04.mat % save data_04 data

figure(6)
subplot(211)
plot(data.time, data.values(:,[1 2 3 5 6])), grid on
ylabel('Voltage (V)')
legend('Setpoint', ...
    'Output 1 measured', ...
    'Output 2 measured', ...
    'Output 1 estimated', ...
    'Output 2 estimated', ...
    'Location', 'best')
subplot(212)
plot(data.time, data.values(:,4)), grid on
ylabel('Voltage (V)'), xlabel('Time (sec)')
legend('Control Input measured', ...
    'Location', 'best')

% Estimated values for state-space controller
load data_05.mat % save data_05 data

figure(7)
subplot(211)
plot(data.time, data.values(:,[1 2 3 5 6])), grid on
ylabel('Voltage (V)')
legend('Setpoint', ...
    'Output 1 measured', ...
    'Output 2 measured', ...
    'Output 1 estimated', ...
    'Output 2 estimated', ...
    'Location', 'best')
subplot(212)
plot(data.time, data.values(:,4)), grid on
ylabel('Voltage (V)'), xlabel('Time (sec)')
legend('Control Input measured', ...
    'Location', 'best')
