%% Compare the theoretical and measured mini cuboid swing-up

% Run mini_cuboid_swingup_ini first.
% Use the initializer settings and torque table flashed for this recording.
% Recording 05 tests the shared fit to runs 03 and 04; do not refit on this run.
% Verified: 500 commands in order, Ts = 1 ms, playback offset = 2 ms.
% Before the 40-deg cutoff (last retained t = 0.359 s):
%   Analytical angle RMSE = 15.51 deg.
%   Shared model with measured initial state: angle RMSE = 15.16 deg,
%   relative wheel RMSE = 11.43 rad/s, gyro-rate RMSE = 181.77 deg/s.
% Over the common first 0.300 s, analytical angle RMSE improves from
% 12.42 deg (run 04) to 8.91 deg (run 05), but tracking remains poor.
param_nominal = get_parameter();
R_test = param.R;  % Effective radius used to generate the trajectory [m]
J2_test = param.J2;  % Effective wheel inertia used to generate the trajectory [kg m^2]
angle_limit = 40*pi/180;  % Stop before opposite-side contact
load(fullfile(fileparts(mfilename('fullpath')), 'data_swingup_05.mat'))

% Match logged torque commands to the table to find the playback start.
% The recording timestamps account for the selected logging rate.
[torque_error, command_index] = min(abs(data.values(:,6) - torque_command(1:end-1).'), [], 2);
matched = torque_error < 1e-6;  % Allow single-precision torque rounding [Nm]
Ts_measured = median(diff(data.time));
% Require most of the expected playback samples, accounting for decimation.
% A few coincidental matches can otherwise accept an older trajectory.
assert(nnz(matched) >= 0.9*min(numel(t)-1, Trun/Ts_measured), ...
    'Too few torque matches; check the recording, trajectory and logging duration.');
assert(all(diff(command_index(matched)) > 0), ...
    'Matched torque commands are not in playback order.');
time_offset = median(data.time(matched) - t(command_index(matched)));
fprintf('Log sampling time = %.6f s, playback offset = %.6f s\n', Ts_measured, time_offset);
t_measured = data.time - time_offset;
inside = t_measured >= -1e-12 & t_measured <= Trun;
t_measured = max(0, t_measured(inside));

% Logged columns: GUI setpoint, commanded current, body angle, body speed,
% relative wheel speed, commanded torque. Current/torque are not measurements.
phi_measured = data.values(inside,3);
omega1_measured = data.values(inside,4);
omega2_measured = data.values(inside,5);
current_logged = data.values(inside,2);
torque_logged = data.values(inside,6);

% Integrate the logged gyro rate, starting at the same angle estimate.
phi_gyro = phi_measured(1) + cumtrapz(t_measured, omega1_measured);

% Keep only the prefix before either angle estimate first reaches 40 deg.
% Do not include later samples when the cube returns below the limit.
contact = find(phi_measured >= angle_limit | phi_gyro >= angle_limit, 1);
if ~isempty(contact)
    keep = 1:contact-1;
    t_measured = t_measured(keep);
    phi_measured = phi_measured(keep);
    phi_gyro = phi_gyro(keep);
    omega1_measured = omega1_measured(keep);
    omega2_measured = omega2_measured(keep);
    current_logged = current_logged(keep);
    torque_logged = torque_logged(keep);
end
t_end = min(Trun, t_measured(end));


%% Parameter comparison using the same torque command

% J2 changes the wheel response; R changes body gravity and pivot inertia.
% These fits can also absorb actuator/geometry errors; they are not measured
% physical parameters. Both models start from the same measured state and
% assume that the recorded torque command is delivered by the actuator.
R_values = [param_nominal.R, R_test];
J2_values = [param_nominal.J2, J2_test];
x_model = cell(2, 1);
x_initial = [phi_measured(1);omega1_measured(1);omega2_measured(1) + omega1_measured(1)];
M_command = @(ti) interp1(t, torque_command, ti, 'previous');
options_test = odeset('RelTol', 1e-8, 'AbsTol', 1e-10, 'MaxStep', Ts/4);
for model = 1:2
    Theta_model = param.J1 + param.m*R_values(model)^2;
    [~, x_model{model}] = ode45(@(ti, x) [x(2); ...
        (param.m*param.g*R_values(model)*sin(x(1)) - M_command(ti))/Theta_model; ...
        M_command(ti)/J2_values(model)], t_measured, x_initial, options_test);
end
fprintf('Comparison ends at %.6f s, before 40 deg.\n', t_end);
for model = 1:2
    fprintf('R = %.6f m, J2 = %.6g kg m^2: angle RMSE = %.3f deg, wheel RMSE = %.3f rad/s\n', ...
        R_values(model), J2_values(model), ...
        sqrt(mean((x_model{model}(:,1) - phi_measured).^2))*180/pi, ...
        sqrt(mean((x_model{model}(:,3) - x_model{model}(:,2) - omega2_measured).^2)));
end


%% Figures

figure('Name', 'Mini cuboid swing-up: theory and measurement')
tiledlayout(3, 2)

nexttile
plot(tfine, phi(tfine)*180/pi, t, x_zoh(:,1)*180/pi, '--', ...
    t_measured, phi_measured*180/pi, t_measured, phi_gyro*180/pi), grid on
xlim([0, t_end])
yline(-45, ':', 'Flat')
xlabel('Time (s)'), ylabel('Body angle (deg)')
legend('Analytical', ...
    'ZOH averaged current', ...
    'Measured (complementary filter)', ...
    'Integrated gyro', ...
    'Location', 'best')

nexttile
plot(tfine, omega1_fun(tfine)*180/pi, t, x_zoh(:,2)*180/pi, '--', ...
    t_measured, omega1_measured*180/pi), grid on
xlim([0, t_end])
xlabel('Time (s)'), ylabel('Body speed (deg/s)')
legend('Analytical', ...
    'ZOH averaged current', ...
    'Measured', ...
    'Location', 'best')

nexttile
plot(t, omega2, t, x_zoh(:,3) - x_zoh(:,2), '--', ...
    t_measured, omega2_measured), grid on
xlim([0, t_end])
xlabel('Time (s)'), ylabel('Relative wheel speed (rad/s)')
legend('Analytical', ...
    'ZOH averaged current', ...
    'Measured (encoder)', ...
    'Location', 'best')

nexttile
plot(tfine, torque(tfine)), hold on
stairs(t, torque_command, '--')
plot(t_measured, torque_logged, '.'), hold off, grid on
xlim([0, t_end])
xlabel('Time (s)'), ylabel('Motor torque (Nm)')
legend('Continuous', ...
    'Interval-averaged command', ...
    'Logged command', ...
    'Location', 'best')

nexttile
plot(tfine, current(tfine)), hold on
stairs(t, double(current_single), '--')
plot(t_measured, current_logged, '.'), hold off, grid on
xlim([0, t_end])
xlabel('Time (s)'), ylabel('Motor current (A)')
legend('Continuous', ...
    'Interval-averaged command', ...
    'Logged command', ...
    'Location', 'best')

nexttile
plot(t, (x_zoh(:,1) - phi(t))*180/pi, '--', ...
    t_measured, (phi_measured - phi(t_measured))*180/pi), grid on
xlim([0, t_end])
xlabel('Time (s)'), ylabel('Body angle error (deg)')
legend('Averaged ZOH - analytical', ...
    'Measured - analytical', ...
    'Location', 'best')
sgtitle('Ideal free-pivot theory compared with the hardware recording')


%% Effect of changing R and J2

figure('Name', 'Mini cuboid swing-up: parameter comparison')
tiledlayout(3, 1)

nexttile
plot(t_measured, phi_measured*180/pi, ...
    t_measured, x_model{1}(:,1)*180/pi, '--', ...
    t_measured, x_model{2}(:,1)*180/pi), grid on
xlim([0, t_end])
xlabel('Time (s)'), ylabel('Body angle (deg)')
legend('Measured (complementary filter)', ...
    'Nominal parameters', ...
    'Test parameters', ...
    'Location', 'best')

nexttile
plot(t_measured, omega1_measured*180/pi, ...
    t_measured, x_model{1}(:,2)*180/pi, '--', ...
    t_measured, x_model{2}(:,2)*180/pi), grid on
xlim([0, t_end])
xlabel('Time (s)'), ylabel('Body speed (deg/s)')
legend('Measured (gyro)', ...
    'Nominal parameters', ...
    'Test parameters', ...
    'Location', 'best')

nexttile
plot(t_measured, omega2_measured, ...
    t_measured, x_model{1}(:,3) - x_model{1}(:,2), '--', ...
    t_measured, x_model{2}(:,3) - x_model{2}(:,2)), grid on
xlim([0, t_end])
xlabel('Time (s)'), ylabel('Relative wheel speed (rad/s)')
legend('Measured (encoder)', ...
    'Nominal parameters', ...
    'Test parameters', ...
    'Location', 'best')
sgtitle(sprintf('Same torque input: R = %.4f m, J2 = %.3g kg m^2', R_test, J2_test))
