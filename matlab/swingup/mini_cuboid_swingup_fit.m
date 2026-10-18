%% Fit one R and J2 to swing-up recordings 03 and 04

% Run 05 is a separate validation experiment, excluded from this fit.
% It still shows substantial tracking error; see mini_cuboid_swingup_notes.md.

% Does not regenerate or export a trajectory. Uses the logged torque commands,
% not measured motor torque. All other parameters remain from get_parameter.
% R: minimize mean squared body-angle error, with equal weight for each run.
% J2: at that R, minimize mean squared relative wheel-speed error, again with
% equal weight for each run. The gyro rate is checked separately, not fitted.

matlab_dir_fit = fileparts(fileparts(mfilename('fullpath')));
addpath(matlab_dir_fit);
param_fit = get_parameter();
files_fit = {'data_swingup_03.mat', 'data_swingup_04.mat'};
% Both logs were checked against all 500 exported commands: the first torque
% sample is row 3 (2 ms after logging begins). These indices are run-specific.
first_sample_fit = [3, 3];
angle_limit_fit = 40*pi/180;
recordings_fit = cell(2, 1);

for run_index = 1:2
    logged = load(fullfile(fileparts(mfilename('fullpath')), files_fit{run_index}));
    rows = first_sample_fit(run_index):first_sample_fit(run_index) + 499;
    recording.t = logged.data.time(rows) - logged.data.time(rows(1));
    recording.values = logged.data.values(rows,:);
    recording.phi_gyro = recording.values(1,3) + ...
        cumtrapz(recording.t, recording.values(:,4));
    contact = find(recording.values(:,3) >= angle_limit_fit | ...
        recording.phi_gyro >= angle_limit_fit, 1);
    if ~isempty(contact)
        keep = 1:contact-1;
        recording.t = recording.t(keep);
        recording.values = recording.values(keep,:);
        recording.phi_gyro = recording.phi_gyro(keep);
    end
    recording.impulse = [0;cumsum(recording.values(1:end-1,6).*diff(recording.t))];
    recordings_fit{run_index} = recording;
end

% Coarse scan checks for other minima before refining the best interval.
R_grid_fit = linspace(0.02, 0.10, 81);
cost_fit = arrayfun(@(R) angle_cost(R, recordings_fit, param_fit), R_grid_fit);
[~, best_fit] = min(cost_fit);
assert(best_fit > 1 && best_fit < numel(R_grid_fit), 'Best radius is at scan boundary.');
R_fit = fminbnd(@(R) angle_cost(R, recordings_fit, param_fit), ...
    R_grid_fit(best_fit-1), R_grid_fit(best_fit+1), optimset('TolX', 1e-10));

% Absolute wheel speed = relative encoder speed + body speed.
% Predicted absolute wheel speed = initial absolute speed + impulse/J2.
% Solve the linear least-squares problem in 1/J2 analytically.
numerator_fit = 0;
denominator_fit = 0;
for run_index = 1:2
    recording = recordings_fit{run_index};
    body_fit = simulate_body(R_fit, recording, param_fit);
    omega_abs_initial = recording.values(1,5) + recording.values(1,4);
    target_fit = recording.values(:,5) + body_fit(:,2) - omega_abs_initial;
    numerator_fit = numerator_fit + mean(recording.impulse.*target_fit);
    denominator_fit = denominator_fit + mean(recording.impulse.^2);
end
J2_fit = denominator_fit/numerator_fit;
assert(isfinite(J2_fit) && J2_fit > 0, 'No positive wheel-inertia fit.');
fprintf('\nShared fit: R = %.9f m, J2 = %.9g kg m^2\n', R_fit, J2_fit);


%% Compare both runs using the same shared parameters

figure('Name', 'Mini cuboid: shared fit to recordings 03 and 04')
tiledlayout(3, 2)
for run_index = 1:2
    recording = recordings_fit{run_index};
    body_fit = simulate_body(R_fit, recording, param_fit);
    omega_abs_initial = recording.values(1,5) + recording.values(1,4);
    wheel_fit = omega_abs_initial + recording.impulse/J2_fit - body_fit(:,2);
    fprintf('%s, through %.3f s: angle RMSE = %.3f deg, gyro RMSE = %.3f deg/s, wheel RMSE = %.3f rad/s\n', ...
        files_fit{run_index}, recording.t(end), ...
        sqrt(mean((body_fit(:,1) - recording.values(:,3)).^2))*180/pi, ...
        sqrt(mean((body_fit(:,2) - recording.values(:,4)).^2))*180/pi, ...
        sqrt(mean((wheel_fit - recording.values(:,5)).^2)));

    nexttile(run_index)
    plot(recording.t, recording.values(:,3)*180/pi, ...
        recording.t, recording.phi_gyro*180/pi, ':', ...
        recording.t, body_fit(:,1)*180/pi, '--'), grid on
    xlabel('Time (s)'), ylabel('Body angle (deg)')
    title(files_fit{run_index}, 'Interpreter', 'none')
    legend('Measured (complementary filter)', ...
        'Integrated gyro', ...
        'Shared fit', ...
        'Location', 'best')

    nexttile(run_index + 2)
    plot(recording.t, recording.values(:,4)*180/pi, ...
        recording.t, body_fit(:,2)*180/pi, '--'), grid on
    xlabel('Time (s)'), ylabel('Body speed (deg/s)')
    legend('Measured (gyro)', 'Shared fit', 'Location', 'best')

    nexttile(run_index + 4)
    plot(recording.t, recording.values(:,5), recording.t, wheel_fit, '--'), grid on
    xlabel('Time (s)'), ylabel('Relative wheel speed (rad/s)')
    legend('Measured (encoder)', 'Shared fit', 'Location', 'best')
end
sgtitle(sprintf('Shared effective parameters: R = %.6f m, J2 = %.6g kg m^2', R_fit, J2_fit))


%% Local helpers

function cost = angle_cost(R, recordings, param)
    cost = 0;
    for run_index = 1:numel(recordings)
        recording = recordings{run_index};
        body = simulate_body(R, recording, param);
        cost = cost + mean((body(:,1) - recording.values(:,3)).^2)/numel(recordings);
    end
end

function body = simulate_body(R, recording, param)
    Theta = param.J1 + param.m*R^2;
    alpha = param.m*param.g*R/Theta;
    body = zeros(numel(recording.t), 2);
    body(1,:) = recording.values(1,3:4);
    for k = 1:numel(recording.t)-1
        h = (recording.t(k+1) - recording.t(k))/4;
        state = body(k,:).';
        M = recording.values(k,6);
        rhs = @(x) [x(2);alpha*sin(x(1)) - M/Theta];
        for step = 1:4
            k1 = rhs(state);
            k2 = rhs(state + h*k1/2);
            k3 = rhs(state + h*k2/2);
            k4 = rhs(state + h*k3);
            state = state + h*(k1 + 2*k2 + 2*k3 + k4)/6;
        end
        body(k+1,:) = state.';
    end
end
