%% Mini cuboid: combined analytical swing-up from paper 2, Section 4.3

% Henrici & Robbiani, Mathematics 2024, 12, 3675.
% https://doi.org/10.3390/math12233675
%
% Run this script to create figures and the struct swingup. No firmware or
% Simulink model is changed. Requires base MATLAB, no Symbolic/Optimization
% Toolbox. Local functions below evaluate analytical derivatives.
%
% Convention: phi=-pi/4 initially, phi=0 upright; positive motor torque
% accelerates the wheel in the world frame and acts negatively on the body.
% The encoder measures wheel speed RELATIVE to the body.
%
% This is the paper's ideal free-pivot model. It includes neither contact
% with the table, friction nor motor dynamics. In particular, the zero-torque
% start allows an initial dip below -pi/4. The plots report this limitation.
% Spin-up before t=0 and the subsequent balancing controller are separate.


%% Physical parameters for the simulation

% Load shared mechanical parameters; add the motor constant used by the firmware.
matlab_dir = fileparts(fileparts(mfilename('fullpath')));
addpath(matlab_dir);
param = get_parameter();
param.km = 36.9e-3;  % Motor torque constant [Nm/A]


%% Simulation settings and initial conditions

Trun = 0.5;  % Swing-up run time [s], paper's example duration tau
Ts = 1e-3;  % Firmware and From Workspace sampling period [s]
current_limit = 15;  % Firmware limit [A], no clipping

% Fixed starting configuration for the paper's swing-up construction.
phi1_0 = -pi/4;  % Initial body angle [rad]
omega1_0 = 0;  % Initial body angular velocity [rad/s]
phi2_0 = 0;  % Initial flywheel angle relative to the body [rad]
% omega2_0 [rad/s] is CALCULATED below from the torque integral; it is
% not independent of Trun and the shape parameters.
%
% Simulink settings:
%   Stop time: Trun; maximum solver step: Ts
%   Phi 1: position target phi1_0 [rad], velocity target omega1_0 [rad/s]
%   Phi 2: position target phi2_0 [rad], velocity target omega2_0 [rad/s]
% Enable the corresponding joint targets and set their units accordingly.


%% Soft constraints in mini_cuboid_simscape_swingup_sim.slx

% Phi 1, Limits: the same settings are used at both sides.
%   Lower / upper bound: -pi/4 / pi/4 [rad]
%   Spring stiffness: 100 [N*m/rad]
%   Damping coefficient: 1 [N*m*s/rad]
%   Transition region width: 5e-4 [rad]
% These are assumed compliant-contact settings, not parameters from paper 2
% or measured O-ring properties. The transition width smooths contact onset.
% At phi1_0=-pi/4 the stop starts with zero penetration and zero spring
% preload. A real cube resting on O-rings is already supported/compressed;
% its contact geometry and preload require separate experimental validation.
% The MATLAB analytical/ODE curves below exclude these stops. A Simscape
% trajectory with contact is therefore not expected to match them exactly.
% https://www.mathworks.com/help/sm/ref/revolutejoint.html


%% Adjustable parameters: paper 2, Section 4.3 example

% Selected test: ramp_slope=800; the paper's example uses 200.
% The other settings retain the paper's example choices. These are tuning parameters;
% the fitted coefficients, blend weights and initial wheel speed below
% are calculated from them using OUR cuboid's physical parameters.
% Thus this uses the paper's construction, not its example alpha and beta.
ramp_slope = 800;  % Tuned ramp: paper's m [1/s], distinct from mass param.m
ramp_power = 1;  % Initial ramp: paper's n (denominator exponent)
initial_excursion = -4;  % Eq. (25): coefficient of s^2*(s-1)^6 [rad]
final_excursion = 3;  % Eq. (26): coefficient of s^6*(s-1)^2 [rad]

assert(phi1_0 == -pi/4 && omega1_0 == 0, ...
    'This trajectory requires a stationary body initially at -pi/4.');
assert(isfinite(Trun) && Trun > 0 && isfinite(Ts) && Ts > 0);
N = round(Trun / Ts);
assert(N >= 1 && abs(N*Ts - Trun) <= 1e-12*max(1, Trun), ...
    'Choose Trun to be an integer multiple of Ts; duration is not rounded.');
assert(ramp_slope > 0 && ramp_power >= 1 && mod(ramp_power, 1) == 0);
assert(all([param.J1, param.J2, param.m, param.R, param.g, param.km] > 0));

Theta = param.J1 + param.m*param.R^2;  % Body inertia about pivot [kg m^2]
alpha = param.m*param.g*param.R/Theta;  % Gravity coefficient [1/s^2]
beta = Theta/param.J2;


%% Analytical reference and fifth-order Hermite approximation, Eq. (18)

% All shapes below use NORMALIZED time s=t/Trun. For the elliptic reference,
% the reciprocal-parameter identity keeps ellipj's parameter in [0,1].
% Reference: paper 2, Eq. (4); Hermite basis: Eqs. (16)-(17).
% Fit the coefficients using Eqs. (19)-(21), rather than rounded Eq. (22) values.
k = sin(pi/16);
K = ellipke(k^2);
phi_reference = @(s) 2*asin(k*ellipj(K*(2*s - 1), k^2)) - pi/8;
h00 = polynomial_shape((-pi/4)*[-6, 15, -10, 0, 0, 1]);
h20 = polynomial_shape([-0.5, 1.5, -1.5, 0.5, 0, 0]);
h21 = polynomial_shape([0.5, -1, 0.5, 0, 0, 0]);
H = {h20, h21};
a5 = fit_correction(H, @(s) phi_reference(s) - h00.q(s));
phi2 = combine_shapes([{h00}, H], [1;a5]);


%% Initial and final excursions, Eqs. (25) and (26)

% Fit INCREMENTS to phi2, as in the equations, rather than copying the
% printed coefficients (which contain inconsistent labels/numbers).
theta13 = polynomial_shape(initial_excursion*[1, -6, 15, -20, 15, -6, 1, 0, 0]);
theta23 = polynomial_shape(final_excursion*[1, -2, 1, 0, 0, 0, 0, 0, 0]);
b13 = fit_correction(H, @(s) phi_reference(s) - phi2.q(s) - theta13.q(s));
b23 = fit_correction(H, @(s) phi_reference(s) - phi2.q(s) - theta23.q(s));
phi13 = combine_shapes([{phi2}, H, {theta13}], [1;b13;1]);
phi23 = combine_shapes([{phi2}, H, {theta23}], [1;b23;1]);


%% Initial ramp, Eq. (27): enforce M(0)=0, then minimize angle deviation

% Normalized versions of the paper's r_j(t). Factors Trun^6 are absorbed in
% the fitted coefficients, so these coefficients have units rad and cannot
% be compared directly with coefficients for unnormalized time.
rho = ramp_slope*Trun;
r0 = rational_shape([1, -4, 6, -4, 1, 0, 0], rho, ramp_power);
r1 = polynomial_shape([1, -3, 3, -1, 0, 0, 0]);
r2 = polynomial_shape([1, -2, 1, 0, 0, 0, 0]);
initial_acceleration_s = Trun^2*alpha*sin(phi1_0);
a0 = (initial_acceleration_s - phi2.ddq(0))/r0.ddq(0);
a12 = fit_correction({r1, r2}, @(s) -a0*r0.q(s));
phi3 = combine_shapes({phi2, r0, r1, r2}, [1;a0;a12]);


%% Final blend: minimize ||phi4-phi2|| with Eqs. (30) and (31)

% The intended Eq. (30) is c3+c13+c23=1. Enforce this and M(0)=0
% explicitly. Negative weights are allowed, as in the paper.
parts = {phi3, phi13, phi23};
deviations = cellfun(@(p) @(s) p.q(s) - phi2.q(s), parts, ...
    'UniformOutput', false);
G = gram_matrix(deviations);
acc = cellfun(@(p) p.ddq(0), parts);
scale = max([1, abs(acc), abs(initial_acceleration_s)]);
Aeq = [ones(1, 3);acc/scale];
beq = [1;initial_acceleration_s/scale];
assert(rank(Aeq) == 2, 'Blend constraints are not independent.');
c0 = Aeq.'*((Aeq*Aeq.')\beq);
Z = null(Aeq);
assert(all(eig(Z.'*G*Z) > 0), 'Blend objective has no unique minimum.');
c = c0 - Z*((Z.'*G*Z)\(Z.'*G*c0));
phi4 = combine_shapes(parts, c);


%% Continuous functions, valid only for 0 <= t <= Trun

phi = @(t) phi4.q(t/Trun);
omega1_fun = @(t) phi4.dq(t/Trun)/Trun;
phi_ddot = @(t) phi4.ddq(t/Trun)/Trun^2;
% Paper 2, Eq. (8): physical torque M=Theta*f; derivatives use seconds.
torque = @(t) param.m*param.g*param.R*sin(phi(t)) - Theta*phi_ddot(t);
current = @(t) torque(t)/param.km;

% Paper 2, integral condition (3): choose the initial wheel speed so the
% wheel stops at t=Trun. At both ends
% the body is stationary, so absolute and relative wheel speeds coincide.
torque_impulse = integral(torque, 0, Trun, 'AbsTol', 1e-12, 'RelTol', 1e-11);
omega2_abs_0 = -torque_impulse/param.J2;
assert(omega2_abs_0 > 0, 'The torque integral must be negative for this convention.');
omega2_0 = omega2_abs_0 - omega1_0;  % Joint velocity relative to the body [rad/s]


%% Independently integrate the nonlinear dynamics using continuous torque

options = odeset('RelTol', 1e-10, 'AbsTol', 1e-11, 'MaxStep', Ts);
x0 = [phi1_0;omega1_0;omega2_abs_0];  % [body angle; body speed; absolute wheel speed]
ode_solution = ode45(@(t, x) cuboid_rhs(x, torque(t), alpha, Theta, param.J2), ...
    [0, Trun], x0, options);


%% Sampled vectors for later Simulink and C++ use

% The analytical samples below remain point values of the continuous curve.
% The held commands are interval averages, calculated separately below.
% N intervals, N+1 values: command k acts on [t(k),t(k+1)). The final
% command is the terminal torque, used when holding the value after Trun.
% The block can hold the final torque after Trun, as configured below. That
% continuation is separate from the analytical swing-up on [0,Trun].
t = (0:N).'*Ts;
t(end) = Trun;  % Prevent roundoff from placing the endpoint beyond the ODE solution
x_continuous = deval(ode_solution, t.').';
phi_samples = phi(t);
omega1 = omega1_fun(t);  % Body angular velocity [rad/s]
acceleration_samples = phi_ddot(t);
torque_samples = torque(t);
current_samples = current(t);
omega2_abs = x_continuous(:,3);  % Flywheel angular velocity in world frame [rad/s]
omega2 = omega2_abs - omega1;  % Flywheel angular velocity relative to body [rad/s]


%% Interval-averaged torque for zero-order hold

% M_k = integral(M(t), t(k), t(k+1)) / (t(k+1)-t(k)). This preserves the
% continuous torque impulse in each interval before float quantization.
% The analytical curve and paper parameters above are unchanged. The first
% held command is an interval average, so it generally differs from M(0)=0.
torque_average = zeros(N + 1, 1);
for interval = 1:N
    torque_average(interval) = integral(torque, t(interval), t(interval+1), ...
        'AbsTol', 1e-13, 'RelTol', 1e-11) / (t(interval+1) - t(interval));
end
torque_average(end) = torque(Trun);  % Terminal hold value, no interval after Trun
current_average = torque_average / param.km;

% Use the same single-precision current commands for the MATLAB prediction,
% Simulink torque input and later C++ playback. No saturation is applied.
current_single = single(current_average);
torque_command = double(current_single) * param.km;
x_zoh = integrate_zoh(t, torque_command, x0, alpha, Theta, param.J2, 4);


%% Simulink input: From Workspace

% Connect to the model's M (Nm) input. Columns: [time (s), torque (Nm)].
% From Workspace block settings:
%   Data: simin
%   Sample time: Ts
%   Interpolate data: off (zero-order hold)
%   Form output after final data value by: Holding final value
% https://www.mathworks.com/help/simulink/slref/fromworkspace.html
% The held final command approximates torque(Trun), with single-precision
% current quantization. It is separate from the subsequent balancing control.
simin = [t, torque_command];


%% Results and reference signals

swingup = struct();
swingup.param = param;
swingup.Trun = Trun;
swingup.Ts = Ts;
swingup.Theta = Theta;
swingup.alpha = alpha;
swingup.beta = beta;
swingup.omega1_0 = omega1_0;
swingup.omega2_0 = omega2_0;
swingup.omega2_abs_0 = omega2_abs_0;
swingup.current_limit = current_limit;
swingup.initial_state = x0;  % [phi; omega1; omega2_abs]
swingup.simulation_initial_conditions = struct('phi1_0', phi1_0, ...
    'omega1_0', omega1_0, 'phi2_0', phi2_0, 'omega2_0', omega2_0);
swingup.coefficients = struct('a5', a5, 'b13', b13, 'b23', b23, ...
    'ramp', [a0;a12], 'blend', c, 'rho', rho, 'ramp_power', ramp_power, ...
    'initial_excursion', initial_excursion, 'final_excursion', final_excursion);
swingup.continuous = struct('phi', phi, 'omega1', omega1_fun, ...
    'phi_ddot', phi_ddot, 'torque', torque, 'current', current);
swingup.samples = struct('t', t, 'phi', phi_samples, 'omega1', omega1, ...
    'phi_ddot', acceleration_samples, 'torque', torque_samples, ...
    'current', current_samples, 'omega2_abs', omega2_abs, 'omega2', omega2);
swingup.commands = struct('t', t, 'sampling', 'interval_average', ...
    'torque_average', torque_average, 'current_average', current_average, ...
    'current_single', current_single, 'torque', torque_command);
swingup.current_single = current_single; % Same index/time grid as samples.t
% From Workspace accepts these [time, value...] matrices. Disable
% interpolation for ZOH playback; continuous handles above are the exact form.
swingup.simulink.torque = simin;
swingup.simulink.current = [t, double(current_single)];
swingup.simulink.reference = [t, phi_samples, omega1, omega2];
swingup.validation.continuous_state = x_continuous;
swingup.validation.zoh_state = x_zoh;


%% Checks and physical limitations

edge = [phi(0)-phi1_0;phi(Trun);omega1_fun(0)-omega1_0;omega1_fun(Trun)];
assert(max(abs(edge)) < 1e-9, 'Angle/velocity boundary conditions failed.');
assert(norm(Aeq*c - beq, inf) < 1e-10, 'Blend constraints failed.');
assert(abs(torque(0)) < 1e-9, 'Initial continuous motor torque is not zero.');
assert(abs(sum(torque_average(1:N).*diff(t)) - torque_impulse) < 1e-10, ...
    'Interval-averaged torque does not preserve the continuous impulse.');
assert(max(abs(x_continuous(:,1) - phi_samples)) < 1e-7, ...
    'Continuous nonlinear model does not reproduce the analytical angle.');
assert(max(abs(x_continuous(:,2) - omega1)) < 1e-6, ...
    'Continuous nonlinear model does not reproduce the analytical velocity.');
assert(abs(omega2_abs(end)) < 1e-6, 'Continuous wheel stopping condition failed.');

% Resolve the short initial ramp on a denser grid for plots/limit checks.
tfine = linspace(0, Trun, max(10001, N + 1)).';
phi_fine = phi(tfine);
current_fine = current(tfine);
swingup.validation.peak_current = max(abs(current_fine));
swingup.validation.minimum_phi = min(phi_fine);
swingup.validation.peak_command_current = max(abs(double(current_single)));
swingup.validation.current_within_limit = ...
    max(abs(current_fine)) <= current_limit && ...
    swingup.validation.peak_command_current <= current_limit;
swingup.validation.command_impulse_error = ...
    sum(torque_command(1:N).*diff(t)) - torque_impulse;
swingup.validation.contact_compatible = min(phi_fine) >= -pi/4 - 1e-9;
swingup.validation.final_zoh_error = x_zoh(end,:); % Target is [0,0,0]
swingup.validation.zoh_tracks_within_one_degree = ...
    max(abs(x_zoh(:,1) - phi_samples)) <= pi/180;

fprintf('\nPaper 2 combined swing-up, using mini cuboid parameters\n');
fprintf('Trun = %.6f s, Ts = %.6f s, %d intervals / %d samples\n', Trun, Ts, N, N + 1);
fprintf('alpha = %.6f 1/s^2, beta = %.6f\n', alpha, beta);
fprintf('Initial wheel speed = %.6f rad/s (%.1f rpm)\n', omega2_0, omega2_0*60/(2*pi));
fprintf('Blend [c3 c13 c23] = [%.9f %.9f %.9f]\n', c);
fprintf('Peak current on dense grid = %.6f A (limit %.1f A)\n', ...
    swingup.validation.peak_current, current_limit);
fprintf('Minimum body angle = %.6f deg (flat = -45 deg)\n', min(phi_fine)*180/pi);
fprintf('Torque at Trun, before handover = %.6f Nm\n', torque(Trun));
fprintf('Continuous ODE max angle error = %.3g rad\n', ...
    max(abs(x_continuous(:,1) - phi_samples)));
fprintf('ZOH terminal [rad, rad/s, rad/s] = [%.6g %.6g %.6g]\n', x_zoh(end,:));
fprintf('Averaged ZOH max body angle error = %.6f deg\n', ...
    max(abs(x_zoh(:,1) - phi_samples))*180/pi);
fprintf('Peak held current = %.6f A, initial held current = %.6f A\n', ...
    swingup.validation.peak_command_current, double(current_single(1)));
if ~swingup.validation.current_within_limit
    warning('swingup:CurrentLimit', ...
        'Trajectory exceeds the current limit. Commands have NOT been clipped.');
end
if ~swingup.validation.zoh_tracks_within_one_degree
    warning('swingup:SampledTracking', ...
        ['ZOH playback deviates by more than 1 degree from the analytical ', ...
         'trajectory. Sampled control needs further design before hardware use.']);
end
if ~swingup.validation.contact_compatible
    warning('swingup:Contact', ...
        ['Ideal trajectory dips below -45 deg. It is not directly executable ', ...
         'from rest on a table; contact/release needs separate treatment.']);
end


%% Figures

figure('Name', 'Paper 2: combined mini cuboid swing-up')
tiledlayout(3, 2)

nexttile
plot(tfine, phi_fine*180/pi, t, x_zoh(:,1)*180/pi, '--'), grid on
yline(-45, ':', 'Flat')
xlabel('Time (s)'), ylabel('Body angle (deg)')
legend('Analytical', ...
    'ZOH averaged current', ...
    'Location', 'best')

nexttile
plot(tfine, omega1_fun(tfine)*180/pi, t, x_zoh(:,2)*180/pi, '--'), grid on
xlabel('Time (s)'), ylabel('Body speed (deg/s)')
legend('Analytical', ...
    'ZOH averaged current', ...
    'Location', 'best')

nexttile
plot(t, [omega2, omega2_abs]), grid on
xlabel('Time (s)'), ylabel('Wheel speed (rad/s)')
legend('Relative to body (encoder)', ...
    'Absolute (world)', ...
    'Location', 'best')

nexttile
plot(tfine, torque(tfine)), hold on
stairs(t, torque_command, '--'), hold off, grid on
xlabel('Time (s)'), ylabel('Motor torque (Nm)')
legend('Continuous', ...
    'Interval-averaged command', ...
    'Location', 'best')

nexttile
plot(tfine, current_fine), hold on
stairs(t, double(current_single), '--'), hold off, grid on
yline(current_limit, ':')
yline(-current_limit, ':')
xlabel('Time (s)'), ylabel('Motor current (A)')
legend('Continuous', ...
    'Interval average (single precision)', ...
    'Location', 'best')

nexttile
plot(t, (x_continuous(:,1) - phi_samples)*180/pi, ...
    t, (x_zoh(:,1) - phi_samples)*180/pi), grid on
xlabel('Time (s)'), ylabel('Body angle error (deg)')
legend('Continuous ODE - analytical', ...
    'Averaged ZOH - analytical', ...
    'Location', 'best')
sgtitle('Ideal free pivot; spin-up, table contact and balancing excluded')

figure('Name', 'Paper 2: construction of the combined trajectory')
tiledlayout(2, 1)
s = tfine/Trun;

nexttile
plot(tfine, [phi_reference(s), phi2.q(s), phi4.q(s)]*180/pi), grid on
xlabel('Time (s)'), ylabel('Body angle (deg)')
legend('Elliptic reference', ...
    'Fifth-order approximation', ...
    'Final blend', ...
    'Location', 'best')

nexttile, hold on
for j = 1:numel(parts)
    plot(tfine, param.m*param.g*param.R*sin(parts{j}.q(s)) - Theta*parts{j}.ddq(s)/Trun^2)
end
plot(tfine, torque(tfine), 'k', 'LineWidth', 1.5), hold off, grid on
xlabel('Time (s)'), ylabel('Motor torque (Nm)')
legend('Ramp component', ...
    'Initial excursion component', ...
    'Final excursion component', ...
    'Final blend', ...
    'Location', 'best')


%% Local analytical and numerical helpers

function p = polynomial_shape(coeff)
    dcoeff = polyder(coeff);
    ddcoeff = polyder(dcoeff);
    p.q = @(s) polyval(coeff, s);
    p.dq = @(s) polyval(dcoeff, s);
    p.ddq = @(s) polyval(ddcoeff, s);
end

function r = rational_shape(coeff, rho, n)
    p = polynomial_shape(coeff);
    r.q = @(s) p.q(s)./(1 + rho*s).^n;
    r.dq = @(s) p.dq(s)./(1 + rho*s).^n ...
        - n*rho*p.q(s)./(1 + rho*s).^(n + 1);
    r.ddq = @(s) p.ddq(s)./(1 + rho*s).^n ...
        - 2*n*rho*p.dq(s)./(1 + rho*s).^(n + 1) ...
        + n*(n + 1)*rho^2*p.q(s)./(1 + rho*s).^(n + 2);
end

function p = combine_shapes(parts, weights)
    p.q = @(s) evaluate_combination(parts, weights, s, 'q');
    p.dq = @(s) evaluate_combination(parts, weights, s, 'dq');
    p.ddq = @(s) evaluate_combination(parts, weights, s, 'ddq');
end

function y = evaluate_combination(parts, weights, s, field)
    y = zeros(size(s));
    for j = 1:numel(parts)
        f = parts{j}.(field);
        y = y + weights(j)*f(s);
    end
end

function G = gram_matrix(functions)
    G = zeros(numel(functions), numel(functions));
    for j = 1:numel(functions)
        for k = j:numel(functions)
            G(j,k) = inner_product(functions{j}, functions{k});
            G(k,j) = G(j,k);
        end
    end
end

function value = inner_product(f, g)
    value = integral(@(s) f(s).*g(s), 0, 1, 'AbsTol', 1e-13, 'RelTol', 1e-10);
end

function weights = fit_correction(basis, target)
    functions = cellfun(@(p) p.q, basis, 'UniformOutput', false);
    rhs = cellfun(@(f) inner_product(f, target), functions).';
    weights = gram_matrix(functions)\rhs;
end

function dx = cuboid_rhs(x, M, alpha, Theta, J2)
    dx = [x(2);alpha*sin(x(1))-M/Theta;M/J2];
end

function x = integrate_zoh(t, M, x0, alpha, Theta, J2, substeps)
    x = zeros(numel(t), 3);
    x(1,:) = x0.';
    for k = 1:numel(t)-1
        h = (t(k + 1) - t(k))/substeps;
        state = x(k,:).';
        f = @(state) cuboid_rhs(state, M(k), alpha, Theta, J2);
        for j = 1:substeps
            k1 = f(state);
            k2 = f(state + h*k1/2);
            k3 = f(state + h*k2/2);
            k4 = f(state + h*k3);
            state = state + h*(k1 + 2*k2 + 2*k3 + k4)/6;
        end
        x(k + 1,:) = state.';
    end
end
