# Swing-up measurements and shared-parameter fit — 2026-10-18

The paper 2 analytical construction generates a consistent ideal free-pivot
trajectory, but the measured swing-ups are not yet reproduced reliably by that
model. A shared fit of `R` and `J2` improves the combined error but leaves substantial
errors in both fitted runs. A further hardware test, run 05, gives some improvement
but still substantial tracking error. The shared fit is not a validated plant model.

## Experiments

All three runs use `Trun = 0.5 s`, `Ts = 0.001 s`, 500 interval-averaged torque commands,
`ramp_slope = 200`, `ramp_power = 1`, `initial_excursion = -4`, and
`final_excursion = 3`. The firmware ramps the wheel-speed setpoint over 3 s, then
accepts a nonzero GUI setpoint as the playback trigger. Balancing is disabled.

| Recording | R used to generate commands (m) | J2 used to generate commands (kg m²) | Initial wheel-speed target (rad/s) | Peak commanded current (A) |
|---|---:|---:|---:|---:|
| `data_swingup_03.mat` | 0.066 | 2.81e-4 | 346.290 | 12.565 |
| `data_swingup_04.mat` | 0.0616 | 3.41e-4 | 266.344 | 11.653 |
| `data_swingup_05.mat` | 0.059864129 | 3.31487857e-4 | 266.269 | 11.298 |

The current peaks above are the held commands. None of the recordings measures actual
motor current or torque. All 500 logged torques in each recording were checked
against its corresponding generated table and matched in order, within 1e-6 Nm.
Playback begins at row 3, 2 ms after logging starts, in all three recordings. The
measured initial relative wheel speeds were 346.944, 267.209 and 266.264 rad/s for
runs 03, 04 and 05, respectively.

The fitted values used for run 04 came from run 03. Their excellent body-angle fit
to run 03 did not transfer to run 04. At 0.300 s in run 04, the analytical angle is
−15.54°, the model with measured initial conditions predicts approximately −15.19°,
and the measured angle is +17.00°.

## Shared fit

Run [mini_cuboid_swingup_fit.m](mini_cuboid_swingup_fit.m) directly; it does not need
the initializer and does not export a header. It uses each recording's logged
commands and its own measured initial body angle, body angular velocity and
relative wheel velocity. The start indices in that script are specific to these
two checked recordings.

Keep the prefix before either the complementary-filter angle or the integrated
gyro first reaches +40°. Do not include samples returning below that threshold
later. The retained endpoints are 0.308 s for run 03 and 0.330 s for run 04.
This excludes the observed opposite-side impact region; it does not remove the
initial table-contact phase or model rounded-corner contact.

The fixed parameters are `J1 = 7.66e-4 kg m²`, `m = 0.816 kg`, `g = 9.81 m/s²` and
`km = 0.0369 Nm/A`. The forward model is:

```text
Theta = J1 + m*R²
body acceleration = (m*g*R*sin(body angle) - commanded torque)/Theta
absolute wheel acceleration = commanded torque/J2
relative wheel velocity = absolute wheel velocity - body velocity
```

First fit `R` to the mean of the two runs' mean squared body-angle errors, giving
each run equal weight. Scan 0.02–0.10 m before refining the best interval. At that
`R`, fit `1/J2` analytically to minimise the equally weighted mean squared relative
wheel-speed errors. Report gyro-rate errors separately; they are not part of the
fit objective. No torque gains, delays, offsets or other physical parameters are
adjusted.

The resulting shared effective parameters are:

```matlab
R_fit = 0.059864129;  % m
J2_fit = 3.31487857e-4;  % kg m^2
```

| Model parameters | Run 03 angle RMSE (°) | Run 04 angle RMSE (°) | Run 03 relative wheel RMSE (rad/s) | Run 04 relative wheel RMSE (rad/s) |
|---|---:|---:|---:|---:|
| Nominal: R=0.066, J2=2.81e-4 | 16.87 | 33.92 | 37.96 | 24.34 |
| Run 03 fit: R=0.0616, J2=3.41e-4 | 0.41 | 16.52 | 6.83 | 15.54 |
| Shared fit | 8.40 | 7.82 | 9.03 | 9.47 |

The shared fit's body-rate RMSE is 71.49°/s for run 03 and 110.04°/s for run 04.
These are RMS errors, not bounds on instantaneous error.

As a separate check, choosing `R` to minimise the worse of the two angle RMSEs
gives `R ≈ 0.05992057 m`, leaving approximately 8.12° RMS in each run. Changing
`J2` cannot reduce these body-angle errors in this fixed-command model, since `J2`
does not enter its body equation. Fits over common 0.300 s and early 0.200 s windows,
and a fit to gyro rate instead of angle, also fail to provide a close match to both
complete retained records.

## Interpretation and stopping point

The shared pair is a compromise, not a validated estimate of physical radius and
wheel inertia. Run 03 becomes worse while run 04 improves relative to the previous
fit. A constant `R` and `J2` alone do not give a convincing common explanation of
these two experiments under the assumed model.

The logs do not identify the cause uniquely. Rounded-corner/table contact,
actuator torque delivery, friction and sensor dynamics remain possible contributors.
The complementary-filter angle and integrated gyro also disagree during motion;
neither is an independent ground-truth angle measurement. The initial wheel-speed
errors and a wrong exported command table do not explain the observed discrepancy.

## Run 05: independent hardware check of the shared fit

The initializer uses the shared fit above; run 05 was not included in that fit.
Duration, sampling and shape settings remain unchanged. The initial wheel-speed
target is 266.269123 rad/s (2542.7 rpm), with peak held current 11.297523 A and
ideal sampled angle error below 0.041969°. The ideal initial angle minimum remains
−45.039284°, so table-contact limitations remain.

All 500 logged torque commands match the regenerated table in order; maximum
matching error is 3.98e-8 Nm. The log interval is 1 ms and playback starts at 2 ms.
The measured initial state is body angle −44.890742°, body speed −0.010385447 rad/s,
and relative wheel speed 266.264191 rad/s. Initial speed therefore differs from
the target by only −0.004932 rad/s.

The last retained sample is at 0.359 s: complementary-filter angle 34.554° and
integrated gyro 39.740°. The next sample at 0.360 s crosses the gyro-angle cutoff.
All errors below use the retained prefix and exclude the later opposite-side
impact region, but still include initial contact and release.

| Comparison for run 05 | Body-angle RMSE (°) | Body-rate RMSE (°/s) | Relative wheel-speed RMSE (rad/s) |
|---|---:|---:|---:|
| Analytical target, prescribed initial state | 15.510 | — | — |
| Ideal sampled trajectory, prescribed initial state | 15.507 | — | — |
| Nominal model, measured initial state | 46.183 | 503.407 | 23.170 |
| Shared-fit model, measured initial state | 15.161 | 181.775 | 11.432 |

At 0.300 s, the analytical body angle is −15.537°, the shared model using measured
initial conditions predicts −15.031°, and the measured angle is +7.535°. Measured
relative wheel speed is −20.021 rad/s, versus the analytical −1.900 rad/s.
The gyro rate also departs substantially from the prediction, so disagreement is
not confined to the complementary-filter angle.

The per-run cutoffs differ, so compare improvement over the same 0–0.300 s window:
analytical body-angle RMSE decreases from 12.422° in run 04 to 8.906° in run 05,
approximately 28%. This is a partial improvement, not successful tracking. Correct
command playback, nearly exact initial wheel speed and the small ideal sampling
error do not explain the remaining large discrepancy. The relative contribution
of geometry/contact, torque delivery, friction and sensing is still unresolved.

Stop here for this session. Keep the tested shared-fit initializer, header and
run-05 evaluation for reproducibility; do not present the fitted values as measured
physical parameters. No new fit or trajectory was made after examining run 05.
Before further tuning, a model should account for all three recordings using one
consistent parameter set.

## Verification

The fit was independently evaluated in Python and MATLAB. Both reproduce the
shared parameters and the errors above. In the Python calculation, increasing
RK4 integration from four to eight substeps per 1 ms interval changed body angle
by less than 3e-12 rad at the shared fit, so numerical integration error does not
explain the residuals. The MATLAB script also produces angle, gyro-rate and relative
wheel-speed comparison figures for both recordings.

The run-05 evaluation was also executed in MATLAB and checked independently
in Python using the corresponding fixed command table.
