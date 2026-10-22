# Rotary Pendulum

Rotary Pendulum Mbed Project for the course Control Systems 2.

## Changes

- One 10 kHz realtime thread handles acquisition, notch/velocity filtering, balancing and PI current control; hardware PWM remains at 20 kHz. PlatformIO/Mbed configuration targets NUCLEO-F446RE; Simulink packet exchange and the communication watchdog are removed.
- B1: `INIT` -> `ARMED` (LED blinks) -> automatic upright capture -> `BALANCE` (LED solid). A button press cancels/disarms; leaving the outer band returns to `INIT` and requires rearming. No automatic swing-up/down.
- Capture/disable bands: 0.174533 / 0.523599 rad from upright. Removed the double encoder negation; velocity filtering and wrapped upright feedback follow Simulink. Current saturation remains +/-0.5 A followed by friction compensation, allowing +/-0.5205 A commanded current.
- Shared MATLAB app UART logging, project-specific telemetry, `m_` member names, `PID_Cntrl`, and only the used second-order additions to `IIR_Filter`. Removed the obsolete serial libraries and committed firmware binary.

- UART/GPA aligned with RCRC and mini_cuboid: validated packet lengths/checksums, fixed GPA storage for up to 100 points, and seven-float complex results. MATLAB computes `Gur`, `Gyr`, and `Gyu = Gyr/Gur`; unused motor/type controls and the startup image are removed.
- Shared GUI/export and project-independent `cut_data.m`; telemetry remains unaveraged current/theta1/theta2. GUI defaults: zero reference amplitude, logging downsampling 20 (approximately 2 sec at 500 Hz), and GPA upper frequency 4990 Hz for the 10 kHz control loop.

- Removed the separate fast thread, its mutex/snapshots, PB5 timing output, and current-setpoint low-pass filter. Encoder notches and velocity filters run continuously; the PI resets whenever balancing is disabled.
- Current-controller roll-off is optional: uncomment `ENABLE_CURRENT_ROLLOFF` in `realtime_thread.cpp` to enable the existing 3 kHz filter. It is disabled by default; `PID_Cntrl` bypasses it explicitly when its time constant is zero. MATLAB models both variants.

## Attention before and during hardware testing

- Reset with the pendulum downward and the arm at its intended zero. Rearming does not reset encoders. Start with GUI amplitude and offset both zero; references/angles are rad and velocities rad/s.
- With roll-off and setpoint smoothing disabled, verify current response/ripple and loop timing before balancing. The MATLAB R/L check omits back-EMF, PWM ripple, deadzone, limits and computation delay.
- Confirm encoder directions, current-sensor zero/polarity, drive direction and the configured 24 V supply. Arm travel limits and PB8 driver-fault handling are not implemented.
- Verify button/LED, capture/disarm and reset behaviour. Measure the 100 us control period and execution time on PB4 while logging/transmitting; thread flags do not count missed ticks. UART disconnection does not disarm the controller.
- App `Out` / `In1` / `In2`: current [A] / continuous arm angle [rad] / continuous pendulum angle [rad, zero downward]. Six logged columns: commanded current, measured current, arm angle, wrapped upright pendulum error, arm velocity, pendulum velocity.
- GUI references apply only during logging and return to zero afterward. Keep references within permitted arm travel. The app keeps generic Out/In1/In2 labels; retained SLDRT UART models are references only.
- [MATLAB initializer](matlab/inv_rot_pen_ini.m) requires Control System Toolbox. After controller changes, copy printed `K`/`V` into C++, keep sampling/filter settings consistent, then rebuild/upload; MATLAB does not configure firmware automatically.
- GPA excitation/update is commented out. Finalize its measurement path before use; future outer-loop GPA frequencies must stay below 5000 Hz. Hardware balancing and timing remain unverified by software checks.
