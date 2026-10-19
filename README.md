# Rotary Pendulum

Rotary Pendulum Mbed Project for the course Control Systems 2.

## Changes

- Local balancing at 1 kHz with the existing 20 kHz PWM/current loop. PlatformIO/Mbed configuration targets NUCLEO-F446RE; Simulink packet exchange and the communication watchdog are removed.
- B1: `INIT` -> `ARMED` (LED blinks) -> automatic upright capture -> `BALANCE` (LED solid). A button press cancels/disarms; leaving the outer band returns to `INIT` and requires rearming. No automatic swing-up/down.
- Capture/disable bands: 0.174533 / 0.523599 rad from upright. Removed the double encoder negation; velocity filtering and wrapped upright feedback follow Simulink. Current saturation remains +/-0.5 A followed by friction compensation, allowing +/-0.5205 A commanded current.
- Shared MATLAB app UART logging, project-specific telemetry, `m_` member names, `PID_Cntrl`, and only the used second-order additions to `IIR_Filter`. Removed the obsolete serial libraries and committed firmware binary.

## Attention before and during hardware testing

- Reset with the pendulum downward and the arm at its intended zero. Rearming does not reset encoders. Start with GUI amplitude and offset both zero; references/angles are rad and velocities rad/s.
- Confirm encoder directions, current-sensor zero/polarity, drive direction and the configured 24 V supply. Arm travel limits and PB8 driver-fault handling are not implemented.
- Verify button/LED, capture/disarm and reset behaviour. Check 1 kHz / 20 kHz timing on PB4 / PB5 while logging/transmitting. UART disconnection does not disarm the controller.
- App `Out` / `In1` / `In2`: current [A] / continuous arm angle [rad] / continuous pendulum angle [rad, zero downward]. Six logged columns: commanded current, measured current, arm angle, wrapped upright pendulum error, arm velocity, pendulum velocity.
- GUI references apply only during logging and return to zero afterward. Keep references within permitted arm travel. The unchanged app still has generic labels/defaults; retained SLDRT UART models are references only.
- [MATLAB initializer](matlab/inv_rot_pen_ini.m) requires Control System Toolbox. After controller changes, copy printed `K`/`V` into C++, keep sampling/filter settings consistent, then rebuild/upload; MATLAB does not configure firmware automatically.
- GPA excitation/update is commented out. Finalize its measurement path before use; future outer-loop GPA frequencies must stay below 500 Hz. Hardware balancing and timing remain unverified by software checks.
