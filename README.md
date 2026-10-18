# RCRC Mbed

RCRC Mbed Project for the course Control Systems 2.

## Software Architecture (Core Modules)

### 1. Scope of the visualization

To keep the diagrams understandable, only these files/components are shown:

#### MATLAB

- `matlab/app/GPA_nucleo_UART_exported.m`

#### C++ (firmware)

- `src/main.cpp`
- `lib/GPA/GPA.*`
- `lib/DataLogger/DataLogger.*`
- `lib/IO_handler/IO_handler.*`
- `lib/realtime_thread/realtime_thread.*`
- `lib/LinearCharacteristics/LinearCharacteristics.*`
- `lib/observer/observer.*`

Not shown on purpose:

- Low-level peripherals and drivers (encoder, IMU, ESCON pin-level details)
- Mbed internals (Thread/Ticker internals, HAL)
- UART helper threads
- Eigen

### 2. What the component diagram communicates

The component diagram answers:

- Which module creates the system?
- Which module provides excitation/adaptation support?
- Which module logs data?
- Which tool starts measurements and receives experiment results?
- Which module handles hardware I/O?
- Which module runs the periodic loop?
- Which module performs scaling/calibration?
- Which module performs state estimation?

Intended interpretation:

- `main.cpp` is the entry point, wires the system together, and defines global `DataLogger` and `GPA` instances available to runtime threads.
- `GPA` is used by `realtime_thread` to compute/update excitation.
- `GPA` receives start/configuration commands through the communication path (UART helper threads hidden).
- `DataLogger` is used by `realtime_thread` to get reference values and to collect/export logged signals.
- `DataLogger` receives measurement-start and waveform parameter commands through the communication path (UART helper threads hidden).
- The MATLAB GUI is the host-side operator interface: it opens serial communication, sends experiment setup/start commands, receives logged time-series data, and reconstructs GPA FRD results in MATLAB workspace variables.
- `IO_handler` is the central interface to sensors, actuators, and analog signal handling.
- `realtime_thread` is the periodic execution unit.
- `LinearCharacteristics` is used by `IO_handler` for offset/gain conversion and scaling.
- `observer` is used by `realtime_thread` for observer-based state estimation in the control law.

### 3. MATLAB GUI purpose

The GUI implementation in `matlab/app/GPA_nucleo_UART_exported.m` is the experiment control and data acquisition front end.

Main responsibilities:

- Connect to the Nucleo board over serial (`115200` by default).
- Configure and start time-measurement runs (signal type, amplitude, frequency, offset, downsampling).
- Configure and start GPA identification runs (`f0`, `f1`, `A0`, `A1`, `N`, mode fields).
- Parse incoming framed packets, reconstruct streamed measurement data, and export MATLAB workspace variables.
- Build FRD output from GPA packets and export the identified model to workspace.

Conceptually, the GUI controls experiments and receives results. In the diagrams, GUI arrows to `DataLogger` and `GPA` mean logical interaction via the UART communication path. The UART worker threads carry bytes on the firmware side but are intentionally hidden from these high-level diagrams.

## Mermaid Diagrams

### Component Diagram (flowchart)

```mermaid
flowchart LR
	A[src/main.cpp\nEntry point / system wiring]
	B[realtime_thread\nPeriodic loop execution]
	C[IO_handler\nSensor/actuator interface]
	D[LinearCharacteristics\nScaling and calibration]
	F[DataLogger\nReference generation and logging]
	G[GPA\nExcitation update]
	H[observer\nState estimation]
	X[MATLAB GUI\nExperiment control and acquisition]

	A -->|creates| C
	A -->|creates| B
	A -->|defines global instance| F
	A -->|defines global instance| G

	B -->|calls read/write functions| C
	B -->|uses set value and writes log| F
	B -->|updates excitation| G
	B -->|uses estimated states for control| H

	C -->|uses for gain/offset mapping| D
	X -->|logical setup and start commands via UART path| F
	X -->|logical gpa setup and start via UART path| G
	F -->|logical logged data stream via UART path| X
	G -->|logical gpa result stream via UART path| X
```

### Sequence Diagram

```mermaid
sequenceDiagram
	participant M as main.cpp
	participant RT as realtime_thread
	participant OBS as observer
	participant IO as IO_handler
	participant LC as LinearCharacteristics
	participant DL as DataLogger
	participant GPA as GPA
	participant GUI as MATLAB GUI

	M->>IO: construct()
	IO->>LC: init(...) for analog input/output scaling
	M->>RT: construct(&io, Ts)
	M->>RT: start_loop()

	GUI->>DL: logical time measurement setup and start
	GUI->>GPA: logical gpa setup and start

	loop every Ts
		RT->>DL: get_set_value(time)
		DL-->>RT: w
		RT->>IO: read_ain1(), read_ain2()
		IO->>LC: scale raw sensor values
		IO-->>RT: y1, y2
		RT->>OBS: do_step(u, y2)
		OBS-->>RT: x_hat
		RT->>IO: write_aout(u)
		IO->>LC: map control output to actuator range
		RT->>DL: write_to_log(time, w, y1, y2, u, x_hat1, x_hat2)
		RT->>GPA: update(w, y2)
		GPA-->>RT: excitation for next step
	end

	DL-->>GUI: logical logged packets when run is complete
	GPA-->>GUI: logical gpa points and finished status
	GUI->>GUI: decode packets and export MATLAB workspace variables
```

