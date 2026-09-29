# ESP8266 robot with Python PPO control

`main.cpp` integrates the tested firmware's motor pin assignments, PWM functions,
servo geometry, ultrasonic reads, radar sweep and map rendering. Movement decisions
still come from `server.py`, which loads the existing `neural_network.zip` PPO model.
The Python files and saved model have not been changed or retrained.

## Build

Open this folder in PlatformIO and build environment `d1_mini` (`pio run`).
The board matches the configuration found in the existing PROIECT_SM and
cartografiere projects. If the actual board differs, adjust `platformio.ini`.
NewPing and ArduinoJson versions are pinned; Wi-Fi, HTTP, web server and Servo
come from the ESP8266 Arduino framework. The source is deliberately at the project
root, selected by `src_dir` and `build_src_filter`.

## Run

1. Upload the firmware using your usual PlatformIO procedure.
2. Connect the computer to Wi-Fi `Robot_Mapare`, password `12345678`.
3. Ensure the computer's address on that network is `192.168.4.2`. Otherwise edit
   `pythonTargetUrl` and `pythonSensorUrl` in `main.cpp` and rebuild. Allow inbound
   TCP port 5000 for the Python server on this local network.
4. Run `python server.py` from this directory in your existing Python environment
   with Flask, NumPy, Gymnasium, Stable-Baselines3, PyTorch and Matplotlib installed.
   Do not use `main.py` to start the server: that script resumes model training.
5. Open the ESP's control page, normally `http://192.168.4.1/`.
6. With the robot stopped, enter a target and measured times for a 5 cm forward
   movement, a 90 degree left turn, and a 90 degree right turn at PWM 200/255.
   No guessed timings are supplied. The original 100/125 ms steering corrections
   do not establish a 90 degree turn time. Enter the total measured durations.
7. Press **START AI**. **STOP** is available on the same page. Configuration is
   held in RAM and must be entered again after a reboot.

The initial position is `(0, 0)`, with the robot initially facing positive Y.
Targets use the existing normalized range `[-1, 1]`; one 5 cm grid step changes a
coordinate by `2/24`. Positive X is initially right; positive Y is initially forward.
The default target `(0, 0)` must be changed before starting. The robot stops within
half a grid cell on each axis of the requested target, using its estimated position.
This is not an independent detection of leaving the box.

## What controls movement

The ESP first sends `POST /target` with `target_x` and `target_y`, then sends
`POST /sensor` with `d1`, `d2`, `x`, and `y`. The server retains its existing
eight-value observation and action meanings:

| Action | Destination direction in the initial frame |
| --- | --- |
| 0 | Right (+X) |
| 1 | Left (-X) |
| 2 | Up (+Y) |
| 3 | Down (-Y) |

The ESP turns to the requested heading using measured timing, drives one estimated
5 cm step, then stops before its next request. A down command may require turning
around and driving forward; it is not necessarily a reverse-motor command.
There are no autonomous left/right escape routines choosing an alternative route:
that would bypass the current model. Local obstacle checks only reject movement.

The drive functions and pin polarities are from the pasted working firmware.
Turning and forward motion now have bounded durations so a Python action can
finish and return control. The robot stops on invalid readings, malformed actions,
HTTP errors, STOP, and detected Wi-Fi disconnection. A `409` from `/sensor` causes
the target to be resent, allowing recovery after a Python server restart.
HTTP operations run with motors stopped and a 1500 ms timeout. A blocked model
action stops the run; the user can inspect the robot and restart AI.

An interrupted movement invalidates the position estimate. Physically return the
robot to its initial position and heading before pressing the reset-position
button. No encoder or heading sensor was present in the supplied firmware, so
even completed movements accumulate position and heading error.

## Scanning and sensor limitations

The browser provides radar, front/rear distance inspection and a local map while
the robot is stopped. The 0..180 degree map sweep reads both sensors and assumes
they rotate together, facing in opposite directions. The map is a fresh snapshot
in the robot's current frame. It is not a map accumulated along the robot's route.
The green marker is the farthest accepted echo, not a confirmed exit or an
automatically selected neural-network target.

Serial output is disabled because the rear sonar uses GPIO1/GPIO3. Status is shown
on the control page instead. No-echo readings stop AI movement: this can mean a
sensor problem **or** open space beyond the sensor range, so the controller may
stop at an opening. Cheap sensor readings cannot resolve that ambiguity.

Distance inputs preserve the original firmware's 0..100 cm to 0..2 scaling;
the map can measure up to 200 cm. This preserves the interface, not the validity
of the trained model. In `main.py`, the two training distances are identical
expressions derived from X; they do not represent independent front/rear sonars.
Training also lacks walls, collisions, unknown exits and robot heading. Those
existing issues remain, as does the training step size of 0.1 versus firmware's
2/24. Successful hardware integration therefore does not establish that this
saved policy can escape a box.

## Checks

`pio run` checks the real ESP8266 firmware build. These host checks require Python
and `g++`, and use mocked GPIO, clock, sonar and Wi-Fi rather than physical hardware:

```text
python tests/test_hardware_primitives.py
python tests/test_motion.py
```

They check the actual extracted C++ functions: motor polarity, stop outputs,
scan geometry, all 16 heading/action combinations, invalid actions, boundaries,
obstacle refusal, interrupted movement and stopping at the estimated target.
