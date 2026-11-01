# Echo Navigation Robot

A mobile robot designed to move toward a selected target using a learned navigation policy. Ultrasonic sensors provide distance measurements, an ESP8266 controls the robot's movements, and a Python server runs the AI model. A browser interface lets the user set a target, start or stop navigation, and inspect the surrounding space.

## How navigation works

1. The user selects a target position and starts navigation.
2. The robot measures the distances in front of and behind it, then sends those readings and its estimated position to the Python server over Wi-Fi.
3. The server combines this information with the target coordinates and the remaining displacement to the target. The model chooses one of four directions: right, left, up, or down in the initial coordinate frame.
4. The firmware turns the robot toward the selected direction and moves it forward by a short, timed step, estimated at 5 cm.
5. The robot stops, updates its estimated position, and repeats the cycle until it reaches the target tolerance or navigation is interrupted.

The Python model chooses the direction; the firmware handles the physical movement and local obstacle checks. If a movement is blocked, the robot stops the current run rather than selecting an alternative route on its own. It also stops on invalid sensor readings, communication errors, or a user STOP command.

## How the model learns

The navigation policy is trained with **PPO (Proximal Policy Optimization)** in a custom simulated environment. It receives eight values: two distance inputs, the robot's X and Y coordinates, the target's X and Y coordinates, and the displacement to the target along each axis.

During training, actions change the simulated position. Moving closer to the target produces a positive progress reward, moving farther away produces a negative one, and reaching the target gives an additional reward. The aim is to learn which direction brings the robot closer to its destination.

The trained policy is saved and loaded by the Python server for movement decisions. The robot uses that policy during navigation; it does not train the model while driving.

## Scanning the surroundings

While stationary, the robot can rotate its ultrasonic sensors to inspect nearby objects and build a local map. The browser displays measured obstacles, estimated free space, and the robot's location within that scan. A green marker highlights the farthest accepted measurement.

This map is a snapshot around the robot. It is displayed for inspection and is not passed to the navigation model or used to choose the target automatically.

## Expected behavior and current scope

The intended behavior is gradual movement toward a user-defined target, with a new model decision after each completed step. Position and heading are estimated from timed movements, so physical errors can accumulate.

The current training environment simplifies movement and does not model walls, collisions, or robot heading; its distance inputs are derived from position rather than simulated independent sonars. Navigation around real obstacles and discovery of an unknown exit therefore remain goals for further development.
