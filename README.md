# Framework for the Design and Control of Bimodal Aerial Robots

This work was a dissertation project developed to obtain the MSc in Aerospace Engineering. It consisted of a project to simulate the use of Wheeled Drones for the Inspection of Infrastructures and Surfaces. It uses a framework to easily add and test the use of wheels on different UAV's, and two motion controllers specifically designed for these type of robots. To know more you can access:
<p>
  <a href="https://youtube.com/playlist?list=PLgG5y8BKRgZPmJshbTEFnqItOTOg3Py_f" target="_blank"><img alt="Youtube" src="https://img.shields.io/badge/YouTube-FF0000?style=for-the-badge&logo=youtube&logoColor=white" /></a>
  <a href="https://drive.google.com/file/d/1OVuv63CjqF6x6oheJa9JgF4hPEkpL2wt/view?usp=sharing" target="_blank"><img alt="Abstract" src="https://img.shields.io/badge/-Extended_Abstract-lightgrey?&style=for-the-badge" /></a>
</p>

![Image of Drones](bimodal_robots.jpg?raw=true "Title")

## Configuration Procedures

In order to use this repository you must have the RotorS or CrazyS Simulator set up on you local computer, follow the instructions on: 

> https://github.com/ethz-asl/rotors_simulator
or
> https://github.com/gsilano/CrazyS

After installing the simulator you must clone/download the repository into your local folder and add/merge the folders to the following packages within the RotorS catkin directory:

+ **"urdf"** merge w/ **rotors_description/urdf** --> description files in urdf format for the addition of wheels to any quadrotor
+ **"meshes"** merge w/ **rotors_description/meshes** --> visual files for wheels

Furthermore, if you wish to control the wheled drones with a joystick, you should:

+ **"joy/include"** replace w/ **rotors_joy_interface/include** --> configuration files
+ **"joy/src"** replace w/ **rotors_joy_interface/src** --> corrected code to allow using a joystick to control wheeled UAVs

## wheeled_drones_control

This works as a separate package from the RotorS simulator and can be placed on your catkin folder. It implements a linear and a non-linear approach for the control of the wheeled UAV's and a simple path publisher.

To run the cascaded PID controller:

> roslaunch wheeled_drones_control wheeled_mav_pid.launch mav_name:=<'name of robot'>

Or the DFL controller:

> roslaunch wheeled_drones_control wheeled_mav_dfl.launch mav_name:=<'name of robot'>

Where 'name of robot' should be replaced by the desired wheeled MAV. If the argument "mav_name" is ommited then the programme starts with the default vehicle (Wheelbird = Wheeled Hummingbird). To simply spawn one of the wheeled MAVs, run:

> roslaunch wheeled_drones_control wheeled_mav_control.launch mav_name:=<'name of robot'>


### Waypoint Following

After starting one of the controllers, the robot lifts-off to (0, 0, 1) and stays hovering until a new position is received. To publish a waypoint, one can publish directly to the topic '/desired_position' or run the python script:

> rosrun path_publish.py flight x y z psi
  
> rosrun path_publish.py ground x y
  
> rosrun path_publish.py inclined z gamma psi0

where x y z and psi is the desired pose/position. For the inclined case, 'gamma' is the inclination of the slope and 'psi0' its orientation in the plane.


### Landing and Taking-off

To land in any surface simply run:

> rosrun path_publish.py land

The controller is automatically switched to ground mode if if lands on a flat surface, and it switches to inclined mode when landing on a slope.
![Image of Landing](landing.png?raw=true "Title")

To take-off from any surface:

> rosrun path_publish.py takeoff

## Simulation Gains

To check the gains we refer to ![Resources](https://github.com/MiguelPimentel97/Bimodal_Aerial_Robots/tree/main/wheeled_drones_control/resource)
The simulation gains used for the experiments reported in the videos were the following:

### PID Controller Gains
```
| Mode                      | Gains        |  x   |  y  |  z   |  v  | \varphi | \theta   |  \psi  |
|---------------------------|--------------|------|-----|------|-----|---------|----------|--------|
|                           | Proportional | 1.0  | 1.0 | 4.5  | --  | 56.0    | 56.0     | 7.3    |
|           Flight          | Derivative   | 1.2  | 1.2 | 2.6  | --  | 12.0    | 12.0     | 5.2    |
|                           | Integral     | --   | --  | 0.25 | --  | --      | --       | --     |
|---------------------------|--------------|------|-----|------|-----|---------|----------|--------|
|                           | Proportional | 0.48 | --  | --   | 3.8 | --      | 15.6     | 5.5    |
|           Ground          | Derivative   | --   | --  | --   | --  | --      | 7.3      | 5.2    |
|                           | Integral     | --   | --  | --   | 0.1 | --      | 0.4      | --     |
|---------------------------|--------------|------|-----|------|-----|---------|----------|--------|
|                           | Proportional | --   | --  | 0.55 | 3.5 | --      | 15.6     | 5.5    |
|          Inclined         | Derivative   | --   | --  | --   | 0.1 | --      | 7.3      | 5.2    |
|                           | Integral     | --   | --  | 0.8  | 0.6 | --      | 0.4      | --     |
|---------------------------|--------------|------|-----|------|-----|---------|----------|--------|
```
### DFL Controller Gains
```
| Mode    | Gains | z   | \varphi | \theta | \psi |
|---------|-------|-----|---------|--------|------|
| Flight  | k^0   | 8.5 | 22      | 22     | 15   |
|         | k^1   | 4.4 | 8       | 8      | 7.5  |
|---------|-------|-----|---------|--------|------|
| Surface | k^0   | --  | 19      | 19     | 14.6 |
|         | k^1   | --  | 10      | 10     | 7.5  |
|---------|-------|-----|---------|--------|------|
```
```
| Mode    | Gains        | x    | y    | z   | v   | Acceleration Valve | \theta Valve |
|---------|--------------|------|------|-----|-----|--------------------|--------------|
| Flight  | Proportional | 1.7  | 1.7  | --  | --  | --                 | --           |
|         | Derivative   | 2.05 | 2.05 | --  | --  | --                 | --           |
|---------|--------------|------|------|-----|-----|--------------------|--------------|
| Surface | Flat         | 0.7  | --   | --  | 4.8 | 0.8                | 0.8          |
|         | Inclined     | --   | --   | 1.1 | 5.5 | 1.5                | 0.8          |
|---------|--------------|------|------|-----|-----|--------------------|--------------|
```
