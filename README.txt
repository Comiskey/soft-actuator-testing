The Trajectory Cycle program serves as a means of running a cyclic actuator inflation test. 
This program is designed for the arduino nano platform and is intended to be used with a
2/2 solenoid valve. The program is designed to be used with any soft pneumatic actuator,
and requires manually tuning of PID gain values for different actuators.

The main control loop can be found in main.cpp, with supporting functions and variabls found in
all other files. If building your own test stand to run this program, be sure to adjust the
pinout values in the various supporting cpp files.

--Evan Comiskey, 2024/04, MIT Fabrication-Integrated Design Lab, comiskey@mit.edu
