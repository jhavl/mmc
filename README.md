
Jesse Haviland, Peter Corke

Under Review for RA-L/IROS

Preprint: 

### _Manipulability Motion Control (MMC) is a new and improved approach to Resolved-Rate Motion Control (RRMC)._

MMC is designed for serial-link manipulators which have more degrees-of-freedom than nessecary to access their entire task space. Examples of these redunant robots include the Franka-Emika Panda, Kuka LBR iiwa, Rethink Robotics Sawyer, and the Kinova Gen3.

## Abstract
Resolved-rate motion control of redundant serial-link manipulators is commonly achieved using the Moore-Penrose pseudoinverse in which the norm of the control input is minimized. However, as kinematic singularities are a significant issue for robotic manipulators, we propose a Manipulability Motion Controller which chooses joint velocities which will also increase the manipulability of the robot. The manipulability measure has a complex non-linear relationship with the robot's joint configuration and in this paper we derive the manipulability Jacobian which directly relates joint velocities to the rate of change of  manipulability. Furthermore, we use the manipulability Jacobian within a constrained quadratic program to create an improved resolved-rate motion controller for redundant robots. The resulting real-time controller provides joint velocities which achieve a desired Cartesian end-effector velocity while also maximising the robot's manipulability.
