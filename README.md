## Manipulability Motion Control
### Jesse Haviland and Peter Corke

_Under Review for RA-L/IROS_

_Preprint: _

MMC is designed for serial-link manipulators which have more degrees-of-freedom than nessecary to access their entire task space. Examples of these redunant robots include the Franka-Emika Panda, Kuka LBR iiwa, Rethink Robotics Sawyer, and the Kinova Gen3.

![Cover Image](/images/cover_lite.svg)


## Abstract
Resolved-rate motion control of redundant serial-link manipulators is commonly achieved using the Moore-Penrose pseudoinverse in which the norm of the control input is minimized. However, as kinematic singularities are a significant issue for robotic manipulators, we propose a Manipulability Motion Controller which chooses joint velocities which will also increase the manipulability of the robot. The manipulability measure has a complex non-linear relationship with the robot's joint configuration and in this paper we derive the manipulability Jacobian which directly relates joint velocities to the rate of change of  manipulability. Furthermore, we use the manipulability Jacobian within a constrained quadratic program to create an improved resolved-rate motion controller for redundant robots. The resulting real-time controller provides joint velocities which achieve a desired Cartesian end-effector velocity while also maximising the robot's manipulability.


![Manipulability](/images/wide_lite.svg)
End-effector angular velocity ellipsoids created using the kinematic Jacobian at two different robot configurations **q**1 and **q**2, on a Panda robot. The ellipsoid depicts how easily the robot's end-effector can move with an arbitrary  angular velocity. The left ellipsoid shows the manipulator's configuration is well conditioned to rotate the end-effector in any direction. While the right configuration is near singular as the end-effector will struggle to rotate around the y or z-axis. This directly corresponds to the manipulability denoted by _m_1, and _m_2.
