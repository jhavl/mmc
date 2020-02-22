## Manipulability Motion Control
### Jesse Haviland and Peter Corke

_Under Review for RA-L/IROS_

_Preprint:_

MMC is designed for serial-link manipulators which have more degrees-of-freedom than nessecary to access their entire task space. Examples of these redunant robots include the Franka-Emika Panda, Kuka LBR iiwa, Rethink Robotics Sawyer, and the Kinova Gen3.

![Cover Image](/images/cover_lite.svg)


## Abstract
Resolved-rate motion control of redundant serial-link manipulators is commonly achieved using the Moore-Penrose pseudoinverse in which the norm of the control input is minimized. However, as kinematic singularities are a significant issue for robotic manipulators, we propose a Manipulability Motion Controller which chooses joint velocities which will also increase the manipulability of the robot. The manipulability measure has a complex non-linear relationship with the robot's joint configuration and in this paper we derive the manipulability Jacobian which directly relates joint velocities to the rate of change of  manipulability. Furthermore, we use the manipulability Jacobian within a constrained quadratic program to create an improved resolved-rate motion controller for redundant robots. The resulting real-time controller provides joint velocities which achieve a desired Cartesian end-effector velocity while also maximising the robot's manipulability.

## What is Manipulability?

The manipulability measure describes how well-conditioned the manipulator is to achieve an arbitrary velocity. It is a scalar which describes the volume of a 6-dimensional ellipsoid created using the kinematic Jacobian. If this ellipsoid has a large volume and is close to spherical, then the manipulator can achieve any arbitrary end-effector velocity easily. However, this 6-dimensional ellipsoid is impossible to display.

The first three rows of the kinematic Jacobian represent the translational component of the end-effector velocity. While the last three rows represent the end-effector angular velocity. Therefore, by using only the first or last three rows of the kinematic Jacobian, the 3-dimensional translation or angular velocity ellipsoids respectively can be found and visualised. For example, see the two translational velocity ellipsoids displayed Below created at two different robot configurations.

The ellipsoid has three radii, along its principle axes. A small radius about an axis represents the robots inability to achieve a velocity in the corresponding direction. At a singularity, the ellipsoid's radius becomes zero about the corresponding axis. Therefore the volume becomes zero. If the manipulator's configuration is well conditioned, these ellipsoids will have a larger volume.

![Manipulability](/images/wide_lite.svg)
End-effector angular velocity ellipsoids created using the kinematic Jacobian at two different robot configurations **q**<sub>1</sub> and **q**<sub>2</sub>, on a Panda robot. The ellipsoid depicts how easily the robot's end-effector can move with an arbitrary  angular velocity. The left ellipsoid shows the manipulator's configuration is well conditioned to rotate the end-effector in any direction. While the right configuration is near singular as the end-effector will struggle to rotate around the y or z-axis. This directly corresponds to the manipulability denoted by _m_<sub>1</sub>, and _m_<sub>2</sub>.
