---
title: Manipulability Motion Control
description: A Purely-Reactive Manipulability-Maximising Motion Controller
---

### _Jesse Haviland and Peter Corke_

**Under Review**

**[Preprint Avaliable Here](https://arxiv.org/abs/2002.11901)**

MMC is designed for serial-link manipulators which have more degrees-of-freedom than nessecary to access their entire task space. Examples of these redunant robots include the Franka-Emika Panda, Kuka LBR iiwa, Rethink Robotics Sawyer, and the Kinova Gen3.

![Cover Image](/images/cover_lite.svg)

Resolved-rate motion control of redundant serial-link manipulators is commonly achieved using the Moore-Penrose pseudoinverse in which the norm of the control input is minimized. However, as kinematic singularities are a significant issue for robotic manipulators, we propose a Manipulability Motion Controller which chooses joint velocities which will also increase the manipulability of the robot. The manipulability measure has a complex non-linear relationship with the robot's joint configuration and in this paper we derive the manipulability Jacobian which directly relates joint velocities to the rate of change of  manipulability. Furthermore, we use the manipulability Jacobian within a constrained quadratic program to create an improved resolved-rate motion controller for redundant robots. The resulting real-time controller provides joint velocities which achieve a desired Cartesian end-effector velocity while also maximising the robot's manipulability.

<br>

<iframe width="560" height="315" src="https://www.youtube.com/embed/zBGLPoPNZ10" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

<br>

* * *

## What is Manipulability?

The manipulability measure describes how well-conditioned the manipulator is to achieve an arbitrary velocity. It is a scalar which describes the volume of a 6-dimensional ellipsoid created using the kinematic Jacobian. If this ellipsoid has a large volume and is close to spherical, then the manipulator can achieve any arbitrary end-effector velocity easily. However, this 6-dimensional ellipsoid is impossible to display.

The first three rows of the kinematic Jacobian represent the translational component of the end-effector velocity. While the last three rows represent the end-effector angular velocity. Therefore, by using only the first or last three rows of the kinematic Jacobian, the 3-dimensional translation or angular velocity ellipsoids respectively can be found and visualised. For example, see the two translational velocity ellipsoids displayed Below created at two different robot configurations.

The ellipsoid has three radii, along its principle axes. A small radius about an axis represents the robots inability to achieve a velocity in the corresponding direction. At a singularity, the ellipsoid's radius becomes zero about the corresponding axis. Therefore the volume becomes zero. If the manipulator's configuration is well conditioned, these ellipsoids will have a larger volume.

![Manipulability](/images/wide_lite.svg)
> End-effector angular velocity ellipsoids created using the kinematic Jacobian at two different robot configurations **q**<sub>1</sub> and **q**<sub>2</sub>, on a Panda robot. The ellipsoid depicts how easily the robot's end-effector can move with an arbitrary  angular velocity. The left ellipsoid shows the manipulator's configuration is well conditioned to rotate the end-effector in any direction. While the right configuration is near singular as the end-effector will struggle to rotate around the y or z-axis. This directly corresponds to the manipulability denoted by _m_<sub>1</sub>, and _m_<sub>2</sub>.

* * *

## How do I use it?

We have a created a robotics Python library called [ropy](https://github.com/jhavl/ropy) which allows our algorithm to be used an any robot. See [ropy](https://github.com/jhavl/ropy) for installation instructions. We use the library [qpsolvers](https://pypi.org/project/qpsolvers/) to solve the optimisation function. However, you can use whichever solver you wish.

### Basic Example
```python
import ropy as rp
import numpy as np
import qpsolvers as qp

# Initialise a Franka-Emika Panda Robot
panda = rp.Panda()

# The current joint angles of the Panda
panda.q = np.array([0, -3, 0, -2.3, 0, 2, 0])

# The desired end-effecor spatial velocity
v = np.array([0.05, 0.05, 0, 0, 0, 0.05])

# Form the equality constraints
# The kinematic Jacobian in the end-effecor frame
Aeq = panda.Je
beq = v

# Gain term (lambda) for control minimisation
Y = 0.005

# Quadratic component of objective function
Q = Y * np.eye(7)

# Linear component of objective function: the manipulability Jacobian
c = -panda.Jm.reshape((7,))

# Solve for the joint velocities dq
dq = qp.solve_qp(Q, c, None, None, Aeq, beq)
```

### Position-Based Servoing Example
```python
import ropy as rp
import numpy as np
import qpsolvers as qp


# Initialise a Franka-Emika Panda Robot
panda = rp.Panda()

# The current joint angles of the Panda
# You need to obtain these from however you interfave with your robot
# eg. ROS messages, PyRep etc.
panda.q = np.array([0, -3, 0, -2.3, 0, 2, 0])

# The current pose of the robot
wTe = panda.T

# The desired pose of the robot
# = Current pose offset 20cm in the x-axis
wTep = np.copy(wTe)
wTep[0,3] += 0.2

# Gain term (lambda) for control minimisation
Y = 0.005

# Quadratic component of objective function
Q = Y * np.eye(7)

arrived = False
while not arrived:

    # The current joint angles of the Panda
    # You need to obtain these from however you interfave with your robot
    # eg. ROS messages, PyRep etc.
    panda.q = np.array([0, -3, 0, -2.3, 0, 2, 0])

    # The desired end-effecor spatial velocity
    v, arrived = rp.p_servo(wTe, wTep)

    # Form the equality constraints
    # The kinematic Jacobian in the end-effecor frame
    Aeq = panda.Je
    beq = v.reshape((6,))

    # Linear component of objective function: the manipulability Jacobian
    c = -panda.Jm.reshape((7,))

    # Solve for the joint velocities dq
    dq = qp.solve_qp(Q, c, None, None, Aeq, beq)

    # Send the joint velocities to the robot
    # eg. ROS messages, PyRep etc.
```

### Acknowledgements

This research was supported by the Australian Centre for Robotic Vision (ACRV) and the Queensland Universilty of Technology Centre for Robotics (QCR).

![thanks](/images/acrvqut.png)
