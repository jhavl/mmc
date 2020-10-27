---
title: Manipulability Motion Control
description: A Purely-Reactive Manipulability-Maximising Motion Controller
---

### _Jesse Haviland and Peter Corke_

**Under Review**

**[Preprint Avaliable Here](https://arxiv.org/abs/2002.11901)**

MMC can be used on any serial-link manipulator regardless of if it is redundant or not. This includes 7 degree-of-freedom robots such as the Fanka-Emika Panda and the 6 degree-of-freedom robots such as the Universal Robotics 5 manipulator.

![Cover Image](/images/cover2_lite.svg)

Resolved-rate motion control of redundant serial-link manipulators is commonly achieved using the Moore-Penrose pseudoinverse in which the norm of the control input is minimized. However, as kinematic singularities are a significant issue for robotic manipulators, we propose a Manipulability Motion Controller which chooses joint velocities which will also increase the manipulability of the robot. The manipulability measure has a complex non-linear relationship with the robot's joint configuration and in this paper we derive the manipulability Jacobian which directly relates joint velocities to the rate of change of  manipulability. Furthermore, we use the manipulability Jacobian within a constrained quadratic program to create an improved resolved-rate motion controller for redundant robots. The resulting real-time controller provides joint velocities which achieve a desired Cartesian end-effector velocity while also maximising the robot's manipulability.

<br>

<iframe width="560" height="315" src="https://www.youtube.com/embed/Vu_rcPlaADI" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

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

We have a created a robotics Python library called [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python) which allows our algorithm to be used an any robot. We use the library [qpsolvers](https://pypi.org/project/qpsolvers/) to solve the optimisation function. However, you can use whichever solver you wish.

Install [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python) from github

```shell
git clone https://github.com/petercorke/robotics-toolbox-python.git
cd robotics-toolbox-python
pip3 install -e .[swift]
```

The following example uses our Swift simulator, see installation instructions at [Swift](https://github.com/jhavl/swift).

### Position-Based Servoing Example

```python
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
import qpsolvers as qp

# Launch the simulator Swift
env = rtb.backend.Swift()
env.launch()

# Create a Panda robot object
panda = rtb.models.Panda()

# Set joint angles to ready configuration
panda.q = panda.qr

# Add the Panda to the simulator
env.add(panda)

# Number of joint in the panda which we are controlling
n = 7

# Set the desired end-effector pose
Tep = panda.fkine() * sm.SE3(0.3, 0.2, 0.3)

arrived = False

while not arrived:

    # The pose of the Panda's end-effector
    Te = panda.fkine()

    # Transform from the end-effector to desired pose
    eTep = Te.inv() * Tep

    # Spatial error
    e = np.sum(np.abs(np.r_[eTep.t, eTep.rpy() * np.pi/180]))

    # Calulate the required end-effector spatial velocity for the robot
    # to approach the goal. Gain is set to 1.0
    v, arrived = rtb.p_servo(Te, Tep, 1.0)

    # Gain term (lambda) for control minimisation
    Y = 0.01

    # Quadratic component of objective function
    Q = np.eye(n + 6)

    # Joint velocity component of Q
    Q[:n, :n] *= Y

    # Slack component of Q
    Q[n:, n:] = (1 / e) * np.eye(6)

    # The equality contraints
    Aeq = np.c_[panda.jacobe(), np.eye(6)]
    beq = v.reshape((6,))

    # The inequality constraints for joint limit avoidance
    Ain = np.zeros((n + 6, n + 6))
    bin = np.zeros(n + 6)

    # The minimum angle (in radians) in which the joint is allowed to approach
    # to its limit
    ps = 0.05

    # The influence angle (in radians) in which the velocity damper
    # becomes active
    pi = 0.9

    # Form the joint limit velocity damper
    Ain[:n, :n], bin[:n] = panda.joint_velocity_damper(ps, pi, n)

    # Linear component of objective function: the manipulability Jacobian
    c = np.r_[-panda.jacobm().reshape((n,)), np.zeros(6)]

    # The lower and upper bounds on the joint velocity and slack variable
    lb = -np.r_[panda.qdlim[:n], 10 * np.ones(6)]
    ub = np.r_[panda.qdlim[:n], 10 * np.ones(6)]

    # Solve for the joint velocities dq
    qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub)

    # Apply the joint velocities to the Panda
    panda.qd[:n] = qd[:n]

    # Step the simulator by 50 ms
    env.step(50)

```

### Acknowledgements

This research was supported by the Australian Centre for Robotic Vision (ACRV) and the Queensland Universilty of Technology Centre for Robotics (QCR).

![thanks](/images/acrvqut.png)
