# Dynamics and Kinematics Implementation with CasADi C++

This document outlines the implementation of the robot's kinematics and dynamics using the CasADi C++ interface.

![Robot Model](docs/media/model.png)

## Kinematics

The implementation leverages CasADi's symbolic capabilities to define the transformation matrices and compute the end-effector pose.

The kinematics are implemented in `src/control/vehicle_model.cpp` and exposed through a CasADi function.

![Kinematics Diagram](docs/media/kinematics.png)

## Dynamics

The vehicle's dynamics are formulated using the Fossen method. CasADi is used to symbolically derive the mass matrix, Coriolis and centrifugal terms, and gravity vector. This allows for efficient computation of the inverse dynamics.

The dynamics are implemented in `src/control/vehicle_model.cpp` and exposed as a CasADi function for use in optimization problems.

![Dynamics Diagram](docs/media/dynamics.png)

# References

- Fossen, T. I. (2011). Handbook of marine craft hydrodynamics and motion control. John Wiley & Sons.
- CasADi: A software framework for nonlinear optimization and optimal control. (n.d.). Retrieved from https://web.casadi.org/