# MPC via sparse and dense QP formulation
This package is used to solve lateral control of a vehicle using Model Predicitve Control(MPC) using sparse and dense formulation of the cost function in quadratic programming framework using [qp_solver_collection](https://github.com/isri-aist/QpSolverCollection) package.

# Solvers tested
1. OSQP
2. QuadProg
3. QLD
4. jRLQP
5. ProxQP
6. QPmad

# Solver time
The system specifications are as follows -

CPU - 12th Gen Intel® Core™ i7-12700H

RAM - 16 GB

| Formulation | Solver       | Approx. Time in milli sec.        |
|    :---:     | :----------      | :------------       |
|  Sparse         | QLD | 0.036 |
|            | QuadProg | 0.081     |
|           | OSQP | 0.000847 |
|           | jRLQP | 0.037 |
|           | ProxQP | 0.0055 |
|           | QPmad | 0.00467 |
|  Dense        | QLD | 0.00046 |
|            | QuadProg | 0.00075     |
|           | OSQP | 0.00063 |
|           | jRLQP | 0.00083 |
|           | ProxQP | 0.000398 |
|           | QPmad | 0.000041 |

# Dependency
- This package requires data input as a ROStopic. The data consist of the state variables - lateral and yaw error, reference trajectory(given by planner), current velocity and steerking angle. All this data is outputed by mpc_follower package from autoware.ai.
- The solvers are part of unified c++ framework for quadratic programming - [qp_solver_collection](https://github.com/isri-aist/QpSolverCollection).

# Note
- The files "mpc_utils" and ""mpc_trajcetory" and "lowpass_filter" are taken from autoware.ai's mpc_follower package.
- Error based Kinematic bicycle model is used as the state propagation matrix.

# Reference
- Jarrod M. Snider, "Automatic Steering Methods for Autonomous Automobile Path Tracking", Robotics Institute, Carnegie Mellon University, February 2009.