Attitude Propagator  
Language: Python  
Author: Alejandro Fernández Ruiz, Space Systems Engineer  

The main goal is to design am attitude propagator in a modular, well-structured way and also with functions as generic as possible to ensure future reusability in control systems.

================================================================================
CODE STRUCTURE
================================================================================

Main_Program.py
    └── The main script where initial propagation parameters are defined
        and the results are obtained directly.

Functions/
    ├── Attitude_Kinematics/
    │       └── Contains the function that computes the time derivative
    │           of the state vector.
    │
    ├── Numerical_Schemes/
    │       └── Includes a general numerical integration scheme (RK4) for
    │           propagation.
    │
    ├── Quaternion_Operators/
    │       └── Provides basic quaternion operations such as multiplication,
    │           conjugation, and inversion.
    │
    ├── Rotations/
    │       └── Contains functions for principal rotations around the x, y,
    │           and z axes, as well as conversions between rotation
    │           representations (DCM, Euler angles, and quaternions).
    │
    └── Solver/
            └── The main solver that propagates the satellite’s attitude
                over time and returns the state vector at each time step.

================================================================================
REPORT
Download link: https://drive.google.com/file/d/1pgnSpvf58rgNjosjuE8FOEkr_l2uyP61/view?usp=drive_link
================================================================================

================================================================================
CONTACT
================================================================================
For any questions or suggestions, please, contact the author:  

alejandrofr.spaceng@gmail.com
