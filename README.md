# RBE501dVRKModeling

This project focuses on developing a kinematic model of the Patient Side Manipulator (PSM), a component of the da Vinci Research Kit (dVRK), using the Product of Exponentials (PoE) formulation from modern robotics.

The PoE method offers a compact and mathematically robust way to represent the forward kinematics of serial manipulators. Unlike traditional Denavit-Hartenberg (DH) parameters, PoE models the manipulator as a product of exponentials of twist coordinates, providing more flexibility and clarity in handling complex configurations and multiple coordinate frames.

In this project:

    Each joint of the PSM is defined by a screw axis in the space frame.

    The forward kinematics are computed using the matrix exponential of each screw, multiplied in sequence with the manipulator’s home configuration.

    The resulting transformation maps joint angles to end-effector poses in 3D space.

This modeling approach supports high-precision control and provides a strong foundation for tasks like calibration, motion planning, and real-time teleoperation. It also aligns with modern robotics theory, as taught in advanced robotics courses and used in professional applications.
