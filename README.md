## Nonsingular Hand-Eye and Robot-World Calibration for SCARA-Type Robots

## Reference paper
Nonsingular Hand-Eye and Robot-World Calibration for SCARA-Type Robots: A Comparative Study (2024), submitted to IEEE TII.

## Overview

<p align="center">
   <img src="figure/PIC1.png" width="100%">
</p>

**_Figure_**: Visual representations of hand-eye and robot-world calibration for (a) an articulated robot in eye-in-hand configuration, (b) a SCARA robot in eye-in-hand configuration, (c) an articulated robot in eye-to-base configuration, and (d) an articulated robot in eye-to-base configuration.

Four degree-of-freedom SCARA robots are increasingly used in industry due to their unique advantages in speed and accuracy. When integrated into visual-guided systems with cameras, SCARA robots require hand-eye and robot-world (HE&RW) calibration to establish the system’s geometric relationships. The common calibration methods are pose-based or point-based, which have been widely validated on full degree-of-freedom articulated robots. However, these methods may appear singular and unusable due to SCARA’s restricted movement. 

Inspired by this, we conduct a thorough study on HE&RW calibration for SCARA robots. In the above reference paper, we first analyze the reasons for the SCARA singularity of conventional methods from the perspective of nonlinear least squares. Then, we redefine parameters with clear geometric interpretation and propose two
nonsingular HE&RW calibration methods. This repository contains the implementations of the four methods discussed in our article：
1) ``con_pose`` the conventional pose-based method,
2) ``pro_pose`` the proposed pose-based method,
3) ``con_point`` the conventional point-based method,
4) ``pro_point`` the proposed point-based method.

Besides, we provide two demos for testing.


<p align="center">
   <img src="figure/PIC2.png" width="80%">
</p>

**_Figure_**: In pose-based calibration (left), the camera pose estimation is performed independently before the HE&RW estimation. In point-based calibration (right), the HE&RW estimation is conducted directly without camera pose estimation.



## How to use
### Dependencies
The code runs on Matlab R2023a without any additional dependencies. All necessary auxiliary functions and a simulated SCARA data are provided.


### Main Instructions
To perform the HE&RW calibration, taking the proposed pose-based method as an example, call
```
[RX_out,tX_out,RY_out,tY_out]=pro_pose(RAi,tAi,qij,rj,K)
```
where
* ``RAi`` (3x3xn): the rotation matrice of the robot poses,
* ``tAi`` (3xn): the translation vectors of the robot poses (unit: m),
* ``qij`` (2xnxm): the pattern point pixels,
* ``rj`` (3xm): the pattern point positions (unit: m),
* ``RX_out`` (3x3): the rotation matrix of the hand-eye pose,
* ``tX_out`` (3x1): the translation vector of the hand-eye pose (unit: m),
* ``RY_out`` (3×3)： the rotation matrix of the robot-world pose.
* ``tX_out`` (3×1)： the translation vector of the robot-world pose (unit: m).
Note that ``n`` is the measurement number and ``m`` is the pattern point number.

### Demos
* ``Demo1`` : Demo 1 shows that under SCARA data, conventional methods do not converge iteratively (i.e. singular), while the proposed method can converge to stable values.
* ``Demo2`` : Demo 2 shows the adaptation scenarios of non-singular pose-based and point-based methods.

## Please refer to the submitted article for details.

