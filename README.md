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

### Instructions
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
* ``Demo1`` : Demo 1 runs the conventional point-based method and produces the following singular results.
```
Warning: Matrix is close to singular or badly scaled. Results may be inaccurate. RCOND =  2.144526e-35. 
> In con_point (line 39)
In demo1 (line 8)
 
Warning: Matrix is close to singular or badly scaled. Results may be inaccurate. RCOND =  2.050568e-50. 
> In con_point (line 94)
In demo1 (line 8)
 
The true value of  TX:
   -0.7040   -0.6935    0.1533   28.7515
    0.6883   -0.7194   -0.0937   35.3894
    0.1753    0.0395    0.9837  -35.4687
         0         0         0    1.0000

The estimated value of  TX:
   1.0e+20 *

   -0.0000   -0.0000    0.0000   -5.2379
    0.0000   -0.0000   -0.0000    0.6839
    0.0000    0.0000    0.0000   -2.3081
         0         0         0    0.0000

The true value of  TY:
   -0.0384   -0.9992   -0.0081 -392.4000
   -0.9993    0.0384    0.0017  138.5000
   -0.0014    0.0082   -1.0000  -29.8000
         0         0         0    1.0000

The estimated value of  TY:
   1.0e+20 *

   -0.0000   -0.0000    0.0000   -0.0000
   -0.0000    0.0000    0.0000   -0.0000
   -0.0000    0.0000   -0.0000   -5.7646
         0         0         0    0.0000
```

* ``Demo2`` : Demo 2 runs the proposed point-based method and produces the following nonsingular results.
```
The true value of original TX:
   -0.7040   -0.6935    0.1533   28.7515
    0.6883   -0.7194   -0.0937   35.3894
    0.1753    0.0395    0.9837  -35.4687
         0         0         0    1.0000

The true value of redefined TX:
   -0.7040   -0.6935    0.1533   33.3200
    0.6883   -0.7194   -0.0937   32.5969
    0.1753    0.0395    0.9837   -6.1537
         0         0         0    1.0000

The estimated value of redefined TX:
   -0.7104   -0.6859    0.1575   31.8659
    0.6816   -0.7263   -0.0888   30.1779
    0.1753    0.0443    0.9835   -5.4886
         0         0         0    1.0000

The true value of original TY:
   -0.0384   -0.9992   -0.0081 -392.4000
   -0.9993    0.0384    0.0017  138.5000
   -0.0014    0.0082   -1.0000  -29.8000
         0         0         0    1.0000

The true value of redefined TY:
   -0.0384   -0.9992   -0.0081 -392.4000
   -0.9993    0.0384    0.0017  138.5000
   -0.0014    0.0082   -1.0000         0
         0         0         0    1.0000

The estimated value of redefined TY:
   -0.0288   -0.9996   -0.0028 -393.7291
   -0.9996    0.0288    0.0012  138.8615
   -0.0011    0.0028   -1.0000         0
         0         0         0    1.0000
```

## Please refer to the submitted article for details.

