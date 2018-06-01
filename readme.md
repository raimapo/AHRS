# Project Title

Attitude and Heading Reference System using MATLAB as simple as possible

## Getting Started

Few implementations of Attitude and Heading Reference System using Matlab in mind to keep it as simple as possible to understand for beginners.
Basis of Matlab scripts was token from x-IO examples. Additionaly scripts from Phil Kim books also was used. Thank You for the Authors !

You can help to fix this examples and add new one with new method or new feature !

What is here implemented:
1. Madwick AHRS algorithm           - Quaternions  	- x-IO
2. Mahony AHRS algorithm            - Quaternions  	- x-IO
3. Gyroscope data integration 		- Euler angles 	- Phil Kim - modifed rotation sequence to compare results
4. Accelerometer data integration 	- Euler angles 	- Phil Kim - modifed g to 1, rotation sequence and added Yaw/Heading/psi calculation. Yaw data calculations is not used because it's needed corections. 
5. Linear Kalman filter (KF) 		- Quaternions 	- Phil Kim - with modifications. Learn to use Q and R matrices. Check with Kalaman gain equal to 0 (H=0*eye(4))
6. Extended Kalman filter (EKF)     - Euler angles  - Phil Kim - Learn to make Jacobian and how infuenced Q and R matrices.
7. Unscented Kalman filter (UKF)    - Euler angles  - Phil Kim
8. Extended Kalman filter           - Quternions    - PX4 autopilot
9. Unscented Kalman filter          - Quaternion    - Jose Gama - RAHRS libraty for R           
10. TRIAD                           - Euler angles  - Jose Gama - RAHRS libraty for R        


### Prerequisites

Matlab

### Installing

Copy AHRS folder anywhere You want

## Running the tests

Start with ExampleScript.m

## Acknowledgments

* [x-IO](http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/) - Madwick and Mahony algorithms impementation, GNU GPL licensing.
* [Rigid Body Dynamics For Beginners: Euler angles & Quaternions by Phil Kim](https://www.amazon.com/Rigid-Body-Dynamics-Beginners-Quaternions/dp/1493598201/ref=pd_bxgy_14_img_2?_encoding=UTF8&pd_rd_i=1493598201&pd_rd_r=92KWPRT3Z44FNK23HVHT&pd_rd_w=bvcMo&pd_rd_wg=dbg68&psc=1&refRID=92KWPRT3Z44FNK23HVHT)
* [Kalman Filter for Beginners: with MATLAB Examples by Phil Kim](https://www.amazon.com/Kalman-Filter-Beginners-MATLAB-Examples/dp/1463648359/ref=pd_lpo_sbs_14_img_1/133-5670404-7424740?_encoding=UTF8&psc=1&refRID=NJK9K8J8BGJXA8147E8P)
* [RAHRS: Data Fusion Filters for Attitude Heading Reference System (AHRS) with Several Variants of the Kalman Filter and the Mahoney and Madgwick Filters](https://cran.r-project.org/web/packages/RAHRS/index.html)
