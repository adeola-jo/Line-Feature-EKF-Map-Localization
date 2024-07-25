# EKF Map-Based Localization Using Line Features

This repository contains the implementation of the Extended Kalman Filter (EKF) algorithm used to localize a Turtlebot using map-based line features. This work is part of the lab assignments for the Probabilistic Robotics course at the Universitat de Girona, under the Computer Vision and Robotics Research Institute (VICOROB).

## Objective

The aim of this lab is to implement the EKF map-based algorithm to localize a Turtlebot.

## Introduction

The Extended Kalman Filter is used when an object's motion follows a nonlinear state equation or when the measurements are nonlinear functions of the state. The EKF algorithm linearizes the state and measurement equations using Jacobians, allowing us to propagate the state and state covariance in a roughly linear structure.

## Implementation

The EKF algorithm consists of three main steps: prediction, data association, and update. These steps are recursively performed to estimate the robot's state.

### 1. State Definition

The state of the Turtlebot at time step t is given by:

x_t = [x_t; y_t; θ_t]

### 2. Process Model

As the robot moves, its new state is computed by adding the odometry displacement to its previous state, transformed into the world frame.

![equation](https://latex.codecogs.com/gif.latex?x_t%5EW%20%3D%20x_%7Bt-1%7D%5EW%20%5Coplus%20%28u_t%5Er%20&plus;%20w_t%5Er%29)

where u_t^r is the odometry displacement and w_t^r is the uncertainty (noise) in the odometry measurement.

### 3. EKF Prediction

The prediction step involves computing the predicted new state and the new covariance.

![equation](https://latex.codecogs.com/gif.latex?%5Chat%7Bx%7D_%7Bk%7Ck-1%7D%20%3D%20f%28x_%7Bk-1%7D%2C%20u_k%2C%20w_k%29)

![equation](https://latex.codecogs.com/gif.latex?P_%7Bk%7Ck-1%7D%20%3D%20A_k%20P_%7Bk-1%7D%20A_k%5ET%20&plus;%20W_k%20Q_k%20W_k%5ET)

where A_k and W_k are the Jacobians of the state equation with respect to the state and noise, respectively.

### 4. Data Association

In this step, the algorithm matches the line measurements taken by the robot's sensors with the map of the environment, using the Mahalanobis distance to find the best matches.

### 5. Update

The update step involves calculating the Kalman gain K_k, the new state x_k, and the new covariance P_k.

![equation](https://latex.codecogs.com/gif.latex?K_k%20%3D%20P_%7Bk%7Ck-1%7D%20H_k%5ET%20S_k%5E%7B-1%7D)

![equation](https://latex.codecogs.com/gif.latex?x_k%20%3D%20%5Chat%7Bx%7D_%7Bk%7Ck-1%7D%20&plus;%20K_k%20v_k)

![equation](https://latex.codecogs.com/gif.latex?P_k%20%3D%20%28I%20-%20K_k%20H_k%29%20P_%7Bk%7Ck-1%7D)

## Files

- `ekf_localization.py`: Contains the implementation of the EKF algorithm.
- `EKF_Moses_Joseph-4.pdf`: The lab report detailing the implementation and discussion of the EKF algorithm.

## Disclaimer

**This code depends on certain modules developed by the University of Girona and cannot be made public without permission.**

The user cannot run the code. This repository is intended to host the results and demonstrate the implementation of the EKF algorithm. Only the parts of the project developed by the authors are made public.

## Results

Here is a GIF showcasing the localization of the Turtlebot using the EKF algorithm:

![EKF Localization Result](result.gif)

## Discussion

The major challenge encountered during the implementation was deriving the measurement model, which required a thorough understanding of the problem to correctly derive the measurement equations.

---

**Authors:**
- [Moses Chuka Ebere](https://github.com/username1)
- [Joseph Oloruntoba Adeola](https://github.com/username2)

---

## References

- [Probabilistic Robotics by Sebastian Thrun, Wolfram Burgard, and Dieter Fox](https://www.probabilistic-robotics.org/)
- Lecture slides and notes from Universitat de Girona

For more details, refer to the lab report `report.pdf`.

---

### Contact

For any inquiries, please contact:

- Moses Chuka Ebere: moses.ebere@example.com
- Joseph Oloruntoba Adeola: joseph.adeola@example.com

---

**License:**

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.