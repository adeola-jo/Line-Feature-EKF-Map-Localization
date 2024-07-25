
# EKF Map-Based Localization Using Line Features

This repository contains the implementation of the Extended Kalman Filter (EKF) algorithm used to localize a Turtlebot using map-based line features. This work is part of the lab assignments for the Probabilistic Robotics course at the Universitat de Girona, under the Computer Vision and Robotics Research Institute (VICOROB).

## Objective

The aim of this lab is to implement the EKF map-based algorithm to localize a Turtlebot.

## Introduction

The Extended Kalman Filter is used when an object’s motion follows a nonlinear state equation or when the measurements are nonlinear functions of the state. The EKF algorithm linearizes the state and measurement equations using Jacobians, allowing us to propagate the state and state covariance in a roughly linear structure.

## Implementation

The EKF algorithm consists of three main steps: prediction, data association, and update. These steps are recursively performed to estimate the robot's state.

### 1. State Definition

The state of the Turtlebot at time step \( t \) is given by:

$$
x_t = \begin{pmatrix} x_t \\ y_t \\ \theta_t \end{pmatrix}
$$

### 2. Process Model

As the robot moves, its new state is computed by adding the odometry displacement to its previous state, transformed into the world frame.

$$
x_t^W = x_{t-1}^W \oplus (u_t^r + w_t^r)
$$

where \( u_t^r \) is the odometry displacement and \( w_t^r \) is the uncertainty (noise) in the odometry measurement.

### 3. EKF Prediction

The prediction step involves computing the predicted new state and the new covariance.

$$
\hat{x}_{k|k-1} = f(x_{k-1}, u_k, w_k)
$$

$$
P_{k|k-1} = A_k P_{k-1} A_k^T + W_k Q_k W_k^T
$$

where \( A_k \) and \( W_k \) are the Jacobians of the state equation with respect to the state and noise, respectively.

### 4. Data Association

In this step, the algorithm matches the line measurements taken by the robot's sensors with the map of the environment, using the Mahalanobis distance to find the best matches.

### 5. Update

The update step involves calculating the Kalman gain \( K_k \), the new state \( x_k \), and the new covariance \( P_k \).

$$
K_k = P_{k|k-1} H_k^T S_k^{-1}
$$

$$
x_k = \hat{x}_{k|k-1} + K_k v_k
$$

$$
P_k = (I - K_k H_k) P_{k|k-1}
$$

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
- [Moses Chuka Ebere](https://github.com/MosesEbere)
- [Joseph Oloruntoba Adeola](https://github.com/adeola-jo)

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
