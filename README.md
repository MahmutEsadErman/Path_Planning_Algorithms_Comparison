# GPS-Denied Navigation with Monocular Camera

![Python](https://img.shields.io/badge/Python-3.x-blue.svg)
![OpenCV](https://img.shields.io/badge/OpenCV-Computer%20Vision-green.svg)
![Status](https://img.shields.io/badge/Status-Active-success.svg)

## Overview

In environments where GPS signals are jammed, unreliable, or unavailable (such as indoors, urban canyons, or forests), relying on satellite positioning is impossible. This system solves that problem by analyzing the motion of features between consecutive image frames to calculate the relative motion of the camera, enabling autonomous navigation without external signals.

## Key Features

* **Feature Detection & Tracking:** Utilizes robust algorithms (e.g., SIFT, SURF, or ORB) to identify landmarks in the environment.
* **Real-time Trajectory Plotting:** Visualizes the estimated path of the vehicle in real-time.
* **GPS-Denied Capability:** Functions entirely offline without need for GNSS/GPS connection.
* **Lightweight:** Designed to run on embedded systems (potential for Raspberry Pi/Jetson deployment).

## Prerequisites

To run this project, you can use docker.

## Installation

1.  **Clone the repository:**
    ```bash
    git clone [https://github.com/MahmutEsadErman/GPS-Denied-Navigation-with-Monocular-Camera.git](https://github.com/MahmutEsadErman/GPS-Denied-Navigation-with-Monocular-Camera.git)
    cd GPS-Denied-Navigation-with-Monocular-Camera
    ```

3.  **Controls:**
    * Press `q` or `ESC` to exit the visualization window.
    * The system will display the current frame with tracked features and a separate window showing the 2D trajectory.

## How It Works

The system follows a standard Monocular Visual Odometry pipeline:

1.  **Image Capture:** Frames are grabbed from the camera stream.
2.  **Feature Extraction:** The system detects keypoints in the current frame $I_k$ (e.g., corners, edges).
3.  **Feature Tracking:** It matches these keypoints to the previous frame $I_{k-1}$ using Optical Flow (Lucas-Kanade) or Feature Matching.
4.  **Pose Estimation:**
    * The **Essential Matrix ($E$)** is computed from the matched point pairs.
    * $E$ is decomposed into Rotation ($R$) and Translation ($t$) matrices using Singular Value Decomposition (SVD).
