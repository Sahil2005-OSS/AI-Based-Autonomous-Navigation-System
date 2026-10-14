# Data Flow Pipeline

## 1. End-to-End Pipeline
This document describes the flow of data through the autonomous navigation pipeline, from raw sensor acquisition to actuator command dispatch.

```
+--------------------+
| Raw Sensor Inputs  |  --> [RGB Frames, Point Clouds, IMU, GPS]
+--------------------+
          │
          ▼
+--------------------+
| Preprocessing      |  --> [Normalization, Downsampling, Filtering, Calibration]
+--------------------+
          │
          ▼
+--------------------+
| Feature Extraction |  --> [Object Bounding Boxes, Free Space Grids, Depth Maps]
+--------------------+
          │
          ▼
+--------------------+
| State Fusion       |  --> [Fused Environmental Costmap + Ego Pose Estimation]
+--------------------+
          │
          ▼
+--------------------+
| Trajectory Planner |  --> [Waypoints, Target Velocity, Heading Angles]
+--------------------+
          │
          ▼
+--------------------+
| Controller         |  --> [Steering Angle, Acceleration/Brake Signals]
+--------------------+
```

## 2. Data Interfaces & Message Contracts
- **Input Feeds:** Sensor message frequencies and tensor shapes.
- **Intermediate Payloads:** Coordinate frames (world frame vs. body frame) and transformation conventions.
- **Output Payloads:** Actuator targets and control limits.
