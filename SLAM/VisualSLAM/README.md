# Visual SLAM Module

This module implements Visual SLAM (Simultaneous Localization and Mapping) supporting monocular, stereo, and RGB-D camera inputs. It combines visual odometry and mapping in real-time to track camera motion and build a 3D map of the environment.

## Features

- Support for multiple camera types:
  - Monocular camera
  - Stereo camera
  - RGB-D camera
- Feature detection and tracking using optical flow
- Visual odometry using essential matrix or homography decomposition
- 3D mapping using triangulation
- Real-time trajectory visualization
- Support for keyframe selection
- 3D point cloud visualization

## Requirements

Install the required dependencies:

```bash
pip install -r requirements.txt
```

## Usage

Basic usage example:

```python
from visual_slam import CameraParams, VisualSLAM

# Initialize camera parameters
camera_params = CameraParams(
    fx=525.0,  # focal length x
    fy=525.0,  # focal length y
    cx=320.0,  # principal point x
    cy=240.0,  # principal point y
    baseline=0.075  # for stereo setup
)

# Create SLAM instance
slam = VisualSLAM(camera_params)

# Process frames
pose, map_points = slam.process_frame(frame)

# Visualize results
slam.visualize()
```

## Demo

Run the included demo script to test Visual SLAM with your camera:

```bash
python visual_slam.py
```

## Components

1. **FeatureTracker**: Handles feature detection and tracking between frames
2. **VisualOdometry**: Estimates camera motion using epipolar geometry
3. **Mapping**: Performs 3D reconstruction using triangulation
4. **VisualSLAM**: Main class that integrates all components

## Notes

- Adjust camera parameters according to your specific camera setup
- For best results, ensure good lighting conditions and textured scenes
- The system performs better with slower, smooth camera motion