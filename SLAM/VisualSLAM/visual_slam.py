"""
Visual SLAM Implementation
This module implements Visual SLAM (Simultaneous Localization and Mapping)
supporting monocular, stereo, and RGB-D camera inputs.
"""

import numpy as np
import cv2
from dataclasses import dataclass
from typing import List, Tuple, Optional
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


@dataclass
class CameraParams:
    """Camera parameters for calibration"""
    fx: float  # Focal length x
    fy: float  # Focal length y
    cx: float  # Principal point x
    cy: float  # Principal point y
    baseline: float = 0.0  # Baseline for stereo setup


class FeatureTracker:
    """Track features across frames using optical flow"""
    
    def __init__(self, max_corners: int = 1000, 
                 quality_level: float = 0.01,
                 min_distance: float = 7):
        self.max_corners = max_corners
        self.quality_level = quality_level
        self.min_distance = min_distance
        self.feature_params = dict(
            maxCorners=max_corners,
            qualityLevel=quality_level,
            minDistance=min_distance,
            blockSize=7
        )
        self.lk_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )

    def detect_features(self, frame: np.ndarray) -> np.ndarray:
        """Detect features in the frame"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
        features = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
        return features

    def track_features(self, prev_frame: np.ndarray, curr_frame: np.ndarray, 
                      prev_features: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Track features between consecutive frames"""
        if prev_features is None or len(prev_features) < 1:
            return None, None
            
        prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY) if len(prev_frame.shape) == 3 else prev_frame
        curr_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY) if len(curr_frame.shape) == 3 else curr_frame
        
        curr_features, status, _ = cv2.calcOpticalFlowPyrLK(
            prev_gray, curr_gray, prev_features, None, **self.lk_params
        )
        
        # Filter out points where tracking failed
        good_curr = curr_features[status == 1]
        good_prev = prev_features[status == 1]
        
        return good_prev, good_curr


class VisualOdometry:
    """Estimate camera motion between frames"""
    
    def __init__(self, camera_params: CameraParams):
        self.camera_params = camera_params
        self.K = np.array([
            [camera_params.fx, 0, camera_params.cx],
            [0, camera_params.fy, camera_params.cy],
            [0, 0, 1]
        ])
        self.pose = np.eye(4)  # Current camera pose
        self.trajectory = [self.pose[:3, 3]]  # Camera trajectory

    def estimate_motion(self, prev_points: np.ndarray, curr_points: np.ndarray,
                       method: str = "essential") -> np.ndarray:
        """Estimate camera motion between frames using epipolar geometry"""
        if method == "essential":
            E, mask = cv2.findEssentialMat(
                curr_points, prev_points, self.K,
                method=cv2.RANSAC, prob=0.999, threshold=1.0
            )
            _, R, t, _ = cv2.recoverPose(E, curr_points, prev_points, self.K, mask)
        else:  # Use homography for planar scenes
            H, mask = cv2.findHomography(
                prev_points, curr_points, cv2.RANSAC, 5.0
            )
            # Decompose homography to get rotation and translation
            _, Rs, ts, _ = cv2.decomposeHomographyMat(H, self.K)
            # Use the first solution (usually the most probable one)
            R, t = Rs[0], ts[0]

        # Update pose
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t.ravel()
        self.pose = self.pose @ T
        self.trajectory.append(self.pose[:3, 3])
        
        return self.pose


class Mapping:
    """3D mapping using triangulation"""
    
    def __init__(self, camera_params: CameraParams):
        self.camera_params = camera_params
        self.points_3d = []
        self.colors = []

    def triangulate_points(self, points1: np.ndarray, points2: np.ndarray,
                         pose1: np.ndarray, pose2: np.ndarray) -> np.ndarray:
        """Triangulate 3D points from corresponding image points and camera poses"""
        P1 = self.camera_params.K @ pose1[:3]  # 3x4 projection matrix for first camera
        P2 = self.camera_params.K @ pose2[:3]  # 3x4 projection matrix for second camera
        
        # Triangulate points
        points_4d = cv2.triangulatePoints(P1, P2, points1.T, points2.T)
        points_3d = points_4d[:3] / points_4d[3]  # Convert from homogeneous coordinates
        
        return points_3d.T


class VisualSLAM:
    """Main Visual SLAM class integrating all components"""
    
    def __init__(self, camera_params: CameraParams):
        self.tracker = FeatureTracker()
        self.odometry = VisualOdometry(camera_params)
        self.mapping = Mapping(camera_params)
        self.prev_frame = None
        self.prev_features = None
        self.map_points = []
        self.keyframes = []

    def process_frame(self, frame: np.ndarray) -> Tuple[np.ndarray, List[np.ndarray]]:
        """Process a new frame"""
        if self.prev_frame is None:
            self.prev_frame = frame
            self.prev_features = self.tracker.detect_features(frame)
            return np.eye(4), []

        # Track features
        prev_pts, curr_pts = self.tracker.track_features(
            self.prev_frame, frame, self.prev_features
        )

        if prev_pts is not None and len(prev_pts) >= 8:
            # Estimate motion
            pose = self.odometry.estimate_motion(prev_pts, curr_pts)
            
            # Triangulate new points if this is a keyframe
            if self._is_keyframe(pose):
                points_3d = self.mapping.triangulate_points(
                    prev_pts, curr_pts,
                    np.eye(4), pose
                )
                self.map_points.extend(points_3d.tolist())
                self.keyframes.append({
                    'frame': frame,
                    'pose': pose,
                    'points': curr_pts
                })
        else:
            pose = np.eye(4)
            self.prev_features = self.tracker.detect_features(frame)

        # Update previous frame and features
        self.prev_frame = frame
        self.prev_features = self.tracker.detect_features(frame)

        return pose, self.map_points

    def _is_keyframe(self, pose: np.ndarray, min_distance: float = 0.3) -> bool:
        """Determine if the current frame should be a keyframe"""
        if not self.keyframes:
            return True
        
        last_pose = self.keyframes[-1]['pose']
        distance = np.linalg.norm(pose[:3, 3] - last_pose[:3, 3])
        return distance > min_distance

    def visualize(self):
        """Visualize the camera trajectory and 3D map"""
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot trajectory
        trajectory = np.array(self.odometry.trajectory)
        ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], 'r-')
        
        # Plot map points
        if self.map_points:
            points = np.array(self.map_points)
            ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='blue', s=1)
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()


def demo_visual_slam():
    """Demo function to test Visual SLAM with video input"""
    # Example camera parameters (adjust according to your camera)
    camera_params = CameraParams(
        fx=525.0,  # focal length x
        fy=525.0,  # focal length y
        cx=320.0,  # principal point x
        cy=240.0,  # principal point y
        baseline=0.075  # for stereo setup
    )
    
    slam = VisualSLAM(camera_params)
    
    # Open video capture (adjust source as needed)
    cap = cv2.VideoCapture(0)  # Use 0 for webcam
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
                
            # Process frame
            pose, map_points = slam.process_frame(frame)
            
            # Visualize current frame with tracked features
            if slam.prev_features is not None:
                for pt in slam.prev_features:
                    pt = pt.ravel()
                    cv2.circle(frame, (int(pt[0]), int(pt[1])), 3, (0, 255, 0), -1)
            
            # Display frame
            cv2.imshow('Frame', frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    finally:
        cap.release()
        cv2.destroyAllWindows()
        
        # Visualize final trajectory and map
        slam.visualize()


if __name__ == "__main__":
    demo_visual_slam()