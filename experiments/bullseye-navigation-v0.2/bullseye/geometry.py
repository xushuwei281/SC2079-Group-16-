"""Calibrated planar marker geometry. All distances in this module are metres."""
from dataclasses import dataclass
import math

import cv2
import numpy as np


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Obstacle:
    x: float
    y: float
    yaw: float  # outward normal of the initially observed face
    size: float = .10

    def viewpoint(self, face, standoff):
        angle = self.yaw + face*math.pi/2
        radius = self.size/2 + standoff
        return Pose(self.x + radius*math.cos(angle), self.y + radius*math.sin(angle),
                    wrap(angle+math.pi))


class CameraGeometry:
    def __init__(self, config):
        self.config = config

    def calibrated(self):
        c = self.config
        try:
            k = np.asarray(c['matrix'], float).reshape(3, 3)
            return bool(np.isfinite(k).all() and k[0, 0] > 0 and k[1, 1] > 0 and
                        np.allclose(k[2], [0, 0, 1]) and c['image_width'] > 0 and
                        c['image_height'] > 0 and c['marker_size_m'] > 0)
        except (KeyError, TypeError, ValueError):
            return False

    def estimate(self, marker, image_shape):
        if not self.calibrated():
            return None
        c = self.config
        if tuple(image_shape[:2]) != (c['image_height'], c['image_width']):
            return None
        k = np.asarray(c['matrix'], float).reshape(3, 3)
        distortion = np.asarray(c.get('distortion', [0]*5), float)
        half = float(c['marker_size_m'])/2
        object_points = np.array([[-half, half, 0], [half, half, 0],
                                  [half, -half, 0], [-half, -half, 0]], np.float64)
        # Some OpenCV builds return a degenerate IPPE orientation for an exactly
        # front-facing symmetric square. Score an iterative candidate as well;
        # neither solution is accepted without the same reprojection/depth checks.
        candidates = []
        for method in (cv2.SOLVEPNP_IPPE_SQUARE, cv2.SOLVEPNP_ITERATIVE):
            ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(
                object_points, marker.corners.astype(np.float64), k, distortion, flags=method)
            if ok:
                candidates.extend(zip(rvecs, tvecs))
        solutions = []
        for rvec, tvec in candidates:
            projected, _ = cv2.projectPoints(object_points, rvec, tvec, k, distortion)
            error = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2)-marker.corners)**2, axis=1))))
            rotation = cv2.Rodrigues(rvec)[0]
            translation = tvec.reshape(3)
            points_camera = object_points @ rotation.T + translation
            if not np.isfinite(translation).all() or points_camera[:, 2].min() <= 0:
                continue
            if not .08 <= translation[2] <= c.get('max_range_m', 2.0):
                continue
            normal = rotation[:, 2]
            if float(normal @ translation) > 0:
                normal = -normal
            if error <= c.get('max_reprojection_px', 2.0):
                solutions.append((error, translation, normal))
        if not solutions:
            return None
        error, translation, normal = min(solutions, key=lambda item: item[0])
        return {'translation': translation, 'normal': normal, 'error_px': error,
                'forward_depth_m': float(translation[2]),
                'bearing_rad': math.atan2(float(translation[0]), float(translation[2]))}

    def obstacle(self, estimate, robot_pose, size):
        c = self.config
        if c.get('base_from_camera_rotation') is None or c.get('camera_origin_in_base_m') is None:
            return None
        rotation = np.asarray(c['base_from_camera_rotation'], float).reshape(3, 3)
        origin = np.asarray(c['camera_origin_in_base_m'], float).reshape(3)
        if (not np.isfinite(rotation).all() or not np.isfinite(origin).all() or
                not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3) or
                not np.isclose(np.linalg.det(rotation), 1, atol=1e-3)):
            return None
        face_base = rotation @ estimate['translation'] + origin
        normal_base = rotation @ estimate['normal']
        if abs(normal_base[2]) > .35:
            return None  # This mission models an upright obstacle face.
        norm = np.linalg.norm(normal_base[:2])
        if norm < .8:
            return None
        normal_base = normal_base[:2]/norm
        centre_base = face_base[:2] - size/2*normal_base
        cs, sn = math.cos(robot_pose.yaw), math.sin(robot_pose.yaw)
        world = np.array([[cs, -sn], [sn, cs]])
        centre = world @ centre_base + np.array([robot_pose.x, robot_pose.y])
        normal = world @ normal_base
        return Obstacle(float(centre[0]), float(centre[1]), math.atan2(normal[1], normal[0]), size)

    def face_roi(self, obstacle, face, robot_pose, image_shape):
        c = self.config
        if c.get('marker_center_height_m') is None or not self.calibrated():
            return None
        angle = obstacle.yaw+face*math.pi/2
        normal = np.array([math.cos(angle), math.sin(angle), 0.0])
        tangent = np.array([-math.sin(angle), math.cos(angle), 0.0])
        centre = np.array([obstacle.x, obstacle.y, c['marker_center_height_m']])+normal*obstacle.size/2
        half = c['marker_size_m']/2
        world_points = np.array([centre+tangent*x+np.array([0, 0, z])
                                 for x, z in [(-half, half), (half, half), (half, -half), (-half, -half)]])
        cs, sn = math.cos(robot_pose.yaw), math.sin(robot_pose.yaw)
        world_from_base = np.array([[cs, -sn, 0], [sn, cs, 0], [0, 0, 1]])
        base = (world_points-np.array([robot_pose.x, robot_pose.y, 0])) @ world_from_base
        rotation = np.asarray(c['base_from_camera_rotation'], float).reshape(3, 3)
        camera = (base-np.asarray(c['camera_origin_in_base_m'], float)) @ rotation
        if camera[:, 2].min() <= .02:
            return None
        k = np.asarray(c['matrix'], float).reshape(3, 3)
        xy = cv2.projectPoints(camera, np.zeros(3), np.zeros(3), k,
                               np.asarray(c.get('distortion', [0]*5), float))[0].reshape(4, 2)
        if not np.isfinite(xy).all():
            return None
        lo, hi = xy.min(axis=0), xy.max(axis=0)
        pad = (hi-lo)*.15
        lo, hi = lo-pad, hi+pad
        h, w = image_shape[:2]
        x1, y1 = np.maximum(lo, [0, 0]).astype(int)
        x2, y2 = np.minimum(hi, [w, h]).astype(int)
        if x2-x1 < 20 or y2-y1 < 20:
            return None
        if (x2-x1)*(y2-y1) < .7*float(np.prod(hi-lo)):
            return None
        return int(x1), int(y1), int(x2), int(y2)


class Confirmation:
    """Require distinct, fresh frames and spatial agreement; reset on absence."""
    def __init__(self, frames=3, gap=.8):
        self.frames, self.gap = frames, gap
        self.reset()

    def reset(self):
        self.value, self.count, self.stamp = None, 0, -math.inf

    def update(self, value, stamp, now, max_age, compatible=None):
        if value is None or not math.isfinite(stamp) or not 0 <= now-stamp <= max_age:
            self.reset()
            return False
        if stamp <= self.stamp:
            return False
        same = (compatible(value, self.value) if compatible and self.value is not None
                else value == self.value)
        self.count = self.count+1 if same and stamp-self.stamp <= self.gap else 1
        self.value, self.stamp = value, stamp
        return self.count >= self.frames
