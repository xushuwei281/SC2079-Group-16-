"""Nested-square detection using the user's reference, without a YOLO dependency."""
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass
class Marker:
    corners: np.ndarray
    quality: float
    stamp: float

    @property
    def centre(self):
        return self.corners.mean(axis=0)


def order_corners(points):
    points = np.asarray(points, dtype=np.float32).reshape(4, 2)
    centre = points.mean(axis=0)
    order = np.argsort(np.arctan2(points[:, 1] - centre[1], points[:, 0] - centre[0]))
    points = points[order]
    points = np.roll(points, -np.argmin(points.sum(axis=1)), axis=0)
    return points


def warp_square(gray, corners, size=128):
    target = np.array([[0, 0], [size-1, 0], [size-1, size-1], [0, size-1]], np.float32)
    matrix = cv2.getPerspectiveTransform(order_corners(corners), target)
    return cv2.warpPerspective(gray, matrix, (size, size))


class MarkerDetector:
    def __init__(self, reference, min_area=500, min_quality=0.80):
        self.min_area = min_area
        self.min_quality = min_quality
        gray = cv2.imread(str(Path(reference)), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            raise ValueError(f"Cannot read marker reference: {reference}")
        binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            raise ValueError("Marker reference has no outline")
        contour = max(contours, key=cv2.contourArea)
        quad = cv2.approxPolyDP(contour, 0.025*cv2.arcLength(contour, True), True)
        if len(quad) != 4:
            raise ValueError("Marker reference must have a quadrilateral outer boundary")
        patch = warp_square(gray, quad)
        self.template = cv2.threshold(patch, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1] > 0

    def detect(self, frame, stamp):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        masks = [cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1],
                 cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                       cv2.THRESH_BINARY_INV, 31, 5)]
        found = []
        h, w = gray.shape
        for mask in masks:
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if hierarchy is None:
                continue
            hierarchy = hierarchy[0]
            for i, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if area < self.min_area or area > h*w*.90:
                    continue
                quad = cv2.approxPolyDP(contour, .025*cv2.arcLength(contour, True), True)
                if len(quad) != 4 or not cv2.isContourConvex(quad):
                    continue
                corners = order_corners(quad)
                if (corners[:, 0].min() < 2 or corners[:, 1].min() < 2 or
                        corners[:, 0].max() > w-3 or corners[:, 1].max() > h-3):
                    continue
                depth, child = 0, hierarchy[i][2]
                while child >= 0 and depth < 20:
                    depth += 1
                    child = hierarchy[child][2]
                if depth < 4:
                    continue
                patch = warp_square(gray, corners)
                patch = cv2.threshold(patch, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1] > 0
                score = max(float(np.mean(np.rot90(patch, k)[4:-4, 4:-4] ==
                                          self.template[4:-4, 4:-4])) for k in range(4))
                if score < self.min_quality:
                    continue
                if any(np.linalg.norm(corners.mean(0)-m.centre) < 15 for m in found):
                    continue
                found.append(Marker(corners, score, stamp))
        return sorted(found, key=lambda item: item.quality, reverse=True)
