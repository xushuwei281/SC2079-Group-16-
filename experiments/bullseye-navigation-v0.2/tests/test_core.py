import copy
import math
from pathlib import Path
import unittest

import cv2
import numpy as np
import yaml

from bullseye.geometry import CameraGeometry, Confirmation, Obstacle, Pose, wrap
from bullseye.vision import Marker, MarkerDetector
from bullseye.mission import Mission, configuration_errors
from bullseye.planner import Move, Scene, at_goal, plan_route, rollout, rectangle, overlaps
from bullseye.symbols import symbol_id

ROOT = Path(__file__).resolve().parents[1]


def config():
    cfg = yaml.safe_load((ROOT/'config/mission.yaml').read_text())
    cfg['motion']['bounds_m'] = [0., 0., 2., 2.]
    cfg['motion']['allow_reverse'] = True
    return cfg


def camera_config():
    return {'image_width': 640, 'image_height': 480,
            'matrix': [500., 0., 320., 0., 500., 240., 0., 0., 1.],
            'distortion': [0.]*5, 'marker_size_m': .10,
            'base_from_camera_rotation': [0, 0, 1, -1, 0, 0, 0, -1, 0],
            'camera_origin_in_base_m': [.10, 0, .10], 'marker_center_height_m': .10}


class VisionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.detector = MarkerDetector(ROOT/'assets/bullseye-reference.png')
        cls.reference = cv2.imread(str(ROOT/'assets/bullseye-reference.png'))

    def test_supplied_marker(self):
        markers = self.detector.detect(self.reference, 1.)
        self.assertEqual(len(markers), 1)
        self.assertGreater(markers[0].quality, .85)

    def test_perspective_blur_and_lighting(self):
        h, w = self.reference.shape[:2]
        src = np.float32([[0, 0], [w-1, 0], [w-1, h-1], [0, h-1]])
        dest = np.float32([[110, 65], [350, 95], [315, 345], [85, 305]])
        transform = cv2.getPerspectiveTransform(src, dest)
        frame = cv2.warpPerspective(self.reference, transform, (640, 480), borderValue=(255,)*3)
        frame = cv2.GaussianBlur(frame, (3, 3), .8)
        frame = np.clip(frame.astype(float)*.65+25, 0, 255).astype(np.uint8)
        self.assertEqual(len(self.detector.detect(frame, 2.)), 1)

    def test_reject_blank_letter_circle_and_plain_square(self):
        for kind in ('blank', 'F', 'circle', 'square'):
            with self.subTest(kind=kind):
                frame = np.full((480, 640, 3), 255, np.uint8)
                if kind == 'F':
                    cv2.putText(frame, 'F', (210, 320), cv2.FONT_HERSHEY_SIMPLEX, 6, (0,)*3, 15)
                elif kind == 'circle':
                    cv2.circle(frame, (320, 240), 100, (0,)*3, -1)
                elif kind == 'square':
                    cv2.rectangle(frame, (210, 130), (430, 350), (0,)*3, 20)
                self.assertEqual(self.detector.detect(frame, 1.), [])


class GeometryTests(unittest.TestCase):
    def test_missing_calibration_produces_no_invented_distance(self):
        geometry = CameraGeometry({'matrix': None})
        self.assertFalse(geometry.calibrated())
        self.assertIsNone(geometry.estimate(Marker(np.zeros((4, 2)), 1, 0), (480, 640, 3)))

    def test_known_square_depth_and_base_transform(self):
        geometry = CameraGeometry(camera_config())
        marker = Marker(np.float32([[270, 190], [370, 190], [370, 290], [270, 290]]), 1., 1.)
        estimate = geometry.estimate(marker, (480, 640, 3))
        self.assertIsNotNone(estimate)
        self.assertAlmostEqual(estimate['forward_depth_m'], .50, places=3)
        obstacle = geometry.obstacle(estimate, Pose(.5, .5, 0), .10)
        self.assertAlmostEqual(obstacle.x, 1.15, places=3)
        self.assertAlmostEqual(obstacle.y, .50, places=3)
        self.assertLess(abs(wrap(obstacle.yaw-math.pi)), .01)

    def test_resolution_mismatch_rejected(self):
        geometry = CameraGeometry(camera_config())
        marker = Marker(np.float32([[270, 190], [370, 190], [370, 290], [270, 290]]), 1., 1.)
        self.assertIsNone(geometry.estimate(marker, (720, 1280, 3)))

    def test_face_roi_uses_obstacle_location(self):
        geometry = CameraGeometry(camera_config())
        roi = geometry.face_roi(Obstacle(1.15, .5, math.pi), 0, Pose(.5, .5, 0), (480, 640, 3))
        self.assertIsNotNone(roi)
        x1, y1, x2, y2 = roi
        self.assertLess(x1, 270); self.assertGreater(x2, 370)
        self.assertLess(y1, 190); self.assertGreater(y2, 290)


class PlannerTests(unittest.TestCase):
    def test_optimized_collision_matches_independent_vertex_projection(self):
        rng = np.random.default_rng(16)
        obstacle = Obstacle(1, 1, .63)
        scene = Scene([0, 0, 2, 2], [obstacle], centre_offset=(.03, -.01))
        target = rectangle(1, 1, .63, .10, .10)
        for _ in range(600):
            x, y = rng.uniform(0, 2, 2)
            angle = rng.uniform(-math.pi, math.pi)
            cx = x+math.cos(angle)*.03+math.sin(angle)*.01
            cy = y+math.sin(angle)*.03-math.cos(angle)*.01
            body = rectangle(cx, cy, angle, .23+.06, .19+.06)
            expected = all(0 < px < 2 and 0 < py < 2 for px, py in body) and not overlaps(body, target)
            self.assertEqual(scene.clear(Pose(x, y, angle)), expected)

    def test_reverse_turn_matches_ackermann_motion(self):
        end = rollout(Pose(0, 0, 0), Move('BL', 90))[-1]
        self.assertAlmostEqual(end.x, -.21)
        self.assertAlmostEqual(end.y, .21)
        self.assertAlmostEqual(end.yaw, -math.pi/2)

    def test_obstacle_and_wall_include_full_chassis(self):
        scene = Scene([0, 0, 2, 2], [Obstacle(1, 1, .6)])
        self.assertFalse(scene.clear(Pose(1, 1, 0)))
        self.assertFalse(scene.clear(Pose(.05, .5, 0)))
        self.assertTrue(scene.clear(Pose(.5, .5, .7)))
        self.assertFalse(scene.clear_move(Pose(.7, 1, 0), Move('FC', 50), .21))

    def test_adjacent_face_route_checks_every_sample(self):
        obstacle = Obstacle(1, 1, -math.pi/2)
        scene = Scene([0, 0, 2, 2], [obstacle])
        start, goal = obstacle.viewpoint(0, .30), obstacle.viewpoint(1, .30)
        route = plan_route(start, goal, scene, allow_reverse=True, max_seconds=5)
        self.assertEqual(route.reason, 'ok')
        self.assertTrue(route.moves)
        self.assertTrue(all(scene.clear(p) for p in route.poses))
        self.assertTrue(at_goal(route.poses[-1], goal))

    def test_unreachable_viewpoint_refused(self):
        obstacle = Obstacle(.10, 1, 0)
        scene = Scene([0, 0, 2, 2], [obstacle])
        route = plan_route(Pose(1, 1, 0), obstacle.viewpoint(2, .30), scene)
        self.assertEqual(route.reason, 'start_or_goal_not_clear')
        self.assertFalse(route.moves)


class ConfirmationTests(unittest.TestCase):
    def test_duplicate_and_stale_frames_do_not_confirm(self):
        c = Confirmation(3)
        self.assertFalse(c.update(25, 1., 1., .5))
        self.assertFalse(c.update(25, 1., 1.1, .5))
        self.assertEqual(c.count, 1)
        self.assertFalse(c.update(25, 1.2, 2., .5))
        self.assertEqual(c.count, 0)

    def test_filled_circle_is_valid_but_marker_is_not(self):
        self.assertEqual(symbol_id('40'), 40)
        for name in ('marker', 'target', 'bullseye', 'unknown', '41', '0'):
            self.assertIsNone(symbol_id(name))

    def test_face_change_and_old_epoch_reject_previous_result(self):
        engine = Mission(config())
        engine.state, engine.epoch, engine.inspection_after = 'INSPECT', 2, 10.
        detections = [{'symbol_id': 25, 'confidence': .95}]
        for stamp in (11., 11.1, 11.2):
            engine.observe_symbols(detections, stamp, stamp, epoch=1)
        self.assertIsNone(engine.result)
        for stamp in (11., 11.1, 11.2):
            engine.observe_symbols(detections, stamp, stamp, epoch=2)
        self.assertEqual(engine.result['symbol_id'], 25)

    def test_marker_does_not_finish_mission(self):
        engine = Mission(config())
        engine.state, engine.epoch, engine.inspection_after = 'INSPECT', 2, 10.
        for stamp in (11., 11.1, 11.2):
            engine.observe_symbols([{'symbol_id': None, 'confidence': .99}], stamp, stamp, 2)
        self.assertIsNone(engine.result)


class MissionTests(unittest.TestCase):
    def test_approach_only_stage_never_orbits_or_reports_symbol(self):
        cfg = config(); cfg['mission_stage'] = 'approach'
        engine = Mission(cfg); engine.start(1.)
        obstacle = Obstacle(1, 1, -math.pi/2)
        for stamp in (1.1, 1.2, 1.3):
            engine.observe_marker(obstacle, stamp, stamp)
        self.assertIsNone(engine.next_move(obstacle.viewpoint(0, .30), 1.3))
        self.assertEqual(engine.state, 'STAGE_COMPLETE')
        self.assertIsNone(engine.result)
        self.assertEqual(engine.visited, {0})

    def test_invalid_footprint_cannot_pass_preflight(self):
        cfg = config(); cfg['motion']['robot_width_m'] = -.2
        errors = configuration_errors(cfg, CameraGeometry(camera_config()))
        self.assertTrue(any('robot_width_m' in e for e in errors))

    def test_default_configuration_blocks_motion(self):
        cfg = yaml.safe_load((ROOT/'config/mission.yaml').read_text())
        camera = CameraGeometry(yaml.safe_load((ROOT/'config/camera_calibration.yaml').read_text()))
        self.assertGreater(len(configuration_errors(cfg, camera)), 5)

    def test_lost_marker_holds_then_stops(self):
        engine = Mission(config())
        engine.start(1.)
        ob = Obstacle(1, 1, -math.pi/2)
        for stamp in (1.1, 1.2, 1.3):
            engine.observe_marker(ob, stamp, stamp)
        self.assertEqual(engine.state, 'APPROACH')
        self.assertIsNone(engine.next_move(Pose(1, .4, math.pi/2), 2.))
        self.assertIsNone(engine.next_move(Pose(1, .4, math.pi/2), 8.))
        self.assertEqual(engine.state, 'STOPPED')

    def test_marker_jump_stops(self):
        engine = Mission(config()); engine.start(1.)
        for stamp in (1.1, 1.2, 1.3):
            engine.observe_marker(Obstacle(1, 1, 0), stamp, stamp)
        for stamp in (1.4, 1.5, 1.6):
            engine.observe_marker(Obstacle(1.3, 1, 0), stamp, stamp)
        self.assertEqual(engine.state, 'STOPPED')

    def test_full_mission_reaches_other_face_and_confirms(self):
        engine = Mission(config()); engine.start(10.)
        obstacle = Obstacle(1, 1, -math.pi/2)
        pose = Pose(1, .40, math.pi/2)
        now, moved = 10., 0
        for _ in range(1200):
            now += .1
            if engine.state in {'ACQUIRE', 'APPROACH'}:
                engine.observe_marker(obstacle, now, now)
            if engine.state == 'INSPECT' and now > engine.inspection_after:
                engine.observe_symbols([{'symbol_id': 25, 'confidence': .95}], now, now, engine.epoch)
            move = engine.next_move(pose, now)
            if move:
                scene = engine.scene()
                self.assertTrue(scene.clear_move(pose, move, .21))
                pose = rollout(pose, move)[-1]
                now += .5
                engine.move_finished(True, 'FIN', now)
                moved += 1
            if engine.state in {'COMPLETE', 'FAILED', 'STOPPED'}:
                break
        self.assertEqual(engine.state, 'COMPLETE', engine.reason)
        self.assertEqual(engine.result['symbol_id'], 25)
        self.assertNotEqual(engine.result['face'], 0)
        self.assertGreater(moved, 10)

    def test_failed_motion_cannot_continue(self):
        engine = Mission(config()); engine.start(1.)
        engine.move_finished(False, 'SENSOR_STALE', 2.)
        self.assertEqual(engine.state, 'STOPPED')
        self.assertIsNone(engine.next_move(Pose(1, 1, 0), 3.))


if __name__ == '__main__':
    unittest.main()
