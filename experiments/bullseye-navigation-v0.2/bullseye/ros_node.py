"""ROS adapter. Preview mode constructs no motion client or control publisher."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image, CompressedImage, Range
from std_msgs.msg import String, Empty

from .geometry import CameraGeometry, Pose, wrap
from .mission import Mission, configuration_errors
from .planner import rollout
from .symbols import SymbolReader
from .vision import MarkerDetector


def stamp_seconds(header):
    return header.stamp.sec+header.stamp.nanosec/1e9


class NavigationNode(Node):
    def __init__(self, root, config, camera_config, motion=False, display=False, seconds=None,
                 capture_calibration=False):
        super().__init__('bullseye_navigation')
        self.root, self.config = root, config
        self.motion, self.display, self.seconds = motion, display, seconds
        self.capture_calibration = capture_calibration
        self.capture_count, self.last_capture = 0, -math.inf
        self.geometry = CameraGeometry(camera_config)
        self.engine = Mission(config)
        self.config_errors = configuration_errors(config, self.geometry)
        self.detector = MarkerDetector(root/config['vision']['reference'],
                                       min_quality=config['vision']['min_quality'])
        self.bridge = CvBridge()
        self.worker = ThreadPoolExecutor(max_workers=1)
        self.vision_future = self.move_future = self.parameter_future = None
        self.latest = None
        self.processed_stamp = -math.inf
        self.poses = deque(maxlen=300)
        self.ranges = {}
        self.last_vision = -math.inf
        self.created = time.monotonic()
        self.last_report = self.last_graph_check = -math.inf
        self.last_graph_fault = None
        self.planned = None
        self.moving = False
        self.armed = self.fault_sent = self.result_sent = False
        self.inspection_pose = None
        self.reader = None
        self.preview = {'markers': [], 'frames_received': 0, 'frames_processed': 0,
                        'calibrated': self.geometry.calibrated()}
        self.reports = root/'reports'
        self.reports.mkdir(exist_ok=True)
        self.log = (self.reports/'events.jsonl').open('a')
        self.annotated_pub = self.create_publisher(CompressedImage, '/bullseye/image_annotated/compressed', 1)
        self.status_pub = self.create_publisher(String, '/bullseye/status', 10)
        self.create_subscription(Image, config['vision']['image_topic'], self.on_image, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/robot_pose/raw', self.on_pose, qos_profile_sensor_data)
        for name in ('ultrasonic', 'ir_left', 'ir_right'):
            self.create_subscription(Range, '/sensors/'+name,
                                     lambda msg, name=name: self.on_range(name, msg), qos_profile_sensor_data)
        if motion:
            if self.config_errors:
                raise ValueError('Motion configuration incomplete: '+'; '.join(self.config_errors))
            from mdp_interfaces.srv import ExecuteMoves
            from rcl_interfaces.srv import GetParameters
            self.move_client = self.create_client(ExecuteMoves, '/execute_moves')
            self.parameter_client = self.create_client(GetParameters, '/motion_controller_node/get_parameters')
            self.estop_pub = self.create_publisher(Empty, '/estop', 10)
            self.result_pub = self.create_publisher(String, config['motion']['result_topic'], 10)
            self.create_subscription(Empty, '/estop', lambda msg: self.stop('external_estop'), 10)
            self.create_subscription(Twist, '/cmd_vel/teleop', lambda msg: self.stop('manual_control_received'), 1)
            self.create_subscription(String, '/android/cmd', lambda msg: self.stop('android_control_received'), 10)
            spec = config['recognition']
            self.reader = SymbolReader(spec['model'], spec['sha256'], spec['threads'],
                                       spec['confidence'], spec['invert_colors'])
        self.timer = self.create_timer(.05, self.tick)
        self.get_logger().info('MOTION REQUESTED: checking prerequisites' if motion else
                               'PREVIEW ONLY: no motion client or control publisher exists')

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    def event(self, kind, **fields):
        self.log.write(json.dumps({'time': self.now(), 'event': kind, **fields})+'\n')
        self.log.flush()

    def on_image(self, msg):
        stamp = stamp_seconds(msg.header)
        if not 0 <= self.now()-stamp <= self.config['vision']['max_frame_age_s']:
            return
        if self.latest is not None and stamp <= self.latest[1]:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().warning(f'Image decode failed: {exc}')
            return
        self.latest = (frame, stamp)
        self.preview['frames_received'] += 1
        if self.capture_calibration and self.capture_count < 20 and time.monotonic()-self.last_capture >= 3:
            directory = self.reports/'calibration-frames'
            directory.mkdir(exist_ok=True)
            cv2.imwrite(str(directory/f'frame-{stamp:.9f}.png'), frame)
            self.capture_count += 1
            self.last_capture = time.monotonic()

    def on_pose(self, msg):
        if msg.header.frame_id != 'odom':
            return
        q, p = msg.pose.orientation, msg.pose.position
        values = [q.x, q.y, q.z, q.w, p.x, p.y]
        if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values[:4])-1) > .02:
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        stamp = stamp_seconds(msg.header)
        if self.poses and stamp <= self.poses[-1][0]:
            return
        self.poses.append((stamp, Pose(p.x, p.y, yaw)))

    def on_range(self, name, msg):
        self.ranges[name] = (stamp_seconds(msg.header), float(msg.range))

    def pose_at(self, stamp):
        items = list(self.poses)
        for (ta, a), (tb, b) in zip(items, items[1:]):
            if ta <= stamp <= tb and tb-ta <= .15:
                factor = (stamp-ta)/(tb-ta)
                return Pose(a.x+(b.x-a.x)*factor, a.y+(b.y-a.y)*factor,
                            wrap(a.yaw+wrap(b.yaw-a.yaw)*factor))
        if items and abs(items[-1][0]-stamp) <= .05:
            return items[-1][1]
        return None

    def safety_error(self):
        now, cfg = self.now(), self.config['motion']
        if not self.poses or not 0 <= now-self.poses[-1][0] <= cfg['max_pose_age_s']:
            return 'odometry_missing_or_stale'
        for name in ('ultrasonic', 'ir_left', 'ir_right'):
            stamp, distance = self.ranges.get(name, (-math.inf, math.nan))
            if not 0 <= now-stamp <= .4 or not math.isfinite(distance) or distance <= 0:
                return name+'_invalid_or_stale'
            limit = cfg['front_stop_m'] if name == 'ultrasonic' else cfg['ir_stop_m']
            if distance <= limit:
                return name+'_too_close'
        if self.engine.obstacle is not None and not self.engine.scene().clear(self.poses[-1][1]):
            return 'robot_footprint_not_clear'
        return None

    def graph_error(self):
        now = time.monotonic()
        if now-self.last_graph_check >= .5:
            self.last_graph_check = now
            names = self.get_node_names()
            competitors = [name for name in names if
                           'planner_node' in name or 'fastest_car' in name or 'teleop_keyboard' in name]
            if names.count('bullseye_navigation') != 1:
                competitors.append('duplicate_bullseye_navigation')
            self.last_graph_fault = 'competing_controller:'+','.join(competitors) if competitors else None
        return self.last_graph_fault

    def stop(self, reason):
        if not self.motion:
            return
        self.planned = None
        self.engine.stop(reason, self.now())
        if self.armed and not self.fault_sent:
            self.estop_pub.publish(Empty())
            self.fault_sent = True
            self.event('stop', reason=reason)

    def preflight(self):
        if self.engine.state == 'STOPPED':
            return
        error = self.safety_error() or self.graph_error()
        if error:
            self.engine.reason = 'preflight:'+error
            return
        if not self.move_client.service_is_ready() or not self.parameter_client.service_is_ready():
            self.engine.reason = 'preflight:motion_service_or_parameters_unavailable'
            return
        if self.parameter_future is None:
            from rcl_interfaces.srv import GetParameters
            request = GetParameters.Request()
            request.names = ['velocity_speed_mps', 'velocity_turn_radius_m']
            self.parameter_future = self.parameter_client.call_async(request)
            self.parameter_requested = time.monotonic()
            return
        if not self.parameter_future.done():
            if time.monotonic()-self.parameter_requested > 3:
                self.stop('controller_parameter_timeout')
            return
        try:
            values = self.parameter_future.result().values
            if len(values) != 2 or any(v.type != 3 for v in values):
                raise ValueError('controller parameters unavailable or not double values')
            speed, radius = (v.double_value for v in values)
            cfg = self.config['motion']
            if not 0 < speed <= cfg['max_controller_speed_mps'] or abs(radius-cfg['turning_radius_m']) > .001:
                raise ValueError(f'controller speed/radius mismatch: {speed}, {radius}')
        except Exception as exc:
            self.stop('preflight:'+str(exc))
            return
        self.armed = True
        self.engine.start(self.now())
        self.event('armed', controller_speed=speed, turning_radius=radius)

    def process_image(self, frame, stamp, pose, epoch, inspect, roi):
        start = time.monotonic()
        markers = self.detector.detect(frame, stamp)
        estimate = self.geometry.estimate(markers[0], frame.shape) if len(markers) == 1 else None
        obstacle = (self.geometry.obstacle(estimate, pose, self.config['obstacle_size_m'])
                    if estimate is not None and pose is not None else None)
        detections = self.reader.predict(frame, roi) if inspect and roi is not None else []
        return {'frame': frame, 'stamp': stamp, 'epoch': epoch, 'markers': markers,
                'estimate': estimate, 'obstacle': obstacle, 'detections': detections,
                'inspect': inspect, 'roi': roi, 'elapsed_s': time.monotonic()-start}

    def vision_tick(self):
        now = self.now()
        if self.vision_future is not None and self.vision_future.done():
            try:
                result = self.vision_future.result()
            except Exception as exc:
                self.preview['error'] = str(exc)
                self.stop('vision_error:'+str(exc))
            else:
                self.apply_vision(result)
            self.vision_future = None
        if self.vision_future is not None or self.latest is None:
            return
        if now-self.last_vision < 1/self.config['vision']['max_fps']:
            return
        frame, stamp = self.latest
        if stamp <= self.processed_stamp or not 0 <= now-stamp <= self.config['vision']['max_frame_age_s']:
            return
        pose = self.pose_at(stamp)
        inspect = (self.motion and self.engine.state == 'INSPECT' and
                   stamp > self.engine.inspection_after and not self.moving)
        roi = None
        if inspect and pose is not None:
            roi = self.geometry.face_roi(self.engine.obstacle, self.engine.face, pose, frame.shape)
        self.processed_stamp, self.last_vision = stamp, now
        self.vision_future = self.worker.submit(self.process_image, frame.copy(), stamp, pose,
                                                self.engine.epoch, inspect, roi)

    def apply_vision(self, result):
        now = self.now()
        self.preview['frames_processed'] += 1
        self.preview.update({'capture_stamp': result['stamp'], 'processing_s': result['elapsed_s'],
                             'result_age_s': now-result['stamp'],
                             'markers': [{'centre': m.centre.tolist(), 'quality': m.quality,
                                          'corners': m.corners.tolist()} for m in result['markers']],
                             'distance_m': result['estimate']['forward_depth_m'] if result['estimate'] else None})
        if self.motion and not self.moving and result['epoch'] == self.engine.epoch:
            self.engine.observe_marker(result['obstacle'], result['stamp'], now)
            self.engine.observe_symbols(result['detections'], result['stamp'], now, result['epoch'])
        frame = result['frame'].copy()
        for marker in result['markers']:
            cv2.polylines(frame, [marker.corners.astype(np.int32)], True, (0, 220, 0), 2)
            cv2.circle(frame, tuple(marker.centre.astype(int)), 4, (0, 0, 255), -1)
        if result['roi'] is not None:
            x1, y1, x2, y2 = result['roi']
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 200, 0), 2)
        mode = self.engine.state if self.motion else 'PREVIEW - MOTION DISABLED'
        label = mode+(' | calibration needed' if not self.geometry.calibrated() else '')
        cv2.putText(frame, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 220, 255), 1)
        cv2.imwrite(str(self.reports/'latest.jpg'), frame)
        msg = self.bridge.cv2_to_compressed_imgmsg(frame)
        msg.header.stamp.sec = int(result['stamp'])
        msg.header.stamp.nanosec = int((result['stamp'] % 1)*1e9)
        self.annotated_pub.publish(msg)
        if self.display:
            try:
                cv2.imshow('Bullseye navigation', frame)
                if cv2.waitKey(1) & 0xff == ord('q'):
                    self.stop('operator_quit')
                    rclpy.shutdown()
            except cv2.error as exc:
                self.display = False
                self.get_logger().warning(f'Display unavailable; use the annotated topic or reports/latest.jpg: {exc}')

    def motion_tick(self):
        if not self.armed:
            self.preflight()
            return
        error = self.safety_error() or self.graph_error()
        if error and self.engine.state not in {'STOPPED', 'COMPLETE', 'FAILED', 'STAGE_COMPLETE'}:
            self.stop(error)
        now = self.now()
        if self.move_future is not None:
            if self.move_future.done():
                try:
                    response = self.move_future.result()
                    self.engine.move_finished(response.success, response.status, now)
                    self.event('move_finished', success=response.success, status=response.status)
                except Exception as exc:
                    self.stop('motion_service_error:'+str(exc))
                self.move_future = None
                self.moving = False
            elif time.monotonic()-self.move_started > 3.0:
                self.stop('bounded_primitive_timeout')
            return
        if (self.engine.state not in {'COMPLETE', 'FAILED', 'STOPPED', 'STAGE_COMPLETE'} and
                now-self.engine.started_at > self.config['mission_timeout_s']):
            self.stop('mission_timeout')
        if self.engine.state in {'STOPPED', 'FAILED'}:
            self.stop(self.engine.reason)
            return
        if self.engine.state == 'STAGE_COMPLETE':
            return
        if self.engine.state == 'COMPLETE':
            if not self.result_sent:
                result = self.engine.result
                self.result_pub.publish(String(data=f"{result['obstacle_id']},{result['symbol_id']}"))
                self.result_sent = True
                self.event('result', **result)
            return
        pose = self.poses[-1][1]
        if self.engine.state == 'INSPECT':
            if self.inspection_pose is None:
                self.inspection_pose = pose
            old = self.inspection_pose
            if math.hypot(pose.x-old.x, pose.y-old.y) > .02 or abs(wrap(pose.yaw-old.yaw)) > math.radians(5):
                self.stop('robot_moved_during_inspection')
                return
        else:
            self.inspection_pose = None
        if self.planned is None:
            move = self.engine.next_move(pose, now)
            if move is not None:
                self.planned = (move, pose, self.engine.obstacle)
                plan = self.engine.last_plan
                if plan is not None:
                    (self.reports/'planned_path.json').write_text(json.dumps({
                        'poses': [asdict(p) for p in plan.poses], 'reason': plan.reason,
                        'moves': [asdict(m) for m in plan.moves]}, indent=2)+'\n')
            return  # Let fresh ROS callbacks run after potentially expensive planning.
        move, planned_pose, obstacle = self.planned
        if self.engine.state == 'APPROACH' and now-self.engine.marker_stamp > self.config['vision']['max_frame_age_s']:
            if now-self.engine.marker_stamp > self.config['vision']['lost_marker_timeout_s']:
                self.stop('marker_lost_during_approach')
            return
        if math.hypot(pose.x-planned_pose.x, pose.y-planned_pose.y) > .02 or abs(wrap(pose.yaw-planned_pose.yaw)) > .10:
            self.planned = None
            self.engine.route = []
            return
        if (self.engine.state == 'APPROACH' and (
                math.hypot(obstacle.x-self.engine.obstacle.x, obstacle.y-self.engine.obstacle.y) > .025 or
                abs(wrap(obstacle.yaw-self.engine.obstacle.yaw)) > math.radians(10))):
            self.planned = None
            return
        if not self.engine.scene().clear_move(pose, move, self.config['motion']['turning_radius_m']):
            self.stop('planned_move_no_longer_clear')
            return
        from mdp_interfaces.msg import MoveCommand
        from mdp_interfaces.srv import ExecuteMoves
        request = ExecuteMoves.Request()
        request.commands = [MoveCommand(command=move.command, value=move.value)]
        self.move_future = self.move_client.call_async(request)
        self.move_started = time.monotonic()
        self.moving = True
        self.planned = None
        self.event('move_requested', command=move.command, value=move.value, pose=asdict(pose))

    def tick(self):
        try:
            if self.motion:
                self.motion_tick()
            self.vision_tick()
            if time.monotonic()-self.last_report >= 1:
                self.last_report = time.monotonic()
                data = {'mode': 'motion' if self.motion else 'preview', 'time': self.now(),
                        'mission': self.engine.status(), 'preview': self.preview,
                        'configuration_errors': self.config_errors}
                payload = json.dumps(data, allow_nan=False)
                temp = self.reports/'status.tmp'
                temp.write_text(payload+'\n')
                temp.replace(self.reports/'status.json')
                self.status_pub.publish(String(data=payload))
                self.get_logger().info(f"{data['mode']}: {self.engine.state} | "
                                       f"markers={len(self.preview['markers'])} | {self.engine.reason}")
            if self.seconds is not None and time.monotonic()-self.created >= self.seconds:
                self.stop('run_duration_limit')
                rclpy.shutdown()
        except Exception as exc:
            self.stop('runtime_error:'+str(exc))
            self.get_logger().error(str(exc))

    def close(self):
        self.stop('node_shutdown')
        self.timer.cancel()
        self.worker.shutdown(wait=True, cancel_futures=True)
        self.log.close()
        if self.display:
            cv2.destroyAllWindows()
