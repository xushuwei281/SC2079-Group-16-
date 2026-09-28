"""Hardware-independent mission states. All effects are returned to the ROS adapter."""
from dataclasses import asdict
import math

from .geometry import Confirmation, Obstacle, wrap
from .planner import Scene, at_goal, plan_route


def configuration_errors(config, camera):
    errors = []
    if config.get('mission_stage', 'full') not in {'approach', 'one_face', 'full'}:
        errors.append('mission_stage must be approach, one_face or full')
    if not camera.calibrated():
        errors.append('camera intrinsics are missing or invalid')
    c = camera.config
    for key in ('range_verified', 'mount_verified'):
        if c.get(key) is not True:
            errors.append(f'camera.{key} must be verified')
    for key in ('base_from_camera_rotation', 'camera_origin_in_base_m', 'marker_center_height_m'):
        if c.get(key) is None:
            errors.append(f'camera.{key} is missing')
    cfg = config['motion']
    for key in ('cleared_area_verified', 'exclusive_control_confirmed', 'chassis_geometry_verified'):
        if cfg.get(key) is not True:
            errors.append(f'motion.{key} is not verified')
    bounds = cfg.get('bounds_m')
    if not (isinstance(bounds, list) and len(bounds) == 4 and
            all(isinstance(x, (int, float)) and math.isfinite(x) for x in bounds) and
            bounds[0] < bounds[2] and bounds[1] < bounds[3]):
        errors.append('motion.bounds_m must be measured in the odometry frame')
    if not .21 <= cfg['turning_radius_m'] <= 1.0:
        errors.append('turning radius must be between 0.21 and 1.0 metres')
    if not .02 <= cfg['clearance_m'] <= .20:
        errors.append('clearance margin must be between 0.02 and 0.20 metres')
    if not 0 < cfg['max_controller_speed_mps'] <= .10:
        errors.append('first-test controller speed limit must be at most 0.10 m/s')
    for key, lower, upper in [('robot_width_m', .19, .6), ('robot_length_m', .23, .6),
                              ('view_standoff_m', .25, .8), ('max_pose_age_s', .05, .4),
                              ('front_stop_m', .12, .5), ('ir_stop_m', .10, .5)]:
        value = cfg.get(key)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or not lower <= value <= upper:
            errors.append(f'motion.{key} must be finite and within [{lower}, {upper}]')
    offset = cfg.get('footprint_center_offset_m')
    if not (isinstance(offset, list) and len(offset) == 2 and
            all(isinstance(x, (int, float)) and math.isfinite(x) and abs(x) <= .3 for x in offset)):
        errors.append('motion.footprint_center_offset_m must contain two measured, finite offsets')
    return errors


class Mission:
    def __init__(self, config):
        self.config = config
        self.state, self.reason = 'IDLE', 'not_started'
        self.obstacle = None
        self.face = 0
        self.visited = set()
        self.route = []
        self.last_plan = None
        self.marker_stamp = -math.inf
        self.last_move_finished = -math.inf
        self.inspection_after = math.inf
        self.started_at = None
        self.state_since = 0
        self.result = None
        self.epoch = 0
        self.marker_confirmation = Confirmation(3, .8)
        self.symbol_confirmation = Confirmation(3, config['recognition']['max_result_age_s'])

    def transition(self, state, reason, now):
        self.state, self.reason, self.state_since = state, reason, now
        self.epoch += 1

    def start(self, now):
        self.started_at = now
        self.transition('ACQUIRE', 'waiting_for_marker', now)

    def stop(self, reason, now):
        self.route = []
        self.transition('STOPPED', reason, now)

    def observe_marker(self, obstacle, stamp, now):
        if self.state not in {'ACQUIRE', 'APPROACH'}:
            return
        good = self.marker_confirmation.update(
            obstacle, stamp, now, self.config['vision']['max_frame_age_s'],
            compatible=lambda a, b: math.hypot(a.x-b.x, a.y-b.y) < .04 and
            abs(wrap(a.yaw-b.yaw)) < math.radians(12))
        if not good:
            return
        if self.obstacle is not None and (
                math.hypot(obstacle.x-self.obstacle.x, obstacle.y-self.obstacle.y) > .10 or
                abs(wrap(obstacle.yaw-self.obstacle.yaw)) > math.radians(25)):
            self.stop('marker_identity_or_pose_jump', now)
            return
        self.obstacle = obstacle
        self.marker_stamp = stamp
        # Approach segments are replanned from each new observation.
        if self.state == 'ACQUIRE':
            self.transition('APPROACH', 'marker_confirmed', now)

    def observe_symbols(self, detections, stamp, now, epoch):
        if self.state != 'INSPECT' or epoch != self.epoch or stamp <= self.inspection_after:
            return
        if not 0 <= now-stamp <= self.config['recognition']['max_result_age_s']:
            return
        valid = {d['symbol_id'] for d in detections
                 if d['symbol_id'] is not None and d['confidence'] >= self.config['recognition']['confidence']}
        # A frame containing both a marker and a target, or conflicting targets,
        # cannot establish which face supplies the result.
        marker = any(d['symbol_id'] is None for d in detections)
        value = next(iter(valid)) if len(valid) == 1 and not marker else None
        if self.symbol_confirmation.update(value, stamp, now,
                                            self.config['recognition']['max_result_age_s']):
            self.result = {'obstacle_id': self.config['obstacle_id'], 'symbol_id': value,
                           'face': self.face, 'stamp': stamp}
            self.transition('COMPLETE', 'symbol_confirmed', now)

    def scene(self):
        cfg = self.config['motion']
        others = [Obstacle(**item) for item in cfg.get('other_obstacles', [])]
        return Scene(cfg['bounds_m'], [self.obstacle]+others,
                     cfg['robot_width_m'], cfg['robot_length_m'], cfg['clearance_m'],
                     cfg['footprint_center_offset_m'])

    def make_plan(self, pose, face):
        cfg = self.config['motion']
        goal = self.obstacle.viewpoint(face, cfg['view_standoff_m'])
        plan = plan_route(pose, goal, self.scene(), cfg['turning_radius_m'],
                          cfg['allow_reverse'], cfg['planning_timeout_s'])
        self.last_plan = plan
        return plan

    def choose_next_face(self, pose, now):
        for face in (1, 3, 2):
            if face in self.visited:
                continue
            self.visited.add(face)
            plan = self.make_plan(pose, face)
            if plan.reason in {'ok', 'already_at_goal'}:
                self.face, self.route = face, list(plan.moves)
                self.transition('ORBIT', f'visiting_face_{face}', now)
                return
        self.transition('FAILED', 'no_more_reachable_faces', now)

    def next_move(self, pose, now):
        if self.state in {'IDLE', 'STOPPED', 'COMPLETE', 'FAILED', 'STAGE_COMPLETE'}:
            return None
        if now-self.started_at > self.config['mission_timeout_s']:
            self.stop('mission_timeout', now)
            return None
        if self.state == 'ACQUIRE':
            if now-self.state_since > self.config['vision']['acquisition_timeout_s']:
                self.transition('FAILED', 'marker_not_found_in_view', now)
            return None
        cfg = self.config['motion']
        if now-self.last_move_finished < cfg['settle_s']:
            return None
        if self.state == 'APPROACH':
            if now-self.marker_stamp > self.config['vision']['max_frame_age_s']:
                self.reason = 'waiting_for_fresh_marker'
                if now-self.marker_stamp > self.config['vision']['lost_marker_timeout_s']:
                    self.stop('marker_lost_during_approach', now)
                return None
            if self.marker_stamp <= self.last_move_finished+cfg['settle_s']:
                return None
            goal = self.obstacle.viewpoint(0, cfg['view_standoff_m'])
            if at_goal(pose, goal):
                self.visited.add(0)
                if self.config.get('mission_stage') == 'approach':
                    self.transition('STAGE_COMPLETE', 'approach_test_complete', now)
                    return None
                self.choose_next_face(pose, now)
                return None
            plan = self.make_plan(pose, 0)
            if not plan.moves:
                self.transition('FAILED', 'approach_'+plan.reason, now)
                return None
            return plan.moves[0]
        if self.state == 'ORBIT':
            goal = self.obstacle.viewpoint(self.face, cfg['view_standoff_m'])
            if at_goal(pose, goal):
                self.inspection_after = now+cfg['settle_s']
                self.symbol_confirmation.reset()
                self.transition('INSPECT', f'inspecting_face_{self.face}', now)
                return None
            if not self.route:
                plan = self.make_plan(pose, self.face)
                if not plan.moves:
                    self.choose_next_face(pose, now)
                    return None
                self.route = list(plan.moves)
            move = self.route.pop(0)
            if not self.scene().clear_move(pose, move, cfg['turning_radius_m']):
                self.stop('next_primitive_not_clear', now)
                return None
            return move
        if self.state == 'INSPECT' and now-self.inspection_after > self.config['recognition']['face_timeout_s']:
            if self.config.get('mission_stage') == 'one_face':
                self.transition('STAGE_COMPLETE', 'one_face_test_unconfirmed', now)
            else:
                self.choose_next_face(pose, now)
        return None

    def move_finished(self, success, status, now):
        self.last_move_finished = now
        if not success:
            self.stop('motion_failed:'+status, now)

    def status(self):
        return {'state': self.state, 'reason': self.reason, 'face': self.face,
                'visited_faces': sorted(self.visited),
                'obstacle': asdict(self.obstacle) if self.obstacle else None,
                'result': self.result}
