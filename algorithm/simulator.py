#!/usr/bin/env python3
"""Interactive 2D Pygame Arena Simulator for SC2079 MDP.

Controls:
- Left Click + Drag: Move obstacle to new position
- Right Click on Obstacle: Cycle target image face (N -> E -> S -> W)
- SPACE: Play / Pause trajectory animation
- R: Reset robot to Start Zone
- C: Clear / Regenerate path plan
- Number keys (1-9): Select obstacle
- ESC / Q: Exit simulator
"""

from __future__ import annotations

import math
import os
import random
import sys
import threading
from typing import Dict, List, Optional, Tuple

import pygame

from arena import (
    ARENA_SIZE_CM,
    OBSTACLE_SIZE_CM,
    ROBOT_H_CM,
    ROBOT_W_CM,
    START_ZONE_CM,
    TURNING_RADIUS_CM,
    Config,
    Obstacle,
    default_arena,
)
from planner import (
    DEFAULT_VIEW_DIST_CM,
    FullMissionPlan,
    PlanLeg,
    compute_vantage_pose,
    plan_mission,
)

# Color Palette (Dark High-Contrast Theme)
BG_COLOR = (24, 26, 32)
GRID_COLOR = (42, 46, 58)
GRID_MAJOR = (58, 64, 82)
WALL_COLOR = (120, 130, 150)
START_ZONE_COLOR = (35, 75, 45)
START_BORDER_COLOR = (50, 180, 80)

OBSTACLE_COLOR = (60, 65, 80)
OBSTACLE_BORDER = (180, 190, 210)
FACE_TARGET_COLOR = (245, 60, 60)      # Bright Red for target image face
FACE_NON_TARGET = (100, 110, 130)

PATH_REEDS_COLOR = (0, 220, 140)       # Emerald Green for Reeds-Shepp curve
PATH_ASTAR_COLOR = (255, 140, 30)       # Vivid Orange for A* fallback trajectory
PATH_COLOR = PATH_REEDS_COLOR          # Default alias
PATH_REVERSE_COLOR = (255, 170, 0)     # Amber for reverse moves
VANTAGE_MARKER_COLOR = (0, 180, 255)   # Cyan for camera stopping point

ROBOT_BODY_COLOR = (0, 150, 255)
ROBOT_BORDER_COLOR = (255, 255, 255)
ROBOT_HEAD_COLOR = (255, 220, 0)

SIDEBAR_BG = (18, 20, 24)
TEXT_WHITE = (230, 235, 245)
TEXT_GRAY = (140, 150, 170)
TEXT_ACCENT = (0, 220, 140)
HIGHLIGHT_COLOR = (255, 200, 0)


class ArenaSimulator:
    def __init__(self, width: int = 1200, height: int = 820) -> None:
        pygame.init()
        pygame.display.set_caption("SC2079 MDP — Autonomous Path Planner Simulator")
        self.screen = pygame.display.set_mode((width, height))
        self.clock = pygame.time.Clock()

        # Enhanced Typography System
        self.font_xs = pygame.font.SysFont("DejaVu Sans, Arial, sans-serif", 13)
        self.font_sm = pygame.font.SysFont("DejaVu Sans, Arial, sans-serif", 15)
        self.font_sm_bold = pygame.font.SysFont("DejaVu Sans, Arial, sans-serif", 15, bold=True)
        self.font_md = pygame.font.SysFont("DejaVu Sans, Arial, sans-serif", 17, bold=True)
        self.font_lg = pygame.font.SysFont("DejaVu Sans, Arial, sans-serif", 21, bold=True)
        self.font_xl = pygame.font.SysFont("DejaVu Sans, Arial, sans-serif", 25, bold=True)
        self.font_mono = pygame.font.SysFont("DejaVu Sans Mono, Consolas, Courier New, monospace", 14)
        self.font_obs = pygame.font.SysFont("DejaVu Sans, Arial, sans-serif", 15, bold=True)

        # Arena Display Dimensions
        self.arena_px = 720  # 720x720 pixels for 200x200 cm arena
        self.arena_origin = (45, 50)  # Top-left corner of arena box
        self.scale = self.arena_px / ARENA_SIZE_CM  # 3.6 px per cm

        # Initialize Arena State
        arena_dict = default_arena()
        self.obstacles: List[Obstacle] = arena_dict["obstacles"]
        self.start_pose = Config(20.0, 20.0, math.pi / 2.0)
        self.plan: Optional[FullMissionPlan] = None

        # Dragging & Interaction
        self.dragging_idx: Optional[int] = None
        self.drag_offset_x = 0.0
        self.drag_offset_y = 0.0

        # Animation State
        self.animating = False
        self.anim_idx = 0
        self.current_pose = (self.start_pose.x, self.start_pose.y, self.start_pose.theta)
        self.anim_speed = 3  # Poses per frame

        # Pre-rendered Static UI Surfaces
        self.start_zone_label = self.font_sm_bold.render("START ZONE", True, START_BORDER_COLOR)
        self.title_surface = self.font_xl.render("SC2079 Path Planner", True, TEXT_WHITE)
        self.cmd_header_surface = self.font_md.render("--- 5-BYTE UART COMMANDS ---", True, TEXT_ACCENT)
        self.status_anim_surface = self.font_md.render("Status: ANIMATING", True, TEXT_ACCENT)
        self.status_ready_surface = self.font_md.render("Status: PAUSED / READY", True, HIGHLIGHT_COLOR)

        # Algorithm Color Legend Surfaces
        self.legend_header_surface = self.font_md.render("--- PATH ALGORITHM KEY ---", True, TEXT_GRAY)
        self.legend_reeds_surface = self.font_sm_bold.render("Reeds-Shepp (Direct)", True, PATH_REEDS_COLOR)
        self.legend_astar_surface = self.font_sm_bold.render("A* Search (Fallback)", True, PATH_ASTAR_COLOR)

        help_header = self.font_md.render("--- CONTROLS ---", True, TEXT_GRAY)
        help_box = [
            "Left Drag: Move Obstacle",
            "Right Click: Change Face (N/E/S/W)",
            "Middle Click: Delete Obstacle",
            "Shift+Click / A: Add Obstacle",
            "P: Sample Random Permutation",
            "C / ENTER: Compute Optimal Path",
            "SPACE: Play/Pause | R: Reset",
        ]
        self.help_header_surface = help_header
        self.help_surfaces = [self.font_sm.render(line, True, TEXT_GRAY) for line in help_box]
        self.obs_label_cache: Dict[int, pygame.Surface] = {}

        # Async Recomputation State
        self.is_computing = False
        self.needs_replan = False
        self.is_random_perm = False
        self.status_computing_surface = self.font_md.render("Status: COMPUTING PLAN...", True, (255, 100, 100))
        self.status_stale_surface = self.font_md.render("Status: MODIFIED (Press C to Plan)", True, (255, 180, 50))
        self.status_random_surface = self.font_md.render("Status: RANDOM PERM (Sub-optimal)", True, (80, 210, 255))

        # Interactive Button Geometry & Surfaces
        self.btn_rect = pygame.Rect(800, 168, 360, 36)
        self.btn_add_rect = pygame.Rect(800, 210, 175, 30)
        self.btn_del_rect = pygame.Rect(985, 210, 175, 30)
        self.btn_perm_rect = pygame.Rect(800, 248, 360, 30)

        self.btn_idle_text = self.font_md.render("COMPUTE OPTIMAL PATH (C)", True, TEXT_WHITE)
        self.btn_stale_text = self.font_md.render("RECOMPUTE PATH (C)", True, TEXT_WHITE)
        self.btn_busy_text = self.font_md.render("COMPUTING...", True, TEXT_WHITE)
        self.btn_add_text = self.font_sm_bold.render("+ ADD (A)", True, TEXT_WHITE)
        self.btn_del_text = self.font_sm_bold.render("- REMOVE (D)", True, TEXT_WHITE)
        self.btn_perm_text = self.font_sm_bold.render("SAMPLE RANDOM PERMUTATION (P)", True, TEXT_WHITE)

        # Cached Plan Text Surfaces
        self.cached_metrics_surfaces: List[pygame.Surface] = []
        self.cached_cmd_surfaces: List[Tuple[pygame.Surface, pygame.Surface]] = []
        self._pending_plan: Optional[Tuple[FullMissionPlan, int]] = None
        self._plan_lock = threading.Lock()

        # Initial synchronous plan
        try:
            self.plan = plan_mission(self.obstacles, start_pose=self.start_pose)
            if self.plan and self.plan.all_poses:
                self.current_pose = self.plan.all_poses[0]
            self._update_text_surfaces(self.plan, math.factorial(len(self.obstacles)))
        except Exception as e:
            print(f"[Simulator Warning] Initial plan failed: {e}")

    def _update_text_surfaces(self, plan: FullMissionPlan, total_perms: int) -> None:
        """Render cached UI text surfaces on the main Pygame thread."""
        num_rs = sum(1 for leg in plan.legs if getattr(leg, "method", "reeds_shepp") == "reeds_shepp")
        num_astar = sum(1 for leg in plan.legs if getattr(leg, "method", "reeds_shepp") in ("astar", "hybrid_astar", "astar_fallback"))

        metrics = [
            self.font_sm_bold.render(f"Total Distance: {plan.total_distance_cm:.1f} cm", True, TEXT_WHITE),
            self.font_sm.render(f"TSP Orders: {total_perms} evaluated", True, TEXT_WHITE),
            self.font_sm.render(f"Obstacles Visited: {len(plan.legs)} ({num_rs} Reeds, {num_astar} A*)", True, TEXT_WHITE),
        ]
        cmds = []
        for i, leg in enumerate(plan.legs):
            is_astar = getattr(leg, "method", "reeds_shepp") in ("astar", "hybrid_astar", "astar_fallback")
            method_tag = "[A* Fallback]" if is_astar else "[Reeds-Shepp]"
            header_color = PATH_ASTAR_COLOR if is_astar else HIGHLIGHT_COLOR
            t_surf = self.font_sm_bold.render(f"Leg {i+1} -> Obs {leg.obstacle_id} ({leg.target_face}) {method_tag}:", True, header_color)
            c_str = " ".join(leg.raw_strings)
            c_surf = self.font_mono.render(c_str, True, TEXT_WHITE)
            cmds.append((t_surf, c_surf))

        self.cached_metrics_surfaces = metrics
        self.cached_cmd_surfaces = cmds

    def get_obs_label(self, obs_id: int) -> pygame.Surface:
        if obs_id not in self.obs_label_cache:
            self.obs_label_cache[obs_id] = self.font_obs.render(f"O{obs_id}", True, TEXT_WHITE)
        return self.obs_label_cache[obs_id]

    def add_obstacle(self, wx: Optional[float] = None, wy: Optional[float] = None) -> None:
        """Add a new obstacle (up to maximum 8)."""
        if len(self.obstacles) >= 8:
            return
        next_id = max([o.id for o in self.obstacles], default=0) + 1
        if wx is None or wy is None:
            preset_spots = [
                (100, 100), (140, 100), (100, 140), (140, 140),
                (60, 100), (100, 60), (150, 70), (70, 150)
            ]
            chosen = (100, 100)
            for sx, sy in preset_spots:
                if not any(abs(o.x - sx) < 15 and abs(o.y - sy) < 15 for o in self.obstacles):
                    chosen = (sx, sy)
                    break
            wx, wy = chosen

        self.obstacles.append(Obstacle(id=next_id, x=int(wx), y=int(wy), face="N"))
        self.needs_replan = True

    def remove_obstacle(self, idx: Optional[int] = None) -> None:
        """Remove an obstacle (keeping at least 1)."""
        if len(self.obstacles) <= 1:
            return
        if idx is not None and 0 <= idx < len(self.obstacles):
            self.obstacles.pop(idx)
        else:
            self.obstacles.pop()
        if self.dragging_idx is not None and self.dragging_idx >= len(self.obstacles):
            self.dragging_idx = None
        self.needs_replan = True

    def sample_random_permutation(self) -> None:
        """Sample a random visiting permutation to demonstrate candidate TSP orders."""
        if len(self.obstacles) <= 1:
            return
        indices = list(range(len(self.obstacles)))
        for _ in range(5):
            random.shuffle(indices)
            if indices != list(range(len(self.obstacles))):
                break
        self.is_random_perm = True
        self.recompute_plan(forced_order=indices)

    def recompute_plan(self, forced_order: Optional[List[int]] = None) -> None:
        """Re-run the TSP and Reeds-Shepp trajectory optimizer in a background thread."""
        if self.is_computing:
            return  # Previous compute still active
        self.is_computing = True
        self.needs_replan = False
        if forced_order is None:
            self.is_random_perm = False

        # Copy obstacle state for thread safety
        obs_snapshot = [
            Obstacle(id=ob.id, x=ob.x, y=ob.y, image_id=ob.image_id, face=ob.face, label=ob.label)
            for ob in self.obstacles
        ]

        def _worker():
            try:
                plan = plan_mission(obs_snapshot, start_pose=self.start_pose, forced_order=forced_order)
                total_perms = math.factorial(len(obs_snapshot))
                with self._plan_lock:
                    self._pending_plan = (plan, total_perms)
            except Exception as e:
                print(f"[Simulator Warning] Recompute plan failed: {e}")
            finally:
                self.is_computing = False

        threading.Thread(target=_worker, daemon=True).start()

    # Coordinate Conversions (World cm <-> Screen px)
    def world_to_screen(self, wx: float, wy: float) -> Tuple[int, int]:
        """Convert arena (x, y in cm, origin bottom-left) to Pygame screen coordinates."""
        sx = int(self.arena_origin[0] + wx * self.scale)
        sy = int(self.arena_origin[1] + (ARENA_SIZE_CM - wy) * self.scale)
        return sx, sy

    def screen_to_world(self, sx: int, sy: int) -> Tuple[float, float]:
        """Convert Pygame screen coordinates to arena (x, y in cm)."""
        wx = (sx - self.arena_origin[0]) / self.scale
        wy = ARENA_SIZE_CM - (sy - self.arena_origin[1]) / self.scale
        return max(10.0, min(190.0, wx)), max(10.0, min(190.0, wy))

    # Event Handling
    def handle_events(self) -> bool:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False

            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    return False
                elif event.key == pygame.K_SPACE:
                    self.animating = not self.animating
                elif event.key == pygame.K_r:
                    self.animating = False
                    self.anim_idx = 0
                    self.current_pose = (self.start_pose.x, self.start_pose.y, self.start_pose.theta)
                elif event.key in (pygame.K_c, pygame.K_RETURN, pygame.K_KP_ENTER):
                    self.recompute_plan()
                elif event.key in (pygame.K_p, pygame.K_s):
                    self.sample_random_permutation()
                elif event.key in (pygame.K_a, pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                    self.add_obstacle()
                elif event.key in (pygame.K_d, pygame.K_MINUS, pygame.K_DELETE, pygame.K_BACKSPACE, pygame.K_KP_MINUS):
                    self.remove_obstacle()

            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = event.pos

                # Check if UI Buttons were clicked
                if event.button == 1:
                    if self.btn_rect.collidepoint(mx, my):
                        self.recompute_plan()
                        continue
                    elif self.btn_add_rect.collidepoint(mx, my):
                        self.add_obstacle()
                        continue
                    elif self.btn_del_rect.collidepoint(mx, my):
                        self.remove_obstacle()
                        continue
                    elif self.btn_perm_rect.collidepoint(mx, my):
                        self.sample_random_permutation()
                        continue

                wx, wy = self.screen_to_world(mx, my)

                # Check if an obstacle was clicked
                clicked_idx = None
                for i, ob in enumerate(self.obstacles):
                    if abs(ob.x - wx) <= 8 and abs(ob.y - wy) <= 8:
                        clicked_idx = i
                        break
                    if abs(ob.x - wx) <= 8 and abs(ob.y - wy) <= 8:
                        clicked_idx = i
                        break

                if clicked_idx is not None:
                    if event.button == 1:  # Left Click -> Start Drag
                        self.dragging_idx = clicked_idx
                        self.drag_offset_x = self.obstacles[clicked_idx].x - wx
                        self.drag_offset_y = self.obstacles[clicked_idx].y - wy
                    elif event.button == 2:  # Middle Click -> Delete Obstacle
                        self.remove_obstacle(clicked_idx)
                    elif event.button == 3:  # Right Click -> Cycle Face (N -> E -> S -> W)
                        faces = ["N", "E", "S", "W"]
                        curr_face = self.obstacles[clicked_idx].face or "N"
                        next_face = faces[(faces.index(curr_face) + 1) % len(faces)]
                        self.obstacles[clicked_idx].face = next_face
                        self.needs_replan = True  # Mark modified, wait for manual compute
                else:
                    # Clicked on empty space inside arena
                    if event.button == 1 and (pygame.key.get_mods() & pygame.KMOD_SHIFT):
                        ax, ay = self.arena_origin
                        sz = self.arena_px
                        if ax <= mx <= ax + sz and ay <= my <= ay + sz:
                            self.add_obstacle(wx, wy)

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1 and self.dragging_idx is not None:
                    self.dragging_idx = None
                    self.needs_replan = True  # Mark modified, wait for manual compute

            elif event.type == pygame.MOUSEMOTION:
                if self.dragging_idx is not None:
                    mx, my = event.pos
                    wx, wy = self.screen_to_world(mx, my)
                    nx = max(10.0, min(190.0, wx + self.drag_offset_x))
                    ny = max(10.0, min(190.0, wy + self.drag_offset_y))
                    self.obstacles[self.dragging_idx].x = round(nx)
                    self.obstacles[self.dragging_idx].y = round(ny)

        return True

    def update(self) -> None:
        with self._plan_lock:
            if self._pending_plan is not None:
                plan, total_perms = self._pending_plan
                self._pending_plan = None
                self.plan = plan
                self.anim_idx = 0
                if self.plan and self.plan.all_poses:
                    self.current_pose = self.plan.all_poses[0]
                self._update_text_surfaces(self.plan, total_perms)

        if self.animating and self.plan and self.plan.all_poses:
            self.anim_idx = min(len(self.plan.all_poses) - 1, self.anim_idx + self.anim_speed)
            self.current_pose = self.plan.all_poses[self.anim_idx]
            if self.anim_idx >= len(self.plan.all_poses) - 1:
                self.animating = False

    # Drawing Routines
    def draw_arena(self) -> None:
        ax, ay = self.arena_origin
        sz = self.arena_px

        # Background & Border
        pygame.draw.rect(self.screen, BG_COLOR, (ax, ay, sz, sz))
        pygame.draw.rect(self.screen, WALL_COLOR, (ax - 2, ay - 2, sz + 4, sz + 4), 2)

        # 10cm Grid Lines
        for g in range(10, ARENA_SIZE_CM, 10):
            sx, sy = self.world_to_screen(g, g)
            color = GRID_MAJOR if g % 50 == 0 else GRID_COLOR
            # Vertical
            pygame.draw.line(self.screen, color, (sx, ay), (sx, ay + sz), 1)
            # Horizontal
            pygame.draw.line(self.screen, color, (ax, sy), (ax + sz, sy), 1)

        # Start Zone (40x40 cm)
        start_sx, start_sy = self.world_to_screen(0, START_ZONE_CM)
        sz_px = int(START_ZONE_CM * self.scale)
        pygame.draw.rect(self.screen, START_ZONE_COLOR, (start_sx, start_sy, sz_px, sz_px))
        pygame.draw.rect(self.screen, START_BORDER_COLOR, (start_sx, start_sy, sz_px, sz_px), 2)
        self.screen.blit(self.start_zone_label, (start_sx + 8, start_sy + sz_px - 22))

    def draw_trajectory(self) -> None:
        if not self.plan or not self.plan.legs:
            return

        for leg in self.plan.legs:
            is_astar = getattr(leg, "method", "reeds_shepp") in ("astar", "hybrid_astar", "astar_fallback")
            leg_color = PATH_ASTAR_COLOR if is_astar else PATH_REEDS_COLOR

            if len(leg.poses) > 1:
                points = [self.world_to_screen(p[0], p[1]) for p in leg.poses]
                pygame.draw.lines(self.screen, leg_color, False, points, 3)

            # Draw Vantage Pose
            vx, vy = self.world_to_screen(leg.vantage_pose.x, leg.vantage_pose.y)
            pygame.draw.circle(self.screen, VANTAGE_MARKER_COLOR, (vx, vy), 5)
            # Direction indicator
            th = leg.vantage_pose.theta
            ex = vx + int(12 * math.cos(th))
            ey = vy - int(12 * math.sin(th))
            pygame.draw.line(self.screen, VANTAGE_MARKER_COLOR, (vx, vy), (ex, ey), 2)

    def draw_obstacles(self) -> None:
        half_sz = (OBSTACLE_SIZE_CM / 2.0) * self.scale
        bar_th = 4  # Target face indicator thickness

        for i, ob in enumerate(self.obstacles):
            cx, cy = self.world_to_screen(ob.x, ob.y)
            rect = pygame.Rect(cx - half_sz, cy - half_sz, half_sz * 2, half_sz * 2)

            # Draw obstacle body
            pygame.draw.rect(self.screen, OBSTACLE_COLOR, rect)
            pygame.draw.rect(self.screen, OBSTACLE_BORDER, rect, 2)

            # Draw colored target face
            face = (ob.face or "N").upper()
            if face == "N":
                pygame.draw.line(self.screen, FACE_TARGET_COLOR, (rect.left, rect.top), (rect.right, rect.top), bar_th)
            elif face == "S":
                pygame.draw.line(self.screen, FACE_TARGET_COLOR, (rect.left, rect.bottom), (rect.right, rect.bottom), bar_th)
            elif face == "E":
                pygame.draw.line(self.screen, FACE_TARGET_COLOR, (rect.right, rect.top), (rect.right, rect.bottom), bar_th)
            elif face == "W":
                pygame.draw.line(self.screen, FACE_TARGET_COLOR, (rect.left, rect.top), (rect.left, rect.bottom), bar_th)

            # Cached Obstacle Label
            lbl = self.get_obs_label(ob.id)
            lbl_rect = lbl.get_rect(center=(cx, cy))
            self.screen.blit(lbl, lbl_rect)

    def draw_robot(self, x: float, y: float, theta: float) -> None:
        c, s = math.cos(theta), math.sin(theta)
        hh = ROBOT_H_CM / 2.0  # longitudinal (along heading)
        hw = ROBOT_W_CM / 2.0  # transverse (side-to-side)

        # 4 corners in world coordinates
        corners_world = [
            (x + hh * c - hw * s, y + hh * s + hw * c),
            (x + hh * c + hw * s, y + hh * s - hw * c),
            (x - hh * c + hw * s, y - hh * s - hw * c),
            (x - hh * c - hw * s, y - hh * s + hw * c),
        ]
        poly = [self.world_to_screen(cx, cy) for cx, cy in corners_world]

        pygame.draw.polygon(self.screen, ROBOT_BODY_COLOR, poly)
        pygame.draw.polygon(self.screen, ROBOT_BORDER_COLOR, poly, 2)

        # Front heading marker (Yellow Dot at front bumper)
        fx_w = x + (hh + 3.0) * c
        fy_w = y + (hh + 3.0) * s
        fx, fy = self.world_to_screen(fx_w, fy_w)
        pygame.draw.circle(self.screen, ROBOT_HEAD_COLOR, (fx, fy), 4)

    def draw_sidebar(self) -> None:
        sb_x = 800
        sb_y = 20

        # Cached Title & Status
        self.screen.blit(self.title_surface, (sb_x, sb_y))
        if self.is_computing:
            status_surf = self.status_computing_surface
        elif self.is_random_perm:
            status_surf = self.status_random_surface
        elif self.needs_replan:
            status_surf = self.status_stale_surface
        elif self.animating:
            status_surf = self.status_anim_surface
        else:
            status_surf = self.status_ready_surface
        self.screen.blit(status_surf, (sb_x, sb_y + 32))

        # Cached Metrics Panel
        for i, m_surf in enumerate(self.cached_metrics_surfaces):
            self.screen.blit(m_surf, (sb_x, sb_y + 64 + i * 22))

        # Interactive Compute Button
        mouse_pos = pygame.mouse.get_pos()
        hover = self.btn_rect.collidepoint(mouse_pos)

        if self.is_computing:
            btn_bg = (50, 50, 60)
            btn_border = (80, 80, 90)
            btn_txt = self.btn_busy_text
        elif self.needs_replan:
            btn_bg = (0, 160, 95) if hover else (0, 130, 75)
            btn_border = (50, 230, 150)
            btn_txt = self.btn_stale_text
        else:
            btn_bg = (55, 68, 92) if hover else (40, 50, 70)
            btn_border = (90, 115, 155)
            btn_txt = self.btn_idle_text

        pygame.draw.rect(self.screen, btn_bg, self.btn_rect, border_radius=6)
        pygame.draw.rect(self.screen, btn_border, self.btn_rect, 2, border_radius=6)
        txt_rect = btn_txt.get_rect(center=self.btn_rect.center)
        self.screen.blit(btn_txt, txt_rect)

        # Interactive Add & Remove Obstacle Buttons
        hover_add = self.btn_add_rect.collidepoint(mouse_pos)
        hover_del = self.btn_del_rect.collidepoint(mouse_pos)

        add_bg = (40, 75, 55) if hover_add else (30, 58, 42)
        add_border = (60, 160, 95)
        del_bg = (75, 40, 45) if hover_del else (58, 30, 35)
        del_border = (160, 60, 70)

        pygame.draw.rect(self.screen, add_bg, self.btn_add_rect, border_radius=5)
        pygame.draw.rect(self.screen, add_border, self.btn_add_rect, 1, border_radius=5)
        self.screen.blit(self.btn_add_text, self.btn_add_text.get_rect(center=self.btn_add_rect.center))

        pygame.draw.rect(self.screen, del_bg, self.btn_del_rect, border_radius=5)
        pygame.draw.rect(self.screen, del_border, self.btn_del_rect, 1, border_radius=5)
        self.screen.blit(self.btn_del_text, self.btn_del_text.get_rect(center=self.btn_del_rect.center))

        # Interactive Sample Random Permutation Button
        hover_perm = self.btn_perm_rect.collidepoint(mouse_pos)
        perm_bg = (55, 68, 92) if hover_perm else (38, 48, 68)
        perm_border = (100, 140, 200)
        pygame.draw.rect(self.screen, perm_bg, self.btn_perm_rect, border_radius=5)
        pygame.draw.rect(self.screen, perm_border, self.btn_perm_rect, 1, border_radius=5)
        self.screen.blit(self.btn_perm_text, self.btn_perm_text.get_rect(center=self.btn_perm_rect.center))

        # Cached Instructions / Help
        help_y = 290
        self.screen.blit(self.help_header_surface, (sb_x, help_y))
        help_y += 22
        for h_surf in self.help_surfaces:
            self.screen.blit(h_surf, (sb_x, help_y))
            help_y += 18

        # Algorithm Color Legend
        legend_y = help_y + 8
        self.screen.blit(self.legend_header_surface, (sb_x, legend_y))
        legend_y += 22

        # Reeds-Shepp indicator line
        pygame.draw.line(self.screen, PATH_REEDS_COLOR, (sb_x + 4, legend_y + 8), (sb_x + 28, legend_y + 8), 3)
        self.screen.blit(self.legend_reeds_surface, (sb_x + 36, legend_y))
        legend_y += 20

        # A* indicator line
        pygame.draw.line(self.screen, PATH_ASTAR_COLOR, (sb_x + 4, legend_y + 8), (sb_x + 28, legend_y + 8), 3)
        self.screen.blit(self.legend_astar_surface, (sb_x + 36, legend_y))
        legend_y += 24

        # Cached Command Stream Output
        if self.cached_cmd_surfaces:
            cmd_y = legend_y + 4
            self.screen.blit(self.cmd_header_surface, (sb_x, cmd_y))
            cmd_y += 22

            for leg_title_surf, cmd_str_surf in self.cached_cmd_surfaces:
                self.screen.blit(leg_title_surf, (sb_x, cmd_y))
                cmd_y += 18
                self.screen.blit(cmd_str_surf, (sb_x + 10, cmd_y))
                cmd_y += 22

    def run(self) -> None:
        running = True
        while running:
            running = self.handle_events()
            self.update()

            self.screen.fill(SIDEBAR_BG)
            self.draw_arena()
            self.draw_trajectory()
            self.draw_obstacles()
            self.draw_robot(*self.current_pose)
            self.draw_sidebar()

            pygame.display.flip()
            self.clock.tick(60)

        pygame.quit()


if __name__ == "__main__":
    sim = ArenaSimulator()
    sim.run()
