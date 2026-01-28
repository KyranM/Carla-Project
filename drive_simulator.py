"""
CARLA Manual Drive + Calm NPC Traffic

What this script does:
- Spawns an ego vehicle you drive with keyboard (smooth controls).
- Spawns NPC vehicles + pedestrians controlled by Traffic Manager.
- Calms NPC behavior (slower speeds, bigger following distance, fewer lane changes).
- Optionally detunes NPC physics (less snappy accel/turn/stop).
- Uses a chase spectator camera + a small Pygame HUD.

Controls:
- W/Up: throttle
- S/Down: brake
- A/D or Left/Right: steer
- SPACE: emergency brake (handbrake)
- R: toggle reverse
- ESC or Q: quit
"""

from __future__ import annotations

import math
import time
import random
from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple

import carla
import pygame


# =============================================================================
# Configuration (single source of truth)
# =============================================================================

@dataclass(frozen=True)
class CameraConfig:
    height: float = 2.2
    distance: float = 7.0
    pitch_deg: float = -12.0


@dataclass(frozen=True)
class EgoControlConfig:
    throttle_ramp_up: float = 2.2
    throttle_ramp_down: float = 3.5
    brake_ramp_up: float = 5.0
    brake_ramp_down: float = 6.0
    steer_rate: float = 1.8
    steer_speed_sensitivity: float = 0.070
    engine_brake_strength: float = 0.12


@dataclass(frozen=True)
class NpcTrafficConfig:
    vehicles: int = 50
    pedestrians: int = 25

    # Traffic Manager speed control (positive => slower than speed limit)
    global_speed_diff_pct: float = 35.0
    vehicle_speed_diff_min: float = 25.0
    vehicle_speed_diff_max: float = 45.0

    # Spacing (meters)
    global_follow_distance_m: float = 5.0
    vehicle_follow_distance_m: float = 5.0

    # Lane changes (lower => less twitchy)
    enable_auto_lane_change: bool = True
    lane_change_left_pct: float = 2.0
    lane_change_right_pct: float = 2.0

    # Optional: detune physics so they accelerate/turn/stop less aggressively
    detune_physics: bool = True
    torque_scale: float = 0.75
    brake_scale: float = 0.85
    steer_scale: float = 0.90

    # Traffic Manager performance/continuity options (version dependent)
    hybrid_physics: bool = True
    hybrid_physics_radius_m: float = 120.0
    respawn_dormant_vehicles: bool = True


@dataclass(frozen=True)
class DisplayConfig:
    width: int = 900
    height: int = 540
    fps_limit: int = 60
    font_name: str = "consolas"
    font_size: int = 20


@dataclass(frozen=True)
class Config:
    town: str = "Town01"
    fixed_fps: int = 60
    ego_vehicle_filter: str = "vehicle.tesla.model3"

    camera: CameraConfig = CameraConfig()
    ego: EgoControlConfig = EgoControlConfig()
    npc: NpcTrafficConfig = NpcTrafficConfig()
    display: DisplayConfig = DisplayConfig()


CFG = Config()


# =============================================================================
# Small utilities
# =============================================================================

def clamp(x: float, lo: float, hi: float) -> float:
    """Clamp x into [lo, hi]."""
    return max(lo, min(hi, x))


def try_call(fn, *args, **kwargs) -> bool:
    """
    Try calling a CARLA API that might not exist / might fail on some versions.
    Returns True if it worked, False if it threw.
    """
    try:
        fn(*args, **kwargs)
        return True
    except Exception:
        return False


def get_speed_kmh(vehicle: carla.Vehicle) -> float:
    """Vehicle speed in km/h from CARLA's velocity vector (m/s)."""
    v = vehicle.get_velocity()
    return 3.6 * math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def is_four_wheeled_vehicle(bp: carla.ActorBlueprint) -> bool:
    """Filter out bikes/motorcycles when possible."""
    if not bp.has_attribute("number_of_wheels"):
        return True
    try:
        return int(bp.get_attribute("number_of_wheels")) == 4
    except Exception:
        return True


# =============================================================================
# Camera
# =============================================================================

class ChaseCamera:
    """Moves spectator camera behind ego vehicle each tick (simple chase camera)."""

    def __init__(self, world: carla.World, vehicle: carla.Vehicle, cfg: CameraConfig):
        self._world = world
        self._vehicle = vehicle
        self._cfg = cfg

    def tick(self) -> None:
        tf = self._vehicle.get_transform()
        fwd = tf.get_forward_vector()

        # Build camera location relative to vehicle
        loc = carla.Location(tf.location.x, tf.location.y, tf.location.z)
        loc.z += self._cfg.height
        loc.x -= fwd.x * self._cfg.distance
        loc.y -= fwd.y * self._cfg.distance

        rot = carla.Rotation(
            pitch=self._cfg.pitch_deg,
            yaw=tf.rotation.yaw,
            roll=0.0,
        )

        self._world.get_spectator().set_transform(carla.Transform(loc, rot))


# =============================================================================
# Ego vehicle: smooth keyboard controller
# =============================================================================

@dataclass
class EgoState:
    throttle: float = 0.0
    brake: float = 0.0
    steer: float = 0.0
    reverse: bool = False


@dataclass(frozen=True)
class KeyBindings:
    throttle: Tuple[int, int] = (pygame.K_w, pygame.K_UP)
    brake: Tuple[int, int] = (pygame.K_s, pygame.K_DOWN)
    left: Tuple[int, int] = (pygame.K_a, pygame.K_LEFT)
    right: Tuple[int, int] = (pygame.K_d, pygame.K_RIGHT)
    emergency: int = pygame.K_SPACE
    reverse_toggle: int = pygame.K_r
    quit_keys: Tuple[int, int] = (pygame.K_ESCAPE, pygame.K_q)


class SmoothKeyboardController:
    """
    Converts keyboard presses to smooth VehicleControl values.
    - Ramp throttle/brake up/down over time
    - Speed-based steering limit
    - Optional engine braking to reduce endless coasting feel
    """

    def __init__(self, cfg: EgoControlConfig, keys: KeyBindings = KeyBindings()):
        self._cfg = cfg
        self._keys = keys
        self._state = EgoState()
        self._reverse_prev_down = False

    @staticmethod
    def _any(keys: pygame.key.ScancodeWrapper, codes: Iterable[int]) -> bool:
        return any(keys[c] for c in codes)

    def should_quit(self, keys: pygame.key.ScancodeWrapper) -> bool:
        return self._any(keys, self._keys.quit_keys)

    def tick(
        self,
        vehicle: carla.Vehicle,
        dt: float,
        keys: pygame.key.ScancodeWrapper,
    ) -> Tuple[carla.VehicleControl, float]:
        speed_kmh = get_speed_kmh(vehicle)

        # --- Reverse toggle (edge-triggered) ---
        reverse_down = keys[self._keys.reverse_toggle]
        if reverse_down and not self._reverse_prev_down:
            self._state.reverse = not self._state.reverse
        self._reverse_prev_down = reverse_down

        # --- Read inputs ---
        want_throttle = self._any(keys, self._keys.throttle)
        want_brake = self._any(keys, self._keys.brake)
        want_left = self._any(keys, self._keys.left)
        want_right = self._any(keys, self._keys.right)
        emergency = keys[self._keys.emergency]

        # Emergency brake overrides everything
        if emergency:
            self._state.throttle = 0.0
            self._state.brake = 1.0
            ctrl = carla.VehicleControl(
                throttle=0.0,
                brake=1.0,
                steer=self._state.steer,
                hand_brake=True,
                reverse=self._state.reverse,
            )
            return ctrl, speed_kmh

        # --- Steering target ---
        target_steer = 0.0
        if want_left and not want_right:
            target_steer = -1.0
        elif want_right and not want_left:
            target_steer = 1.0

        # Speed-based steering reduction (less steering authority at high speed)
        max_steer = 1.0 / (1.0 + self._cfg.steer_speed_sensitivity * speed_kmh)
        target_steer *= max_steer

        # --- Throttle/brake targets (binary keyboard => 0 or 1) ---
        target_throttle = 1.0 if want_throttle else 0.0
        target_brake = 1.0 if want_brake else 0.0

        # --- Smooth throttle ---
        self._state.throttle += (
            self._cfg.throttle_ramp_up if target_throttle > self._state.throttle else -self._cfg.throttle_ramp_down
        ) * dt
        self._state.throttle = clamp(self._state.throttle, 0.0, 1.0)

        # --- Smooth brake ---
        self._state.brake += (
            self._cfg.brake_ramp_up if target_brake > self._state.brake else -self._cfg.brake_ramp_down
        ) * dt
        self._state.brake = clamp(self._state.brake, 0.0, 1.0)

        # --- Engine braking (adds mild decel when coasting) ---
        if self._state.throttle < 0.02 and self._state.brake < 0.02 and speed_kmh > 2.0:
            coast_brake = self._cfg.engine_brake_strength * clamp(speed_kmh / 60.0, 0.0, 1.0)
            self._state.brake = max(self._state.brake, coast_brake)

        # --- Smooth steering (rate-limited) ---
        max_delta = self._cfg.steer_rate * dt
        delta = clamp(target_steer - self._state.steer, -max_delta, max_delta)
        self._state.steer = clamp(self._state.steer + delta, -1.0, 1.0)

        ctrl = carla.VehicleControl(
            throttle=self._state.throttle,
            brake=self._state.brake,
            steer=self._state.steer,
            reverse=self._state.reverse,
        )
        return ctrl, speed_kmh


# =============================================================================
# NPC traffic spawning + calming
# =============================================================================

def detune_vehicle_physics(vehicle: carla.Vehicle, cfg: NpcTrafficConfig) -> None:
    """
    Optional: make NPC vehicles less snappy by reducing:
    - engine torque (accel)
    - brake torque (stopping)
    - max steer angle (turning aggressiveness)
    """
    if not cfg.detune_physics:
        return

    try:
        pc = vehicle.get_physics_control()

        # Reduce engine torque curve (slower accel)
        try:
            for p in pc.torque_curve:
                p.y *= float(cfg.torque_scale)
        except Exception:
            pass

        # Reduce braking power & steering angle per wheel
        try:
            for w in pc.wheels:
                w.brake_torque *= float(cfg.brake_scale)
                w.max_steer_angle *= float(cfg.steer_scale)
        except Exception:
            pass

        vehicle.apply_physics_control(pc)
    except Exception:
        # Some CARLA versions/vehicles may fail here; safe to ignore.
        pass


def configure_traffic_manager(tm: carla.TrafficManager, cfg: NpcTrafficConfig) -> None:
    """
    Applies global Traffic Manager behavior:
    - Synchronous mode (required for sync world)
    - Global speed difference (slower/faster than speed limit)
    - Global following distance
    - Hybrid physics + respawning dormant vehicles (if supported)
    """
    tm.set_synchronous_mode(True)

    # Performance/continuity options (version-dependent)
    if cfg.hybrid_physics:
        try_call(tm.set_hybrid_physics_mode, True)
        try_call(tm.set_hybrid_physics_radius, float(cfg.hybrid_physics_radius_m))

    if cfg.respawn_dormant_vehicles:
        try_call(tm.set_respawn_dormant_vehicles, False)

    # Global calmness
    try_call(tm.set_global_distance_to_leading_vehicle, float(cfg.global_follow_distance_m))
    try_call(tm.global_percentage_speed_difference, float(cfg.global_speed_diff_pct))


def apply_per_vehicle_tm_settings(tm: carla.TrafficManager, v: carla.Vehicle, cfg: NpcTrafficConfig) -> None:
    """
    Applies per-vehicle Traffic Manager settings to calm behavior:
    - per-vehicle speed difference randomization
    - per-vehicle following distance
    - lane change frequency / enablement
    - rule obeying (traffic lights / signs)
    """
    # Per-vehicle spacing reduces "race then brake" oscillations
    try_call(tm.distance_to_leading_vehicle, v, float(cfg.vehicle_follow_distance_m))

    # Per-vehicle speed differences make traffic feel more natural
    spd = random.uniform(float(cfg.vehicle_speed_diff_min), float(cfg.vehicle_speed_diff_max))
    try_call(tm.vehicle_percentage_speed_difference, v, spd)

    # Lane change behavior (big contributor to twitchy maneuvers)
    try_call(tm.auto_lane_change, v, bool(cfg.enable_auto_lane_change))
    if cfg.enable_auto_lane_change:
        try_call(tm.random_left_lanechange_percentage, v, float(cfg.lane_change_left_pct))
        try_call(tm.random_right_lanechange_percentage, v, float(cfg.lane_change_right_pct))

    # Strict rule obeying (optional; set to 0 for "normal law-abiding")
    try_call(tm.ignore_lights_percentage, v, 0.0)
    try_call(tm.ignore_signs_percentage, v, 0.0)


def spawn_ego_vehicle(world: carla.World, vehicle_filter: str) -> carla.Vehicle:
    """Spawn ego vehicle (manual controlled)."""
    bp = world.get_blueprint_library().filter(vehicle_filter)[0]
    spawn = world.get_map().get_spawn_points()[0]
    vehicle = world.spawn_actor(bp, spawn)
    vehicle.set_autopilot(False)
    return vehicle


def spawn_npc_vehicles(world: carla.World, tm: carla.TrafficManager, cfg: NpcTrafficConfig) -> List[carla.Vehicle]:
    """Spawn NPC vehicles and configure their Traffic Manager behavior."""
    blueprints = list(world.get_blueprint_library().filter("vehicle.*"))
    spawn_points = list(world.get_map().get_spawn_points())
    random.shuffle(spawn_points)

    vehicles: List[carla.Vehicle] = []
    target = int(cfg.vehicles)
    max_attempts = max(200, target * 6)

    for i in range(min(max_attempts, len(spawn_points) * 10)):
        if len(vehicles) >= target:
            break

        sp = spawn_points[i % len(spawn_points)]
        bp = random.choice(blueprints)

        if not is_four_wheeled_vehicle(bp):
            continue

        v = world.try_spawn_actor(bp, sp)
        if not v:
            continue

        v.set_autopilot(True, tm.get_port())

        apply_per_vehicle_tm_settings(tm, v, cfg)
        detune_vehicle_physics(v, cfg)

        vehicles.append(v)

    return vehicles


def spawn_pedestrians(world: carla.World, count: int) -> Tuple[List[carla.Actor], List[carla.Actor]]:
    """
    Spawn pedestrians + their AI controllers.
    Returns (walkers, controllers).
    """
    walkers: List[carla.Actor] = []
    controllers: List[carla.Actor] = []

    walker_bps = list(world.get_blueprint_library().filter("walker.pedestrian.*"))
    controller_bp = world.get_blueprint_library().find("controller.ai.walker")

    # Oversample candidate locations because some navigation locations can fail
    spawn_transforms: List[carla.Transform] = []
    for _ in range(count * 3):
        loc = world.get_random_location_from_navigation()
        if loc:
            spawn_transforms.append(carla.Transform(loc))
        if len(spawn_transforms) >= count:
            break

    for tf in spawn_transforms:
        w = world.try_spawn_actor(random.choice(walker_bps), tf)
        if not w:
            continue

        c = world.spawn_actor(controller_bp, carla.Transform(), attach_to=w)
        c.start()

        dest = world.get_random_location_from_navigation()
        if dest:
            c.go_to_location(dest)

        c.set_max_speed(1.2)  # calm walking pace
        walkers.append(w)
        controllers.append(c)

    return walkers, controllers


# =============================================================================
# World setup / teardown
# =============================================================================

def ensure_town(client: carla.Client, town: str) -> carla.World:
    world = client.get_world()
    if town in world.get_map().name:
        return world

    # Map load can take a while
    client.set_timeout(30.0)
    world = client.load_world(town)

    # Wait until server is responsive on the new world
    deadline = time.time() + 30.0
    while time.time() < deadline:
        try:
            _ = world.get_map().name
            world.wait_for_tick(1.0)
            break
        except RuntimeError:
            time.sleep(0.2)

    client.set_timeout(5.0)  # restore if you want
    return world


def set_synchronous(world: carla.World, fixed_fps: int) -> carla.WorldSettings:
    """Enable synchronous mode and fixed delta time. Returns old settings to restore later."""
    original = world.get_settings()
    new = world.get_settings()
    new.synchronous_mode = True
    new.fixed_delta_seconds = 1.0 / float(fixed_fps)
    world.apply_settings(new)
    return original


def destroy_actors(world: carla.World, actors: Iterable[Optional[carla.Actor]]) -> None:
    """Destroy actors safely without 'not found' spam."""
    for a in actors:
        if not a:
            continue

        # Re-find by id so we don't destroy stale references
        try:
            live = world.get_actor(a.id)
        except Exception:
            live = None

        if live is None:
            continue

        try:
            live.destroy()
        except Exception:
            pass


def stop_and_destroy_walkers(world: carla.World, walkers, controllers) -> None:
    for c in controllers:
        try:
            c.stop()
        except Exception:
            pass
    destroy_actors(world, controllers)
    destroy_actors(world, walkers)


# =============================================================================
# HUD / Pygame
# =============================================================================

class Hud:
    """Simple Pygame HUD overlay."""

    def __init__(self, cfg: DisplayConfig):
        pygame.init()
        self._cfg = cfg
        self.screen = pygame.display.set_mode((cfg.width, cfg.height))
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(cfg.font_name, cfg.font_size)

    def tick_dt(self) -> float:
        """Returns real dt (seconds) and enforces FPS limit for rendering."""
        return self.clock.tick(self._cfg.fps_limit) / 1000.0

    def fps(self) -> float:
        return self.clock.get_fps()

    def process_quit_events(self) -> bool:
        """Returns True if user closed the window."""
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                return True
        return False

    def draw_lines(self, lines: List[str]) -> None:
        self.screen.fill((15, 15, 15))
        y = 20
        for line in lines:
            if not line:
                y += 10
                continue
            surf = self.font.render(line, True, (230, 230, 230))
            self.screen.blit(surf, (20, y))
            y += 26
        pygame.display.flip()

    def quit(self) -> None:
        pygame.quit()


# =============================================================================
# Main
# =============================================================================

def main() -> None:
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(5.0)

    world = ensure_town(client, CFG.town)
    original_settings = set_synchronous(world, CFG.fixed_fps)

    ego: Optional[carla.Vehicle] = None
    npc_vehicles: List[carla.Vehicle] = []
    walkers: List[carla.Actor] = []
    walker_controllers: List[carla.Actor] = []

    hud: Optional[Hud] = None

    try:
        # --- Spawn ego vehicle ---
        ego = spawn_ego_vehicle(world, CFG.ego_vehicle_filter)

        # --- Configure TM + spawn NPC traffic ---
        tm = client.get_trafficmanager()
        configure_traffic_manager(tm, CFG.npc)

        npc_vehicles = spawn_npc_vehicles(world, tm, CFG.npc)
        walkers, walker_controllers = spawn_pedestrians(world, CFG.npc.pedestrians)

        print(f"Spawned NPC vehicles: {len(npc_vehicles)}/{CFG.npc.vehicles}")
        print(f"Spawned pedestrians: {len(walkers)}/{CFG.npc.pedestrians}")

        # --- Create camera + controller + HUD ---
        camera = ChaseCamera(world, ego, CFG.camera)
        controller = SmoothKeyboardController(CFG.ego)
        hud = Hud(CFG.display)

        # --- Simulation loop ---
        running = True
        while running:
            # Step the simulator deterministically
            world.tick()

            # Real dt for input smoothing + UI
            dt = hud.tick_dt()

            # Window close event
            if hud.process_quit_events():
                break

            keys = pygame.key.get_pressed()
            if controller.should_quit(keys):
                break

            ctrl, speed = controller.tick(ego, dt, keys)
            ego.apply_control(ctrl)
            camera.tick()

            # HUD text
            hud.draw_lines([
                f"FPS: {hud.fps():5.1f}",
                f"Speed: {speed:6.1f} km/h",
                f"Throttle: {ctrl.throttle:.2f}  Brake: {ctrl.brake:.2f}  Steer: {ctrl.steer:.2f}",
                f"Reverse: {'ON' if ctrl.reverse else 'OFF'}",
                "",
                "W/Up = throttle | S/Down = brake",
                "A/D or Left/Right = steer",
                "SPACE = emergency brake | R = reverse | ESC/Q = quit"
            ])

    finally:
        # --- Clean teardown (prevents ghost actors and restores settings) ---
        stop_and_destroy_walkers(world, walkers, walker_controllers)
        destroy_actors(world, npc_vehicles)
        destroy_actors(world, [ego])

        try:
            world.apply_settings(original_settings)
        except Exception:
            pass

        if hud:
            hud.quit()


if __name__ == "__main__":
    main()
