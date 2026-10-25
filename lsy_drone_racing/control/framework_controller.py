"""Educational state-interface baseline for LSY drone racing.

Uses only public observations. This is a development scaffold, not a validated
competition controller or a safety-certified real-drone controller.
"""

from __future__ import annotations

import heapq
import json
import logging
import os
from pathlib import Path

import numpy as np

from lsy_drone_racing.control import Controller

LOGGER = logging.getLogger(__name__)


def gate_normal(quaternion: np.ndarray, direction: int) -> np.ndarray:
    """Rotate local +x into world coordinates; input order is xyzw."""
    q = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if norm < 1e-8 or not np.isfinite(norm):
        raise ValueError("Invalid gate quaternion")
    x, y, z, w = q / norm
    return direction * np.array([1 - 2 * (y*y + z*z), 2*(x*y + z*w), 2*(x*z - y*w)])


def clear_segment(a: np.ndarray, b: np.ndarray, poles: np.ndarray, radius: float) -> bool:
    """Check a 2D segment against inflated poles, conservatively at any altitude."""
    if len(poles) == 0:
        return True
    delta = b[:2] - a[:2]
    denom = float(delta @ delta)
    t = np.clip((poles[:, :2] - a[:2]) @ delta / max(denom, 1e-12), 0, 1)
    nearest = a[:2] + t[:, None] * delta
    return bool(np.all(np.linalg.norm(poles[:, :2] - nearest, axis=1) >= radius))


def pole_route(start: np.ndarray, goal: np.ndarray, poles: np.ndarray,
               low: np.ndarray, high: np.ndarray, radius: float) -> list[np.ndarray]:
    """Visibility graph around pole circles; raises if no candidate route exists.

    This planner does NOT model gate frames. It is intentionally replaceable.
    """
    if clear_segment(start, goal, poles, radius):
        return [goal.copy()]
    nodes = [start.copy(), goal.copy()]
    for pole in poles:
        for angle in np.linspace(0, 2*np.pi, 16, endpoint=False):
            node = np.array([pole[0] + 1.3*radius*np.cos(angle),
                             pole[1] + 1.3*radius*np.sin(angle), goal[2]])
            if np.all(node >= low) and np.all(node <= high):
                if clear_segment(node, node, poles, radius):
                    nodes.append(node)
    distances = {0: 0.0}
    parents = {}
    queue = [(0.0, 0)]
    while queue:
        cost, current = heapq.heappop(queue)
        if cost > distances[current]:
            continue
        if current == 1:
            indices = [1]
            while indices[-1] != 0:
                indices.append(parents[indices[-1]])
            return [nodes[i].copy() for i in reversed(indices[:-1])]
        for nxt, node in enumerate(nodes):
            if nxt == current or not clear_segment(nodes[current], node, poles, radius):
                continue
            candidate = cost + float(np.linalg.norm(node - nodes[current]))
            if candidate < distances.get(nxt, np.inf):
                distances[nxt], parents[nxt] = candidate, current
                heapq.heappush(queue, (candidate, nxt))
    raise RuntimeError("No pole-clear route: hold and inspect geometry")


def quintic(start: np.ndarray, goal: np.ndarray, elapsed: float,
             duration: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Rest-to-rest straight segment with continuous position/velocity/acceleration."""
    u = float(np.clip(elapsed / duration, 0, 1))
    s = 10*u**3 - 15*u**4 + 6*u**5
    ds = (30*u**2 - 60*u**3 + 30*u**4) / duration
    dds = (60*u - 180*u**2 + 120*u**3) / duration**2
    delta = goal - start
    return start + s*delta, ds*delta, dds*delta


class FrameworkController(Controller):
    """Takeoff, approach, cross, exit; count progress only from environment feedback."""

    SPEED = 0.65  # Peak reference speed, m/s. Tune only after simulation validation.
    ACCELERATION = 1.3  # Peak reference acceleration, m/s^2.
    APPROACH_DISTANCE = 0.45
    EXIT_DISTANCE = 0.30
    POLE_CLEARANCE = 0.22  # Inflation heuristic, not a certified vehicle clearance.
    ARRIVAL_TOLERANCE = 0.10
    ARRIVAL_SPEED = 0.20
    SEGMENT_TIMEOUT = 6.0  # Extra settling time after reference reaches its endpoint.

    def __init__(self, obs: dict, info: dict, config: dict):
        """Initialize from official observations and public race configuration."""
        super().__init__(obs, info, config)
        env = config["env"]
        if env["control_mode"] != "state":
            raise ValueError("FrameworkController requires env.control_mode = 'state'")
        self.dt = 1.0 / float(env["freq"])
        limits = env["track"]["safety_limits"]
        self.low = np.asarray(limits["pos_limit_low"], dtype=float) + [0.08, 0.08, 0.08]
        self.high = np.asarray(limits["pos_limit_high"], dtype=float) - [0.08, 0.08, 0.08]
        self.tick = 0
        self.phase = "takeoff"
        self.progress = -1
        self.finished = False
        self.blocked = False
        self.failure_reason = None
        self.route = []
        self.segment = None
        self.geometry = None
        self.latest_obs = obs
        self.events = []
        self.hold = np.asarray(obs["pos"], dtype=float).copy()
        target = self.hold.copy()
        target[2] = max(0.45, target[2])
        self._plan(obs, target, avoid_poles=True)

    def _event(self, name: str, **data: object) -> None:
        self.events.append({"time": self.tick*self.dt, "event": name, **data})

    def _block(self, obs: dict, reason: str) -> None:
        self.blocked = True
        self.failure_reason = reason
        self.hold = np.asarray(obs["pos"], dtype=float).copy()
        self._event("blocked", reason=reason)
        LOGGER.warning("Framework controller holding: %s", reason)

    def _plan(self, obs: dict, target: np.ndarray, avoid_poles: bool) -> None:
        """Replan at settled waypoints; command starts at observed position."""
        start = np.asarray(obs["pos"], dtype=float).copy()
        if np.any(target < self.low) or np.any(target > self.high):
            self._block(obs, "Target outside inset track bounds")
            return
        poles = np.asarray(obs["obstacles_pos"], dtype=float).reshape(-1, 3)
        # Allow the observed takeoff start near ground; intermediate nodes use inset bounds.
        try:
            if avoid_poles:
                self.route = pole_route(start, target, poles, self.low, self.high,
                                       self.POLE_CLEARANCE)
            elif clear_segment(start, target, poles, self.POLE_CLEARANCE):
                self.route = [target.copy()]
            else:
                raise RuntimeError("Pole conflicts with straight gate crossing")
        except RuntimeError as error:
            self._block(obs, str(error))
            return
        self._next_segment(start)

    def _next_segment(self, start: np.ndarray) -> None:
        goal = self.route.pop(0)
        distance = float(np.linalg.norm(goal - start))
        # Quintic peak derivative constants: max s' = 1.875, max |s''| < 5.774.
        duration = max(0.6, 1.875*distance/self.SPEED,
                       np.sqrt(5.774*distance/self.ACCELERATION))
        self.segment = (start.copy(), goal, self.tick, duration)
        self._event("segment", phase=self.phase, target=goal.tolist())

    def _gate(self, obs: dict) -> tuple[np.ndarray, np.ndarray]:
        idx = int(np.asarray(obs["gate_sequence"])[self.progress])
        direction = int(np.asarray(obs["gate_sequence_direction"])[self.progress])
        if direction not in (-1, 1):
            raise ValueError("Gate direction must be +1 or -1")
        center = np.asarray(obs["gates_pos"], dtype=float)[idx]
        normal = gate_normal(np.asarray(obs["gates_quat"])[idx], direction)
        return center, normal

    def _approach(self, obs: dict) -> None:
        self.phase = "approach"
        center, normal = self._gate(obs)
        self.geometry = np.concatenate((center, normal))
        self._plan(obs, center - self.APPROACH_DISTANCE*normal, avoid_poles=True)

    def compute_control(self, obs: dict, info: dict | None = None) -> np.ndarray:
        """Return the current 16D xyzw full-state reference."""
        self.latest_obs = obs
        passed = int(obs["n_gates_passed"])
        sequence = np.asarray(obs["gate_sequence"])
        if passed >= len(sequence):
            self.finished = True
            self.phase = "done"
        if not self.blocked and not self.finished:
            if self.phase not in ("takeoff", "cross") and passed != self.progress:
                self.progress = passed
                self._event("gate_progress", passed=passed)
                self._approach(obs)
            if self.segment is not None:
                start, goal, begin, duration = self.segment
                elapsed = (self.tick-begin)*self.dt
                settled = (np.linalg.norm(np.asarray(obs["pos"])-goal) < self.ARRIVAL_TOLERANCE
                           and np.linalg.norm(obs["vel"]) < self.ARRIVAL_SPEED)
                if elapsed >= duration and settled:
                    if self.route:
                        self._next_segment(np.asarray(obs["pos"], dtype=float))
                    elif self.phase == "takeoff":
                        self.progress = passed
                        self._approach(obs)
                    elif self.phase == "approach":
                        center, normal = self._gate(obs)
                        current_geometry = np.concatenate((center, normal))
                        if np.linalg.norm(current_geometry-self.geometry) > 0.025:
                            # New gate observations are used after approach settles, before crossing.
                            self._approach(obs)
                        else:
                            self.phase = "cross"
                            self._plan(obs, center+self.EXIT_DISTANCE*normal, avoid_poles=False)
                    elif self.phase == "cross":
                        # Never invent a pass based on elapsed time or waypoint proximity.
                        self.progress = passed
                        self._approach(obs)
                elif elapsed > duration+self.SEGMENT_TIMEOUT:
                    self._block(obs, "Reference tracking did not settle before timeout")
        if self.blocked or self.finished:
            pos = self.hold if self.blocked else np.asarray(obs["pos"], dtype=float)
            vel, acc = np.zeros(3), np.zeros(3)
        else:
            start, goal, begin, duration = self.segment
            pos, vel, acc = quintic(start, goal, (self.tick-begin)*self.dt, duration)
        return np.concatenate((pos, vel, acc, [0., 0., 0., 1.], np.zeros(3))).astype(np.float32)

    def step_callback(self, action: np.ndarray, obs: dict, reward: float,
                      terminated: bool, truncated: bool, info: dict) -> bool:
        """Advance the internal clock; only official gate progress indicates success."""
        self.tick += 1
        self.latest_obs = obs
        self.finished = int(obs["n_gates_passed"]) >= len(obs["gate_sequence"])
        return self.finished

    def episode_callback(self) -> None:
        """Optionally append diagnostics outside the competition's metrics file.

        Set ADR_LOG_FILE to an explicit JSONL path to enable local logging.
        """
        destination = os.getenv("ADR_LOG_FILE")
        if destination:
            record = {"steps": self.tick, "success": self.finished,
                      "gates_passed": int(self.latest_obs["n_gates_passed"]),
                      "failure_reason": self.failure_reason, "events": self.events}
            try:
                with Path(destination).open("a", encoding="utf-8") as stream:
                    stream.write(json.dumps(record, ensure_ascii=False)+"\n")
            except OSError as error:
                LOGGER.warning("Could not write optional diagnostic log: %s", error)

    def episode_reset(self) -> None:
        """Official sim constructs a fresh controller per episode; no global state is used."""

