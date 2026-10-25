"""Contract and kinematic tests. These do not replace Crazyflow physics evaluation.

Usage: python check_framework.py --repo PATH_TO_OFFICIAL_REPO
Only NumPy and the Python standard library are needed for these checks.
"""

import argparse
import importlib.util
import inspect
import sys
import types
import unittest
from pathlib import Path

import numpy as np


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


parser = argparse.ArgumentParser()
parser.add_argument("--repo", required=True, type=Path)
args, remaining = parser.parse_known_args()
base = load(args.repo/"lsy_drone_racing/control/controller.py", "official_base")
# Isolate the real base class from package __init__, which imports the full simulator.
# No simulator, environment or flight physics is mocked as if it were validated here.
package = types.ModuleType("lsy_drone_racing")
control = types.ModuleType("lsy_drone_racing.control")
control.Controller = base.Controller
sys.modules["lsy_drone_racing"] = package
sys.modules["lsy_drone_racing.control"] = control
module = load(Path(__file__).parent/"lsy_drone_racing/control/framework_controller.py",
              "starter_controller")


def fixture(direction=1):
    obs = {"pos": np.array([-1., 0., 0.]), "vel": np.zeros(3),
           "n_gates_passed": 0, "gate_sequence": np.array([0, 0]),
           "gate_sequence_direction": np.array([direction, -direction]),
           "gates_pos": np.array([[0., 0., 0.65]]),
           "gates_quat": np.array([[0., 0., 0., 1.]]),
           "obstacles_pos": np.empty((0, 3))}
    if direction < 0:
        obs["pos"][0] = 1.
    cfg = {"env": {"freq": 50, "control_mode": "state", "track": {"safety_limits": {
        "pos_limit_low": [-3., -3., -0.01], "pos_limit_high": [3., 3., 2.]}}}}
    return obs, cfg


class FrameworkChecks(unittest.TestCase):
    def test_loader_and_interface(self):
        classes = [v for _, v in inspect.getmembers(module, inspect.isclass)
                   if issubclass(v, base.Controller) and v.__module__ == module.__name__]
        self.assertEqual(classes, [module.FrameworkController])
        for method in ["compute_control", "step_callback", "episode_callback", "episode_reset"]:
            self.assertEqual(list(inspect.signature(getattr(base.Controller, method)).parameters),
                             list(inspect.signature(getattr(classes[0], method)).parameters))
        obs, cfg = fixture()
        action = classes[0](obs, {}, cfg).compute_control(obs)
        self.assertEqual(action.shape, (16,))
        self.assertEqual(action.dtype, np.float32)
        self.assertTrue(np.isfinite(action).all())
        self.assertAlmostEqual(np.linalg.norm(action[9:13]), 1.)

    def test_quintic_endpoints_and_derivatives(self):
        start, goal = np.zeros(3), np.array([1., 2., 3.])
        for elapsed, expected in [(0, start), (2, goal), (3, goal)]:
            p, v, a = module.quintic(start, goal, elapsed, 2.)
            np.testing.assert_allclose(p, expected)
            np.testing.assert_allclose(v, 0, atol=1e-10)
            np.testing.assert_allclose(a, 0, atol=1e-10)
        t, h = .8, 1e-4
        p0, v0, a0 = module.quintic(start, goal, t, 2.)
        pm, vm, _ = module.quintic(start, goal, t-h, 2.)
        pp, vp, _ = module.quintic(start, goal, t+h, 2.)
        np.testing.assert_allclose((pp-pm)/(2*h), v0, rtol=1e-6)
        np.testing.assert_allclose((vp-vm)/(2*h), a0, rtol=1e-6)

    def test_normals(self):
        q = np.array([0, 0, np.sin(np.pi/4), np.cos(np.pi/4)])
        np.testing.assert_allclose(module.gate_normal(q, 1), [0, 1, 0], atol=1e-8)
        np.testing.assert_allclose(module.gate_normal(q, -1), [0, -1, 0], atol=1e-8)

    def test_pole_detour(self):
        a, b = np.array([-1., 0., .6]), np.array([1., 0., .6])
        poles = np.array([[0., 0., 1.5]])
        route = module.pole_route(a, b, poles, np.array([-2., -2., .1]),
                                  np.array([2., 2., 2.]), .22)
        self.assertGreater(len(route), 1)
        for point in route:
            self.assertTrue(module.clear_segment(a, point, poles, .22))
            a = point
        np.testing.assert_allclose(route[-1], b)

    def test_gate_count_is_never_invented(self):
        obs, cfg = fixture()
        ctl = module.FrameworkController(obs, {}, cfg)
        for _ in range(1800):
            action = ctl.compute_control(obs)
            obs["pos"], obs["vel"] = action[:3].copy(), action[3:6].copy()
            ctl.step_callback(action, obs, 0., False, False, {})
        self.assertFalse(ctl.finished)
        self.assertEqual(obs["n_gates_passed"], 0)

    def test_forward_reverse_repeat_with_observation_update(self):
        for direction in [1, -1]:
            obs, cfg = fixture(direction)
            ctl = module.FrameworkController(obs, {}, cfg)
            updated = False
            for _ in range(2200):
                last = obs["pos"].copy()
                if ctl.phase == "approach" and not updated:
                    obs["gates_pos"][0, 1] = .12
                    updated = True
                action = ctl.compute_control(obs)
                obs["pos"], obs["vel"] = action[:3].copy(), action[3:6].copy()
                passed = obs["n_gates_passed"]
                if passed < 2:
                    sign = obs["gate_sequence_direction"][passed]
                    crossed = last[0]*sign < 0 < obs["pos"][0]*sign
                    if crossed and np.linalg.norm(obs["pos"][1:]-obs["gates_pos"][0, 1:]) < .18:
                        obs["n_gates_passed"] += 1
                if ctl.step_callback(action, obs, 0., False, False, {}):
                    break
            self.assertTrue(ctl.finished, ctl.events[-5:])
            self.assertEqual(obs["n_gates_passed"], 2)

    def test_invalid_target_holds(self):
        obs, cfg = fixture()
        ctl = module.FrameworkController(obs, {}, cfg)
        ctl._plan(obs, np.array([100., 0., 1.]), True)
        self.assertTrue(ctl.blocked)
        np.testing.assert_allclose(ctl.compute_control(obs)[:3], obs["pos"])


unittest.main(argv=[sys.argv[0], *remaining])
