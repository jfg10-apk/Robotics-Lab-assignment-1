"""Run without pytest: python -m unittest discover -s tests -p test_ball.py -v."""
import sys
import unittest
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from controllers import SwingController
from env import HumanoidEnv
from main import _advance_frame


class BallTests(unittest.TestCase):
    def setUp(self):
        self.env = HumanoidEnv()
        self.controller = SwingController(self.env)
        self.locked = self.env.get_locked_humanoid_joints(self.controller.joint_names)

    def test_ball_is_free_and_only_undriven_humanoid_joints_are_locked(self):
        self.assertNotIn("bola_livre", self.locked)
        self.assertIn("root", self.locked)
        self.assertNotIn("shoulder1_right", self.locked)
        self.assertEqual(self.env.model.joint("bola_livre").type[0],
                         mujoco.mjtJoint.mjJNT_FREE)
        self.assertAlmostEqual(self.env.model.body("bola").mass[0], 0.04593)

    def test_ball_stays_on_tee_before_impact(self):
        initial = self.env.data.body("bola").xpos.copy()
        for _ in range(round(1 / self.env.timestep)):
            _advance_frame(self.env, self.controller, "physics", self.locked)
        np.testing.assert_allclose(self.env.data.body("bola").xpos, initial, atol=1e-4)

    def test_swing_hits_ball_and_reset_restores_it(self):
        env = self.env
        initial = env.data.body("bola").xpos.copy()
        pair = {env.model.geom("bola").id, env.model.geom("cabeca_taco").id}
        first_hit = None
        peak_force = 0.0
        for _ in range(round(6 / env.timestep)):
            _advance_frame(env, self.controller, "physics", self.locked)
            for index in range(env.data.ncon):
                contact = env.data.contact[index]
                if {contact.geom1, contact.geom2} == pair:
                    first_hit = env.get_time() if first_hit is None else first_hit
                    force = np.zeros(6)
                    mujoco.mj_contactForce(env.model, env.data, index, force)
                    peak_force = max(peak_force, force[0])
        self.assertIsNotNone(first_hit)
        self.assertTrue(1.7 < first_hit < 2.1)
        self.assertGreater(peak_force, 0)
        self.assertGreater(np.linalg.norm(env.data.body("bola").xpos[:2] - initial[:2]), 0.5)
        self.assertAlmostEqual(env.data.body("bola").xpos[2], 0.02135, delta=0.001)
        self.assertTrue(np.isfinite(env.data.qpos).all())
        self.assertTrue(np.isfinite(env.data.qvel).all())
        self.assertEqual(sum(w.number for w in env.data.warning), 0)
        env.reset()
        np.testing.assert_allclose(env.data.body("bola").xpos, initial)
        np.testing.assert_allclose(env.data.joint("bola_livre").qvel, 0)
        self.assertEqual(env.get_time(), 0)

    def test_ball_falls_under_gravity_away_from_tee(self):
        env = self.env
        env.data.joint("bola_livre").qpos[:3] = [2, 0, 1]
        mujoco.mj_forward(env.model, env.data)
        for _ in range(round(0.1 / env.timestep)):
            _advance_frame(env, self.controller, "physics", self.locked)
        self.assertAlmostEqual(env.data.joint("bola_livre").qpos[2],
                               1 - 0.5 * 9.81 * 0.1**2, delta=0.003)


if __name__ == "__main__":
    unittest.main()
