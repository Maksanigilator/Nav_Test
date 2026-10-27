import math
import unittest
import numpy as np
import cv2
from rail_stage1.servo_control import (point_jacobian, omni_ibvs_command, rail_heading,
                                     AlignmentGate, VelocitySlew, target_pixel)
from test_servo import camera, rz
from test_stage1 import scene, CFG
from rail_stage1.geometry import detect


class OmniServoTests(unittest.TestCase):
    def test_visual_axis_from_rotated_rails_has_correct_heading_sign(self):
        for yaw in (-.2, .2):
            mask, depth, k, distortion, r, origin = scene()
            mask[:] = 0
            half = CFG['rail_gauge']/2
            entry = np.array([1.4, 0., .04])
            pairs = [([0., -half, 0.], [1.4, -half, 0.]),
                     ([0., half, 0.], [1.4, half, 0.]),
                     ([0., -half, 0.], [0., half, 0.])]
            for a, b in pairs:
                base = np.array([rz(yaw) @ a+entry, rz(yaw) @ b+entry])
                pixel = cv2.projectPoints((base-origin) @ r, np.zeros(3), np.zeros(3), k, distortion)[0]
                cv2.line(mask, tuple(pixel[0, 0].astype(int)), tuple(pixel[1, 0].astype(int)), 255, 12)
            _, axis, _, _ = detect(mask, depth, k, distortion, r, origin, CFG)
            self.assertAlmostEqual(rail_heading(axis), yaw, delta=.035)

    def test_three_axis_jacobian_matches_camera_motion(self):
        r, origin = camera()
        base = np.array([1.4, .12, .04])
        p = r.T @ (base-origin)
        dt = 1e-7
        numerical = []
        for new_base in (base-[dt, 0., 0.], base-[0., dt, 0.], rz(-dt) @ base):
            new = r.T @ (new_base-origin)
            numerical.append((new[:2]/new[2]-p[:2]/p[2])/dt)
        np.testing.assert_allclose(point_jacobian(p, r, origin, omni=True),
                                   np.column_stack(numerical), atol=1e-6)
        heading = rail_heading([math.cos(.2), math.sin(.2), 0.])
        moved = rail_heading(rz(-dt) @ [math.cos(.2), math.sin(.2), 0.])
        self.assertAlmostEqual((moved-heading)/dt, -1., places=6)

    def command(self, p, desired, angle, minimums=(.04, .04, .10)):
        r, origin = camera()
        return omni_ibvs_command(p, desired, angle, r, origin, .8, .8,
                                 [.16, .14, .35], minimums, max_reverse=.12)

    def test_point_centered_but_yaw_wrong_requires_rotation(self):
        r, origin = camera()
        p = r.T @ (np.array([1.5, .1, .04])-origin)
        command = self.command(p, p[:2]/p[2], .25, (0., 0., 0.))
        self.assertGreater(command[2], 0.)
        np.testing.assert_allclose(point_jacobian(p, r, origin, omni=True) @ command, [0., 0.], atol=1e-10)
        gate = AlignmentGate(12., 3, .3, 3.)
        for i in range(10):
            self.assertFalse(gate.observe([0., 0.], i, i*.1, .25))
        self.assertFalse(gate.observe([0., 0.], 11, 1.1, 0.))
        self.assertFalse(gate.observe([0., 0.], 12, 1.3, 0.))
        self.assertTrue(gate.observe([0., 0.], 13, 1.5, 0.))

    def test_lateral_error_is_corrected_without_unnecessary_yaw(self):
        r, origin = camera()
        p = r.T @ (np.array([1.5, .15, .04])-origin)
        desired_point = r.T @ (np.array([1.5, 0., .04])-origin)
        command = self.command(p, desired_point[:2]/desired_point[2], 0.)
        self.assertGreater(command[1], 0.)
        self.assertEqual(command[2], 0.)

    def test_velocity_bounds_minimums_and_descent(self):
        r, origin = camera()
        rng = np.random.default_rng(42)
        for _ in range(200):
            p = r.T @ (np.r_[rng.uniform(1., 2.), rng.uniform(-.3, .3), .04]-origin)
            desired = p[:2]/p[2]+rng.uniform(-.15, .15, 2)
            yaw = rng.uniform(-.6, .6)
            command = self.command(p, desired, yaw)
            self.assertTrue(-.120001 <= command[0] <= .160001)
            self.assertLessEqual(abs(command[1]), .140001)
            self.assertLessEqual(abs(command[2]), .350001)
            jacobian = np.vstack((point_jacobian(p, r, origin, omni=True), [0., 0., -1.]))
            error = np.r_[p[:2]/p[2]-desired, yaw]
            self.assertLess(error @ (jacobian @ command), 0.)
        p = r.T @ (np.array([1.5, .1, .04])-origin)
        command = self.command(p, p[:2]/p[2]+[.001, 0.], 0.)
        self.assertGreaterEqual(abs(command[1]), .04)
        np.testing.assert_allclose(self.command(p, p[:2]/p[2], 0.), [0., 0., 0.], atol=1e-8)

    def test_joint_alignment_closed_loop_with_stiction_and_slew(self):
        r, origin = camera()
        feature_world = np.array([1.4, .12, .04])
        desired = (target_pixel(848, 480, 80)-[424., 240.])/606.
        for yaw in (-.35, .35):
            robot = np.array([0., 0., yaw])
            gate = AlignmentGate(12., 3, .3, 3.)
            slew = VelocitySlew(.35, .8)
            converged = False
            for i in range(1200):
                p = r.T @ (rz(-robot[2]) @ (feature_world-np.r_[robot[:2], 0.])-origin)
                error = (p[:2]/p[2]-desired)*606.
                heading = -robot[2]
                if gate.observe(error, i, i*.05, heading):
                    converged = True
                    break
                target = p[:2]/p[2] if np.max(np.abs(error)) <= 12. else desired
                heading_task = 0. if abs(heading) <= math.radians(3.) else heading
                raw = self.command(p, target, heading_task)
                cmd = slew.step(raw, i*.05)
                cmd[:2][np.abs(cmd[:2]) < .035] = 0.  # heavy-base stiction model
                if abs(cmd[2]) < .085:
                    cmd[2] = 0.
                robot[:2] += .05*(rz(robot[2]) @ np.r_[cmd[:2], 0.])[:2]
                robot[2] += .05*cmd[2]
            self.assertTrue(converged, (yaw, error, math.degrees(heading)))

    def test_slew_and_bad_settings(self):
        slew = VelocitySlew(.35, .8)
        np.testing.assert_equal(slew.step([.16, .14, .35], 0.), [0., 0., 0.])
        np.testing.assert_allclose(slew.step([.16, .14, .35], .05), [.0175, .0175, .04])
        slew.reset()
        np.testing.assert_equal(slew.velocity, [0., 0., 0.])
        for direction in ([0., 0., 1.], [float('nan'), 1., 0.]):
            with self.assertRaises(ValueError):
                rail_heading(direction)


if __name__ == '__main__':
    unittest.main()
