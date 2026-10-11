import math
from pathlib import Path
import tempfile
import unittest
import numpy as np
from rail_stage1.servo_control import (target_pixel, point_jacobian, ibvs_command,
                                     AlignmentGate, Localization, StraightDrive)
from rail_stage1.motion_lease import MotionLease


def camera():
    angle = math.radians(30)
    r = np.array([[0., -math.sin(angle), math.cos(angle)],
                  [-1., 0., 0.], [0., -math.cos(angle), -math.sin(angle)]])
    return r, np.array([.44, .015, .65])


def rz(angle):
    return np.array([[math.cos(angle), -math.sin(angle), 0.],
                     [math.sin(angle), math.cos(angle), 0.], [0., 0., 1.]])


class ServoTests(unittest.TestCase):
    def test_target_and_bounds(self):
        np.testing.assert_allclose(target_pixel(848, 480, 80), [423.5, 399])
        for offset in (-1, 479, 1000):
            with self.assertRaises(ValueError):
                target_pixel(848, 480, offset)

    def test_jacobian_against_actual_camera_motion(self):
        r, origin = camera()
        base_point = np.array([1.5, -.1, .04])
        p = r.T @ (base_point-origin)
        analytical = point_jacobian(p, r, origin)
        numerical = []
        dt = 1e-7
        for moved in (base_point-np.array([dt, 0., 0.]), rz(-dt) @ base_point):
            new = r.T @ (moved-origin)
            numerical.append((new[:2]/new[2]-p[:2]/p[2])/dt)
        np.testing.assert_allclose(analytical, np.column_stack(numerical), atol=1e-6)

    def test_saturation_and_image_error_reduction(self):
        r, origin = camera()
        p = r.T @ (np.array([1.5, -.1, .04])-origin)
        desired = np.array([0., .26])
        cmd = ibvs_command(p, desired, r, origin, .5, .08, .04, .25)
        self.assertLessEqual(abs(cmd[1]), .25)
        self.assertLessEqual(cmd[0], .08)
        self.assertGreaterEqual(cmd[0], -.04)
        error = p[:2]/p[2]-desired
        self.assertLess(error @ (point_jacobian(p, r, origin) @ cmd), 0.)

    def test_closed_loop_converges_to_target(self):
        r, origin = camera()
        world_feature = np.array([1.4, .12, .04])
        robot = np.zeros(3)
        desired = (target_pixel(848, 480, 80)-[424., 240.])/606.
        for _ in range(1200):
            in_base = rz(-robot[2]) @ (world_feature-np.r_[robot[:2], 0.])
            p = r.T @ (in_base-origin)
            error_px = (p[:2]/p[2]-desired)*606.
            if np.max(np.abs(error_px)) <= 8.:
                break
            vx, wz = ibvs_command(p, desired, r, origin, .5, .08, .04, .25)
            robot[:2] += .05*vx*np.array([math.cos(robot[2]), math.sin(robot[2])])
            robot[2] += .05*wz
        self.assertLessEqual(np.max(np.abs(error_px)), 8.)

    def test_invalid_depth_and_singular_geometry(self):
        r, origin = camera()
        for z in (0., -.1, float('nan')):
            with self.assertRaises(ValueError):
                ibvs_command([.1, .1, z], [0., 0.], r, origin, .5, .08, .04, .25)
        # Camera along the body x axis, point on that axis: forward motion
        # changes depth only, so two image coordinates cannot be controlled.
        r = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
        with self.assertRaisesRegex(ValueError, 'conditioned'):
            ibvs_command([0., 0., 1.], [0., 0.], r, np.zeros(3), .5, .08, .04, .25)

    def test_alignment_requires_distinct_frames_and_dwell(self):
        gate = AlignmentGate(8., 5, .6)
        for _ in range(20):
            self.assertFalse(gate.observe([1., 1.], 1, 0.))
        for stamp in (2, 3, 4):
            self.assertFalse(gate.observe([1., 1.], stamp, stamp*.1))
        self.assertTrue(gate.observe([1., 1.], 5, .8))
        self.assertFalse(gate.observe([20., 1.], 6, .9))
        self.assertEqual(gate.count, 0)


def pose(x=0., y=0., yaw=0., t=0., frame='odom', received=None):
    return Localization(x, y, yaw, frame, int((100+t)*1e9), t if received is None else received)


class DriveTests(unittest.TestCase):
    def test_constant_speed_and_distance(self):
        controller = StraightDrive(pose(), 1.5, .12, 0.)
        self.assertEqual(controller.deadline, 25.)
        for i in range(1, 126):
            t = i*.1
            step = controller.step(pose(x=.12*t, t=t), t)
            self.assertEqual(step.vx, .12 if i < 125 else 0.)
        self.assertEqual(step.state, 'SUCCEEDED')
        self.assertAlmostEqual(step.distance, 1.5)

    def test_timeout_at_exact_double_nominal_time(self):
        controller = StraightDrive(pose(), 1.5, .12, 0.)
        step = controller.step(pose(t=24.9), 24.9)
        self.assertEqual(step.vx, .12)
        step = controller.step(pose(t=25.), 25.)
        self.assertEqual(step.state, 'FAILED')
        self.assertEqual(step.vx, 0.)
        self.assertIn('timeout', step.reason)
        self.assertEqual(controller.step(pose(x=1.5, t=26.), 26.).state, 'FAILED')

    def test_drift_and_reverse_do_not_count_as_forward(self):
        controller = StraightDrive(pose(), 1.5, .12, 0.)
        self.assertEqual(controller.step(pose(y=1.5, t=2.), 2.).distance, 0.)
        self.assertEqual(controller.step(pose(x=-1., y=1.5, t=4.), 4.).distance, 0.)

    def test_stale_pose_jump_frame_change_and_cancel_stop(self):
        cases = [(pose(), 1., False, 'stale'),
                 (pose(x=1.5, t=.1), .1, False, 'jump'),
                 (pose(t=.1, frame='map'), .1, False, 'frame'),
                 (pose(yaw=math.pi/3, t=.1), .1, False, 'heading'),
                 (pose(t=.1), .1, True, 'canceled')]
        for sample, now, canceled, reason in cases:
            controller = StraightDrive(pose(), 1.5, .12, 0.)
            step = controller.step(sample, now, canceled)
            self.assertEqual(step.vx, 0.)
            self.assertIn(reason, step.reason)

    def test_distance_in_rotated_localization_frame(self):
        controller = StraightDrive(pose(x=2., y=3., yaw=math.pi/2), 1.5, .12, 0.)
        step = controller.step(pose(x=2., y=4.5, yaw=math.pi/2, t=12.5), 12.5)
        self.assertEqual(step.state, 'SUCCEEDED')

    def test_invalid_goal(self):
        for distance, speed in [(0., .1), (1.5, 0.), (-1., .1), (1., float('nan'))]:
            with self.assertRaises(ValueError):
                StraightDrive(pose(), distance, speed, 0.)


class LeaseTests(unittest.TestCase):
    def test_mutually_exclusive_and_release(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root)/'motion.lock'
            approach, entry = MotionLease(path), MotionLease(path)
            self.assertTrue(approach.acquire())
            self.assertFalse(entry.acquire())
            approach.release()
            self.assertTrue(entry.acquire())
            self.assertFalse(approach.acquire())
            entry.release()


if __name__ == '__main__':
    unittest.main()
