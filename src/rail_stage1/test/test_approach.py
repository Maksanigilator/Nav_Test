import math
import unittest
from rail_stage1.approach_control import ApproachGate
from rail_stage1.servo_control import AlignmentGate


class ApproachTests(unittest.TestCase):
    def test_both_position_and_heading_required(self):
        gate = ApproachGate(.15, 10., .3)
        self.assertFalse(gate.observe(.14, math.radians(11), 1, 0.))
        self.assertFalse(gate.observe(.16, 0., 2, .1))
        self.assertFalse(gate.observe(.14, math.radians(9), 3, .2))
        self.assertTrue(gate.observe(.14, math.radians(9), 4, .51))

    def test_fresh_distinct_poses_and_continuous_dwell(self):
        gate = ApproachGate(.15, 10., .3)
        self.assertFalse(gate.observe(.1, 0., 1, 0.))
        self.assertFalse(gate.observe(.1, 0., 1, 1.))
        self.assertFalse(gate.observe(.2, 0., 2, 1.1))
        self.assertFalse(gate.observe(.1, 0., 3, 1.2))
        self.assertTrue(gate.observe(.1, 0., 4, 1.51))
        gate.reset()  # missing/stale TF discards pending confirmation
        self.assertFalse(gate.observe(.1, 0., 5, 2.))
        self.assertFalse(gate.observe(.1, 0., 4, 2.5))

    def test_yaw_wrap_and_invalid_pose(self):
        gate = ApproachGate(.15, 10., .3)
        self.assertTrue(gate.contains(.1, 2*math.pi+.1))
        self.assertFalse(gate.contains(.1, math.pi))
        self.assertFalse(gate.contains(float('nan'), 0.))
        self.assertFalse(gate.contains(.1, float('inf')))

    def test_faster_alignment_still_requires_three_distinct_frames(self):
        gate = AlignmentGate(12., 3, .3)
        self.assertFalse(gate.observe([10., 11.], 1, 0.))
        self.assertFalse(gate.observe([10., 11.], 1, .2))
        self.assertFalse(gate.observe([10., 11.], 2, .21))
        self.assertTrue(gate.observe([10., 11.], 3, .42))
        self.assertFalse(gate.observe([13., 0.], 4, .63))
        self.assertEqual(gate.count, 0)
