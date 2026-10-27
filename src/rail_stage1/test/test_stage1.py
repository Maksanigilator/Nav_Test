import json
import math
from pathlib import Path
import tempfile
import unittest
import cv2
import numpy as np
import yaml
from rail_stage1.geometry import detect, select_three
from rail_stage1.store import RailStore, approach_pose

CFG = yaml.safe_load((Path(__file__).parents[1]/'config/rails.yaml').read_text())['rail_finder']['ros__parameters']


def scene(parts=3, tilt_deg=30):
    angle = math.radians(tilt_deg)
    r = np.array([[0., -math.sin(angle), math.cos(angle)],
                  [-1., 0., 0.], [0., -math.cos(angle), -math.sin(angle)]])
    origin = np.array([0., 0., .6])
    k = np.array([[606., 0., 424.], [0., 606., 240.], [0., 0., 1.]])
    yy, xx = np.mgrid[:480, :848]
    rays = np.stack(((xx-424)/606, (yy-240)/606, np.ones_like(xx)), axis=-1) @ r.T
    with np.errstate(divide='ignore', invalid='ignore'):
        depth = -.6/rays[..., 2]
    depth[(depth <= 0) | (depth > 5)] = 0
    mask = np.zeros((480, 848), np.uint8)
    half = CFG['rail_gauge']/2
    pairs = [([1., -half, .04], [2.4, -half, .04]),
             ([1., half, .04], [2.4, half, .04]),
             ([1., -half, .04], [1., half, .04])]
    for a, b in pairs[:parts]:
        cam = (np.array([a, b])-origin) @ r
        px, _ = cv2.projectPoints(cam, np.zeros(3), np.zeros(3), k, np.zeros(5))
        cv2.line(mask, tuple(px[0, 0].astype(int)), tuple(px[1, 0].astype(int)), 255, 12)
    return mask, depth, k, np.zeros(5), r, origin


class GeometryTests(unittest.TestCase):
    def test_complete_u_and_camera_tilt(self):
        for tilt in (25, 35):
            point, direction, _, lines = detect(*scene(tilt_deg=tilt), CFG)
            np.testing.assert_allclose(point, [1., 0., 0.], atol=.08)
            self.assertGreater(direction[0], .98)
            self.assertEqual(len(lines), 3)

    def test_incomplete_u_rejected(self):
        for parts in (1, 2):
            with self.assertRaisesRegex(ValueError, 'three-line'):
                detect(*scene(parts), CFG)

    def test_crossbar_feature_projects_inside_mask(self):
        data = scene()
        point, _, _, _, feature = detect(*data, CFG, return_feature=True)
        self.assertAlmostEqual(np.linalg.norm(feature-point), CFG['feature_height'], places=5)
        mask, _, k, distortion, r, origin = data
        cam = r.T @ (feature-origin)
        pixel = cv2.projectPoints(cam[None], np.zeros(3), np.zeros(3), k, distortion)[0].reshape(2)
        x, y = np.rint(pixel).astype(int)
        self.assertGreater(np.count_nonzero(mask[y-5:y+6, x-5:x+6]), 3)

    def test_area_threshold(self):
        data = list(scene())
        data[0][:] = 0
        data[0][10:20, 10:20] = 255
        with self.assertRaisesRegex(ValueError, 'area'):
            detect(*data, CFG)

    def test_missing_floor_rejected(self):
        data = list(scene())
        data[1][:] = 0
        with self.assertRaisesRegex(ValueError, 'floor'):
            detect(*data, CFG)


class RegistryTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name)/'rails.json'
        self.store = RailStore(self.path, 'test_map', CFG)

    def confirm(self, start=1, point=None, yaw=0):
        point = np.array([1., 0., 0.]) if point is None else np.array(point)
        rail = None
        for i in range(5):
            rail = self.store.observe(point, yaw, start+i, 1.+i*.2)
        return rail

    def test_independent_frames_required(self):
        for _ in range(10):
            self.assertIsNone(self.store.observe(np.array([1., 0., 0.]), 0., 1, 1.))
        self.assertEqual(len(self.store.rails), 0)

    def test_persistence_map_frame_and_deduplication(self):
        rail = self.confirm()
        self.assertEqual(rail['id'], 'rail_001')
        self.confirm(10, [1.04, .02, 0.])
        self.assertEqual(len(self.store.rails), 1)
        loaded = RailStore(self.path, 'test_map', CFG)
        self.assertEqual(len(loaded.rails), 1)
        self.assertEqual(json.loads(self.path.read_text())['frame'], 'map')
        with self.assertRaisesRegex(ValueError, 'different map'):
            RailStore(self.path, 'other_map', CFG)

    def test_approach_sign_and_distance(self):
        rail = self.confirm()
        point, yaw = approach_pose(rail, .9)
        np.testing.assert_allclose(point, [.1, 0., 0.])
        self.assertEqual(yaw, 0.)

    def test_multiple_objects_and_flipped_pose(self):
        self.confirm()
        self.confirm(10, [2., 1., 0.])
        self.assertEqual(len(self.store.rails), 2)
        self.confirm(20, [1., 0., 0.], math.pi)
        self.assertEqual(len(self.store.rails), 2)


if __name__ == '__main__':
    unittest.main()
