#!/usr/bin/env python3
# ---------------------------------------------------------
# Name:   Michael Leonhard
# Date:   2026-11-18
# Course: CSIS-370 Robotics, Dr. Liu
# Final Project - Color Recognition (Milestone 1)
#
# test_color_geometry.py
# unittest tests for color_geometry on synthetic images (no camera, no ROS).
#   cd ~/catkin_ws/src/color_vision/src && python3 -m unittest -v test_color_geometry
# ---------------------------------------------------------
import math
import os
import tempfile
import unittest

import cv2
import numpy as np

import color_geometry as cg

COLORS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'colors', 'colors.yaml')
W, H = 640, 480


def hsv_to_bgr(h, s, v):
    return tuple(int(x) for x in cv2.cvtColor(np.uint8([[[h, s, v]]]), cv2.COLOR_HSV2BGR)[0, 0])


def scene(disks, bg=(128, 128, 128)):
    """Gray image with filled disks [(cx, cy, r, (h, s, v)), ...]."""
    img = np.full((H, W, 3), bg, np.uint8)
    for cx, cy, r, hsv in disks:
        cv2.circle(img, (cx, cy), r, hsv_to_bgr(*hsv), -1)
    return img


class TestSpecs(unittest.TestCase):

    def test_default_file_loads(self):
        specs = cg.load_colors(COLORS_FILE)
        self.assertEqual(list(specs), ['red', 'yellow', 'green', 'blue'])
        self.assertGreater(specs['red'].lo[0], specs['red'].hi[0])   # red wraps

    def test_names_select_and_order(self):
        specs = cg.load_colors(COLORS_FILE, ['blue', 'red'])
        self.assertEqual(list(specs), ['blue', 'red'])

    def test_unknown_name_refused(self):
        with self.assertRaises(ValueError):
            cg.load_colors(COLORS_FILE, ['purple'])

    def write(self, text):
        f = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
        f.write(text)
        f.close()
        self.addCleanup(os.remove, f.name)
        return f.name

    def test_bad_ranges_refused(self):
        for text in ('x: {lo: [0, 0, 0], hi: [180, 255, 255]}',      # H > 179
                     'x: {lo: [0, 0, 0], hi: [10, 255]}',            # 2 values
                     'x: {lo: [0, 200, 0], hi: [10, 100, 255]}',     # S reversed
                     'x: {hi: [10, 255, 255]}',                      # no lo
                     ''):
            with self.assertRaises(ValueError, msg=text):
                cg.load_colors(self.write(text))

    def test_split_hue_wrap(self):
        self.assertEqual(cg.split_hue_wrap((10, 1, 2), (20, 3, 4)), [((10, 1, 2), (20, 3, 4))])
        self.assertEqual(cg.split_hue_wrap((170, 1, 2), (8, 3, 4)),
                         [((170, 1, 2), (179, 3, 4)), ((0, 1, 2), (8, 3, 4))])


class TestDetect(unittest.TestCase):

    def setUp(self):
        self.specs = cg.load_colors(COLORS_FILE)

    def test_one_disk_per_color(self):
        img = scene([(100, 100, 40, (60, 200, 200)),     # green
                     (500, 120, 30, (110, 220, 200)),    # blue
                     (320, 380, 50, (27, 220, 230))])    # yellow
        dets, masks = cg.detect(img, self.specs)
        by_color = {d.color: d for d in dets}
        self.assertEqual(sorted(by_color), ['blue', 'green', 'yellow'])
        for name, (cx, cy, r) in (('green', (100, 100, 40)), ('blue', (500, 120, 30)), ('yellow', (320, 380, 50))):
            d = by_color[name]
            self.assertAlmostEqual(d.cx, cx, delta=1.0)
            self.assertAlmostEqual(d.cy, cy, delta=1.0)
            self.assertAlmostEqual(d.radius, r, delta=2.0)
            self.assertAlmostEqual(d.area, math.pi * r * r, delta=0.05 * math.pi * r * r)
        self.assertEqual(set(masks), set(self.specs))

    def test_red_both_sides_of_zero(self):
        img = scene([(150, 240, 40, (2, 220, 200)), (450, 240, 40, (176, 220, 200))])
        dets, _ = cg.detect(img, cg.load_colors(COLORS_FILE, ['red']))
        self.assertEqual(len(dets), 2)
        self.assertEqual(sorted(round(d.cx) for d in dets), [150, 450])

    def test_largest_first_and_max_blobs(self):
        img = scene([(100, 100, 20, (60, 200, 200)), (300, 300, 60, (60, 200, 200)), (500, 100, 40, (60, 200, 200))])
        specs = cg.load_colors(COLORS_FILE, ['green'])
        dets, _ = cg.detect(img, specs, max_blobs=2)
        self.assertEqual([round(d.cx) for d in dets], [300, 500])

    def test_min_area_and_noise(self):
        img = scene([(320, 240, 6, (60, 200, 200))])     # area ~113 px
        img[10, 10] = hsv_to_bgr(60, 200, 200)           # single noise pixel
        specs = cg.load_colors(COLORS_FILE, ['green'])
        self.assertEqual(cg.detect(img, specs, min_area=300)[0], [])
        self.assertEqual(len(cg.detect(img, specs, min_area=50)[0]), 1)

    def test_gray_and_dark_ignored(self):
        img = scene([(200, 200, 50, (60, 30, 200)),      # washed-out green (low S)
                     (450, 200, 50, (110, 220, 20))])    # blue in the dark (low V)
        self.assertEqual(cg.detect(img, self.specs)[0], [])

    def test_draw_keeps_input(self):
        img = scene([(320, 240, 40, (60, 200, 200))])
        before = img.copy()
        dets, _ = cg.detect(img, self.specs)
        out = cg.draw(img, dets, self.specs, hfov_deg=60)
        self.assertTrue(np.array_equal(img, before))
        self.assertFalse(np.array_equal(out, before))

    def test_resize_to_width(self):
        img = np.zeros((960, 1280, 3), np.uint8)
        self.assertEqual(cg.resize_to_width(img, 640).shape, (480, 640, 3))
        self.assertIs(cg.resize_to_width(img, 0), img)


class TestGeometry(unittest.TestCase):

    def test_center_is_zero(self):
        self.assertEqual(cg.pixel_to_angles(320, 240, 640, 480, 60), (0.0, 0.0))

    def test_edges_are_half_fov(self):
        yaw, _ = cg.pixel_to_angles(0, 240, 640, 480, 60)
        self.assertAlmostEqual(yaw, math.radians(30))           # left edge: +30 deg
        yaw, _ = cg.pixel_to_angles(640, 240, 640, 480, 60)
        self.assertAlmostEqual(yaw, -math.radians(30))          # right edge: -30 deg

    def test_pitch_sign(self):
        _, pitch = cg.pixel_to_angles(320, 0, 640, 480, 60)
        self.assertGreater(pitch, 0)                            # top of image = up

    def test_detection_dict(self):
        d = cg.Detection('green', 480.0, 240.0, 1000.0, (460, 220, 40, 40), 20.0)
        j = cg.detection_dict(d, 640, 480, 60)
        self.assertEqual(j['nx'], 0.5)
        self.assertEqual(j['ny'], 0.0)
        self.assertLess(j['yaw_deg'], 0)


class TestTuning(unittest.TestCase):

    def test_roi_range_detects_its_own_color(self):
        img = scene([(320, 240, 60, (100, 180, 160))])
        lo, hi = cg.hsv_range_from_roi(img, (300, 220, 40, 40))
        self.assertLessEqual(lo[0], 100)
        self.assertGreaterEqual(hi[0], 100)
        spec = cg.ColorSpec('x', lo, hi, (0, 0, 0))
        self.assertEqual(len(cg.detect(img, {'x': spec})[0]), 1)

    def test_roi_on_red_wraps(self):
        img = scene([])
        img[200:280, 280:320] = hsv_to_bgr(177, 220, 200)
        img[200:280, 320:360] = hsv_to_bgr(2, 220, 200)
        lo, hi = cg.hsv_range_from_roi(img, (290, 210, 60, 60))
        self.assertGreater(lo[0], hi[0])                     # wraps through 0
        self.assertLessEqual(lo[0], 177)
        self.assertGreaterEqual(hi[0], 2)
        self.assertLess((hi[0] - lo[0]) % 180, 30)           # narrow, not the whole circle

    def test_roi_outside_image_refused(self):
        with self.assertRaises(ValueError):
            cg.hsv_range_from_roi(scene([]), (700, 10, 20, 20))

    def test_spec_yaml_round_trip(self):
        text = cg.spec_yaml('pink', (160, 50, 60), (175, 255, 255))
        f = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
        f.write(text)
        f.close()
        self.addCleanup(os.remove, f.name)
        self.assertEqual(cg.load_colors(f.name)['pink'].lo, (160, 50, 60))


if __name__ == '__main__':
    unittest.main()
