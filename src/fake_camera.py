#!/usr/bin/env python3
# ---------------------------------------------------------
# Name:   Michael Leonhard
# Date:   2026-11-18
# Course: CSIS-370 Robotics, Dr. Liu
# Final Project - Color Recognition (Milestone 1)
#
# fake_camera.py
# Simulator: publishes sensor_msgs/CompressedImage on the robot's camera
# topic so color_detector can run on a local master without the robot.
# Default scene: one disk per color moving on a circle over a gray
# background, with optional pixel noise; the true centers go to ~truth
# (JSON) so the detection error can be measured. ~image instead loops a
# saved photo (or every image in a folder).
# ---------------------------------------------------------
import glob
import json
import math
import os
import sys

import cv2
import numpy as np

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

# name -> (H, S, V); the defaults in colors/colors.yaml should detect all of these
SCENE_COLORS = [('red', (2, 220, 200)), ('yellow', (27, 220, 230)),
                ('green', (60, 200, 180)), ('blue', (110, 220, 190))]
IMAGE_EXTS = ('.png', '.jpg', '.jpeg', '.bmp')


def hsv_to_bgr(hsv):
    return tuple(int(x) for x in cv2.cvtColor(np.uint8([[hsv]]), cv2.COLOR_HSV2BGR)[0, 0])


def synthetic_frame(t, w, h, radius, period, noise_std, rng):
    """Gray frame with one disk per SCENE_COLORS entry at time t; returns (bgr, truth list)."""
    img = np.full((h, w, 3), 110, np.uint8)
    truth = []
    n = len(SCENE_COLORS)
    for i, (name, hsv) in enumerate(SCENE_COLORS):
        a = 2 * math.pi * (t / period + i / float(n))
        cx = w / 2.0 + 0.32 * w * math.cos(a)
        cy = h / 2.0 + 0.30 * h * math.sin(a)
        cv2.circle(img, (int(round(cx)), int(round(cy))), radius, hsv_to_bgr(hsv), -1)
        truth.append({'color': name, 'cx': round(cx, 1), 'cy': round(cy, 1), 'radius': radius})
    if noise_std > 0:
        img = np.clip(img.astype(np.float32) + rng.normal(0, noise_std, img.shape), 0, 255).astype(np.uint8)
    return img, truth


def load_images(path):
    """List of BGR images from a file or a folder; exits 1 if none can be read."""
    path = os.path.expanduser(path)
    files = sorted(f for f in glob.glob(os.path.join(path, '*')) if f.lower().endswith(IMAGE_EXTS)) \
        if os.path.isdir(path) else [path]
    imgs = [cv2.imread(f) for f in files]
    imgs = [im for im in imgs if im is not None]
    if not imgs:
        rospy.logerr('fake_camera: no readable images at %s', path)
        sys.exit(1)
    return imgs


def main():
    rospy.init_node('fake_camera')
    topic = rospy.get_param('~topic', '/usb_cam/image_raw/compressed')
    rate_hz = float(rospy.get_param('~rate', 15.0))
    w = int(rospy.get_param('~width', 640))
    h = int(rospy.get_param('~height', 480))
    radius = int(rospy.get_param('~radius', 35))
    period = float(rospy.get_param('~period', 12.0))
    noise_std = float(rospy.get_param('~noise_std', 0.0))
    image = rospy.get_param('~image', '')
    frame_id = rospy.get_param('~frame_id', 'camera_link')

    pub = rospy.Publisher(topic, CompressedImage, queue_size=1)
    truth_pub = rospy.Publisher('~truth', String, queue_size=5)
    imgs = load_images(image) if image else None
    rng = np.random.default_rng(0)
    rospy.loginfo('fake_camera: %s on %s at %.0f Hz', 'images from %s' % image if image else 'synthetic scene',
                  topic, rate_hz)

    rate = rospy.Rate(rate_hz)
    t0 = rospy.Time.now()
    k = 0
    while not rospy.is_shutdown():
        stamp = rospy.Time.now()
        if imgs:
            bgr, truth = imgs[k % len(imgs)], None
        else:
            bgr, truth = synthetic_frame((stamp - t0).to_sec(), w, h, radius, period, noise_std, rng)
        ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 90])
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        pub.publish(msg)
        if truth is not None:
            truth_pub.publish(String(json.dumps({'stamp': stamp.to_sec(), 'truth': truth})))
        k += 1
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break


if __name__ == '__main__':
    main()
