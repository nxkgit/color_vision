#!/usr/bin/env python3
# ---------------------------------------------------------
# Name:   Michael Leonhard
# Date:   2026-11-18
# Course: CSIS-370 Robotics, Dr. Liu
# Final Project - Color Recognition (Milestone 1)
#
# color_detector.py
# Subscribes to the robot camera (sensor_msgs/CompressedImage), finds
# blobs of each configured color with color_geometry, and publishes them
# as JSON on ~detections plus an annotated ~debug_image/compressed.
#
# Perception only: this node never publishes /cmd_vel or moves the arm.
# Images are decoded with cv2.imdecode (no cv_bridge: the VM's pip
# OpenCV 4.8 and ROS's cv_bridge (OpenCV 4.2) do not mix).
#
# Exit codes: 0 shutdown, 1 bad parameters / colors file.
# ---------------------------------------------------------
import json
import os
import sys
import threading
import time

import cv2
import numpy as np

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

import color_geometry as cg

DEFAULT_COLORS_FILE = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                    '..', 'colors', 'colors.yaml'))
WATCHDOG_RATE = 2.0     # Hz: status / stale-image check
LOG_PERIOD = 5.0        # s between throttled detection log lines


def read_params():
    p = {
        'image_topic': rospy.get_param('~image_topic', '/usb_cam/image_raw/compressed'),
        'colors_file': rospy.get_param('~colors_file', DEFAULT_COLORS_FILE),
        'colors': rospy.get_param('~colors', ''),
        'min_area': float(rospy.get_param('~min_area', 300.0)),
        'max_blobs': int(rospy.get_param('~max_blobs', 3)),
        'kernel': int(rospy.get_param('~kernel', 5)),
        'hfov_deg': float(rospy.get_param('~hfov_deg', 60.0)),
        'resize_width': int(rospy.get_param('~resize_width', 640)),
        'image_timeout': float(rospy.get_param('~image_timeout', 1.0)),
        'publish_debug': bool(rospy.get_param('~publish_debug', True)),
        'jpeg_quality': int(rospy.get_param('~jpeg_quality', 80)),
        'show': bool(rospy.get_param('~show', False)),
    }
    names = p['colors']
    if isinstance(names, str):
        names = [n.strip() for n in names.split(',') if n.strip()]
    p['colors'] = list(names)
    return p


def check_params(p):
    """None if the parameters are usable, else the reason."""
    if p['min_area'] < 0:
        return '~min_area must be >= 0 (got %.1f)' % p['min_area']
    if p['max_blobs'] < 1:
        return '~max_blobs must be >= 1 (got %d)' % p['max_blobs']
    if not 0 < p['hfov_deg'] < 180:
        return '~hfov_deg must be in (0, 180) (got %.1f)' % p['hfov_deg']
    if p['kernel'] < 1:
        return '~kernel must be >= 1 (got %d)' % p['kernel']
    if p['image_timeout'] <= 0:
        return '~image_timeout must be > 0 (got %.2f)' % p['image_timeout']
    return None


class ColorDetector(object):

    def __init__(self, p, specs):
        self.p = p
        self.specs = specs
        self.lock = threading.Lock()
        self.received = None        # local time of the last decoded image
        self.frames = 0
        self.bad_frames = 0
        self.proc_time = 0.0        # s, exponential average
        self.last_log = 0.0
        self.show_frame = None

        self.det_pub = rospy.Publisher('~detections', String, queue_size=5)
        self.dbg_pub = rospy.Publisher('~debug_image/compressed', CompressedImage, queue_size=1)
        self.status_pub = rospy.Publisher('~status', String, queue_size=5, latch=True)
        self.status = None
        self.set_status('waiting for images on %s' % p['image_topic'])
        # buff_size: large so a slow callback drops old frames instead of queueing them in the socket
        rospy.Subscriber(p['image_topic'], CompressedImage, self.on_image, queue_size=1, buff_size=2 ** 24)

    def set_status(self, text):
        if text != self.status:
            self.status = text
            self.status_pub.publish(String(text))
            rospy.loginfo('color_detector: %s', text)

    def on_image(self, msg):
        t0 = time.time()
        bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            self.bad_frames += 1
            rospy.logwarn_throttle(LOG_PERIOD, 'color_detector: could not decode image (format %r), %d bad so far'
                                   % (msg.format, self.bad_frames))
            return
        bgr = cg.resize_to_width(bgr, self.p['resize_width'])
        h, w = bgr.shape[:2]
        dets, _ = cg.detect(bgr, self.specs, self.p['min_area'], self.p['max_blobs'], self.p['kernel'])

        out = {
            'stamp': msg.header.stamp.to_sec(),
            'frame_id': msg.header.frame_id,
            'width': w, 'height': h, 'hfov_deg': self.p['hfov_deg'],
            'detections': [cg.detection_dict(d, w, h, self.p['hfov_deg']) for d in dets],
        }
        self.det_pub.publish(String(json.dumps(out)))

        if self.p['publish_debug'] or self.p['show']:
            vis = cg.draw(bgr, dets, self.specs, self.p['hfov_deg'])
            cv2.putText(vis, '%.0f ms' % (self.proc_time * 1000), (8, h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1)
            if self.p['publish_debug'] and self.dbg_pub.get_num_connections() > 0:
                ok, buf = cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, self.p['jpeg_quality']])
                if ok:
                    dbg = CompressedImage()
                    dbg.header = msg.header
                    dbg.format = 'jpeg'
                    dbg.data = buf.tobytes()
                    self.dbg_pub.publish(dbg)
            if self.p['show']:
                with self.lock:
                    self.show_frame = vis

        dt = time.time() - t0
        with self.lock:
            self.received = time.time()
            self.frames += 1
            self.proc_time = dt if self.frames == 1 else 0.9 * self.proc_time + 0.1 * dt
        if time.time() - self.last_log > LOG_PERIOD:
            self.last_log = time.time()
            summary = ', '.join('%s %+.1f deg' % (d['color'], d['yaw_deg']) for d in out['detections']) or 'none'
            rospy.loginfo('color_detector: %dx%d, %.0f ms/frame, detections: %s', w, h, self.proc_time * 1000, summary)

    def age(self):
        with self.lock:
            return float('inf') if self.received is None else time.time() - self.received

    def spin(self):
        rate = rospy.Rate(WATCHDOG_RATE if not self.p['show'] else 30.0)
        while not rospy.is_shutdown():
            age = self.age()
            if self.received is None:
                pass
            elif age > self.p['image_timeout']:
                self.set_status('stale: no image for %.1f s (> %.1f s)' % (age, self.p['image_timeout']))
            else:
                self.set_status('running')
            if self.p['show']:
                with self.lock:
                    frame, self.show_frame = self.show_frame, None
                if frame is not None:
                    cv2.imshow('color_detector', frame)
                cv2.waitKey(1)
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break


def main():
    rospy.init_node('color_detector')
    p = read_params()
    reason = check_params(p)
    if reason:
        rospy.logerr('color_detector: %s', reason)
        sys.exit(1)
    try:
        specs = cg.load_colors(p['colors_file'], p['colors'] or None)
    except (ValueError, IOError) as e:
        rospy.logerr('color_detector: %s', e)
        sys.exit(1)
    rospy.loginfo('color_detector: colors %s from %s, min_area %.0f px, hfov %.1f deg',
                  ', '.join(specs), p['colors_file'], p['min_area'], p['hfov_deg'])
    node = ColorDetector(p, specs)
    node.spin()
    if p['show']:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
