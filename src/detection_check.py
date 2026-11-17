#!/usr/bin/env python3
# ---------------------------------------------------------
# Name:   Michael Leonhard
# Date:   2026-11-18
# Course: CSIS-370 Robotics, Dr. Liu
# Final Project - Color Recognition (Milestone 1)
#
# detection_check.py
# Simulation check: pairs fake_camera ~truth with color_detector
# ~detections by timestamp for ~duration seconds, then prints per-color
# hit rate and centroid error (px) and exits 0, or 2 if any color was
# found in fewer than ~min_hit_rate of the frames.
# ---------------------------------------------------------
import json
import math
import sys
import time

import rospy
from std_msgs.msg import String


def main():
    rospy.init_node('detection_check')
    duration = float(rospy.get_param('~duration', 10.0))
    min_hit = float(rospy.get_param('~min_hit_rate', 0.95))
    truth = {}          # stamp -> truth list
    pairs = []          # (truth list, detections list)

    def on_truth(msg):
        d = json.loads(msg.data)
        truth[round(d['stamp'], 6)] = d['truth']

    def on_det(msg):
        d = json.loads(msg.data)
        t = truth.pop(round(d['stamp'], 6), None)
        if t is not None:
            pairs.append((t, d['detections']))

    rospy.Subscriber('/fake_camera/truth', String, on_truth, queue_size=50)
    rospy.Subscriber('/color_detector/detections', String, on_det, queue_size=50)
    end = time.time() + duration
    while time.time() < end and not rospy.is_shutdown():
        time.sleep(0.1)

    if not pairs:
        print('detection_check: no matched frames (is color_detector running?)')
        sys.exit(2)
    stats = {}
    for tr, dets in pairs:
        for t in tr:
            s = stats.setdefault(t['color'], {'hits': 0, 'n': 0, 'err': []})
            s['n'] += 1
            same = [d for d in dets if d['color'] == t['color']]
            if same:
                e = min(math.hypot(d['cx'] - t['cx'], d['cy'] - t['cy']) for d in same)
                s['hits'] += 1
                s['err'].append(e)
    print('\ndetection_check: %d matched frames' % len(pairs))
    print('%-8s %8s %12s %12s' % ('color', 'hit %', 'err mean px', 'err max px'))
    ok = True
    for name, s in stats.items():
        rate = s['hits'] / float(s['n'])
        ok = ok and rate >= min_hit
        e = s['err'] or [float('nan')]
        print('%-8s %8.1f %12.2f %12.2f' % (name, 100 * rate, sum(e) / len(e), max(e)))
    sys.exit(0 if ok else 2)


if __name__ == '__main__':
    main()
