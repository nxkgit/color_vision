#!/usr/bin/env python3
# ---------------------------------------------------------
# Name:   Michael Leonhard
# Date:   2026-11-18
# Course: CSIS-370 Robotics, Dr. Liu
# Final Project - Color Recognition (Milestone 1)
#
# detect_images.py
# Offline tool (no ROS master needed): runs the detector on image files
# or folders, saves annotated PNGs + a CSV to ~/color_runs, and with
# --sample X Y W H prints an HSV range for that box, ready to paste into
# colors/colors.yaml. Use it on photos saved from the robot camera to
# tune the colors under the real lab lighting.
#
#   python3 detect_images.py ~/color_samples
#   python3 detect_images.py shot.png --sample 300 200 40 40 --name orange
#
# Exit codes: 0 ok, 1 bad input.
# ---------------------------------------------------------
import argparse
import csv
import glob
import os
import sys
import time

import cv2

import color_geometry as cg

DEFAULT_COLORS_FILE = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                    '..', 'colors', 'colors.yaml'))
IMAGE_EXTS = ('.png', '.jpg', '.jpeg', '.bmp')


def parse_args():
    p = argparse.ArgumentParser(description='Run the color detector on saved images.')
    p.add_argument('paths', nargs='+', help='image files or folders')
    p.add_argument('--colors-file', default=DEFAULT_COLORS_FILE)
    p.add_argument('--colors', default='', help='comma-separated subset (default: all in the file)')
    p.add_argument('--min-area', type=float, default=300.0, help='px^2 (after resizing)')
    p.add_argument('--max-blobs', type=int, default=3, help='per color')
    p.add_argument('--kernel', type=int, default=5, help='open/close kernel (px)')
    p.add_argument('--hfov', type=float, default=60.0, help='camera horizontal field of view (deg)')
    p.add_argument('--resize-width', type=int, default=640, help='0 = keep size')
    p.add_argument('--sample', type=int, nargs=4, metavar=('X', 'Y', 'W', 'H'),
                   help='print an HSV range for this box of the first image (in resized pixels)')
    p.add_argument('--name', default='sampled', help='color name for --sample output')
    p.add_argument('--out-dir', default=os.path.expanduser('~/color_runs'))
    p.add_argument('--masks', action='store_true', help='also save each color mask')
    p.add_argument('--no-save', action='store_true', help='print only')
    return p.parse_args()


def expand(paths):
    files = []
    for path in paths:
        path = os.path.expanduser(path)
        if os.path.isdir(path):
            files.extend(sorted(f for f in glob.glob(os.path.join(path, '*')) if f.lower().endswith(IMAGE_EXTS)))
        else:
            files.append(path)
    return files


def main():
    a = parse_args()
    files = expand(a.paths)
    if not files:
        sys.exit('no images found in %s' % ', '.join(a.paths))
    names = [n.strip() for n in a.colors.split(',') if n.strip()]
    try:
        specs = cg.load_colors(a.colors_file, names or None)
    except (ValueError, IOError) as e:
        sys.exit(str(e))

    if a.sample:
        img = cv2.imread(files[0])
        if img is None:
            sys.exit('cannot read %s' % files[0])
        img = cg.resize_to_width(img, a.resize_width)
        try:
            lo, hi = cg.hsv_range_from_roi(img, a.sample)
        except ValueError as e:
            sys.exit(str(e))
        print('# from %s, box %s (image %dx%d)' % (files[0], tuple(a.sample), img.shape[1], img.shape[0]))
        print(cg.spec_yaml(a.name, lo, hi), end='')
        specs = dict(specs)
        specs[a.name] = cg.ColorSpec(a.name, lo, hi, (255, 0, 255))

    run_dir = os.path.join(a.out_dir, time.strftime('%Y%m%d_%H%M%S') + '_images')
    if not a.no_save:
        os.makedirs(run_dir, exist_ok=True)
    rows = []
    for f in files:
        img = cv2.imread(f)
        if img is None:
            print('%-40s  UNREADABLE' % os.path.basename(f))
            continue
        img = cg.resize_to_width(img, a.resize_width)
        h, w = img.shape[:2]
        dets, masks = cg.detect(img, specs, a.min_area, a.max_blobs, a.kernel)
        summary = ', '.join('%s (%.0f, %.0f) %.0f px2' % (d.color, d.cx, d.cy, d.area) for d in dets) or 'none'
        print('%-40s  %s' % (os.path.basename(f), summary))
        base = os.path.splitext(os.path.basename(f))[0]
        for d in dets:
            j = cg.detection_dict(d, w, h, a.hfov)
            rows.append([os.path.basename(f), j['color'], j['cx'], j['cy'], j['area'], j['radius'],
                         j['yaw_deg'], j['pitch_deg']])
        if not a.no_save:
            cv2.imwrite(os.path.join(run_dir, base + '_det.png'), cg.draw(img, dets, specs, a.hfov))
            if a.masks:
                for name, m in masks.items():
                    cv2.imwrite(os.path.join(run_dir, '%s_mask_%s.png' % (base, name)), m)

    if not a.no_save:
        with open(os.path.join(run_dir, 'detections.csv'), 'w', newline='') as fh:
            wr = csv.writer(fh)
            wr.writerow(['file', 'color', 'cx', 'cy', 'area', 'radius', 'yaw_deg', 'pitch_deg'])
            wr.writerows(rows)
        print('saved %d image(s) and detections.csv to %s' % (len(files), run_dir))


if __name__ == '__main__':
    main()
