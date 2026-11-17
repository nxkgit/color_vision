#!/usr/bin/env python3
# ---------------------------------------------------------
# Name:   Michael Leonhard
# Date:   2026-11-18
# Course: CSIS-370 Robotics, Dr. Liu
# Final Project - Color Recognition (Milestone 1)
#
# color_geometry.py
# Pure Python (numpy + cv2 + yaml, no ROS): color specs, HSV masks,
# blob detection, pixel -> bearing angles, HSV range from a sample box,
# and the annotated debug image. Everything the nodes do with pixels
# lives here so it can be unit tested on synthetic images.
#
# HSV is OpenCV's: H 0..179, S 0..255, V 0..255. A range whose hue
# low end is above its high end wraps through 0 (red: H 170 -> 10).
# ---------------------------------------------------------
import collections
import math
import os

import cv2
import numpy as np
import yaml

H_MAX = 179     # OpenCV 8-bit hue is 0..179
SV_MAX = 255

ColorSpec = collections.namedtuple('ColorSpec', 'name lo hi bgr')
# cx, cy: centroid (px); area: contour area (px^2); bbox: (x, y, w, h); radius: enclosing circle (px)
Detection = collections.namedtuple('Detection', 'color cx cy area bbox radius')


# ------------------------------------------------------------------ specs

def check_hsv(name, lo, hi):
    """Raise ValueError unless lo/hi are 3-number HSV triples in range with S, V ordered."""
    for label, t in (('lo', lo), ('hi', hi)):
        if len(t) != 3:
            raise ValueError('%s: %s must have 3 values (H, S, V), got %r' % (name, label, t))
        h, s, v = t
        if not (0 <= h <= H_MAX and 0 <= s <= SV_MAX and 0 <= v <= SV_MAX):
            raise ValueError('%s: %s %r out of range (H 0..%d, S/V 0..%d)' % (name, label, t, H_MAX, SV_MAX))
    if lo[1] > hi[1] or lo[2] > hi[2]:
        raise ValueError('%s: S and V need lo <= hi (lo %r, hi %r); only H may wrap' % (name, lo, hi))


def load_colors(path, names=None):
    """Read a colors YAML into an ordered {name: ColorSpec}; names limits/orders the result."""
    with open(os.path.expanduser(path)) as f:
        raw = yaml.safe_load(f) or {}
    if not isinstance(raw, dict):
        raise ValueError('%s: expected a mapping of color name -> {lo, hi, bgr}' % path)
    specs = collections.OrderedDict()
    for name, d in raw.items():
        try:
            lo, hi = [int(x) for x in d['lo']], [int(x) for x in d['hi']]
        except (KeyError, TypeError, ValueError):
            raise ValueError('%s: color %r needs integer lists lo and hi' % (path, name))
        check_hsv(name, lo, hi)
        bgr = tuple(int(x) for x in d.get('bgr', (255, 255, 255)))
        specs[name] = ColorSpec(name, tuple(lo), tuple(hi), bgr)
    if names:
        missing = [n for n in names if n not in specs]
        if missing:
            raise ValueError('unknown color(s) %s; %s has %s' % (', '.join(missing), path, ', '.join(specs)))
        specs = collections.OrderedDict((n, specs[n]) for n in names)
    if not specs:
        raise ValueError('%s: no colors defined' % path)
    return specs


def split_hue_wrap(lo, hi):
    """List of (lo, hi) ranges for cv2.inRange; a wrapping hue (lo H > hi H) becomes two."""
    if lo[0] <= hi[0]:
        return [(tuple(lo), tuple(hi))]
    return [((lo[0], lo[1], lo[2]), (H_MAX, hi[1], hi[2])),
            ((0, lo[1], lo[2]), (hi[0], hi[1], hi[2]))]


# ------------------------------------------------------------------ detection

def make_mask(hsv, spec, kernel=5):
    """Binary mask (uint8 0/255) of pixels inside spec, cleaned by open + close with a kernel x kernel box."""
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in split_hue_wrap(spec.lo, spec.hi):
        mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    if kernel > 1:
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel, kernel))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    return mask


def find_blobs(mask, color, min_area, max_blobs):
    """Up to max_blobs Detections for the external contours of mask with area >= min_area, largest first."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    found = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        m = cv2.moments(c)
        if m['m00'] <= 0:
            continue
        (_, _), r = cv2.minEnclosingCircle(c)
        found.append(Detection(color, m['m10'] / m['m00'], m['m01'] / m['m00'], area,
                               tuple(int(v) for v in cv2.boundingRect(c)), r))
    found.sort(key=lambda d: d.area, reverse=True)
    return found[:max_blobs]


def detect(bgr, specs, min_area=300, max_blobs=3, kernel=5):
    """Detections for every spec in a BGR image, largest first; also returns {name: mask}."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    dets, masks = [], collections.OrderedDict()
    for name, spec in specs.items():
        masks[name] = make_mask(hsv, spec, kernel)
        dets.extend(find_blobs(masks[name], name, min_area, max_blobs))
    dets.sort(key=lambda d: d.area, reverse=True)
    return dets, masks


def resize_to_width(bgr, width):
    """Scale an image to the given width (keeping aspect); width <= 0 or equal leaves it unchanged."""
    h, w = bgr.shape[:2]
    if width <= 0 or w == width:
        return bgr
    return cv2.resize(bgr, (int(width), int(round(h * width / float(w)))), interpolation=cv2.INTER_AREA)


# ------------------------------------------------------------------ geometry

def pixel_to_angles(cx, cy, width, height, hfov_deg):
    """Bearing (rad) of a pixel from the optical axis, pinhole model, square pixels.

    ROS convention: yaw > 0 = left of center, pitch > 0 = above center.
    """
    f = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    yaw = -math.atan2(cx - width / 2.0, f)
    pitch = -math.atan2(cy - height / 2.0, f)
    return yaw, pitch


def detection_dict(d, width, height, hfov_deg):
    """JSON-ready dict for one Detection (pixels, normalized offsets in -1..1, angles in degrees)."""
    yaw, pitch = pixel_to_angles(d.cx, d.cy, width, height, hfov_deg)
    return {
        'color': d.color,
        'cx': round(d.cx, 1), 'cy': round(d.cy, 1),
        'nx': round((d.cx - width / 2.0) / (width / 2.0), 4),
        'ny': round((d.cy - height / 2.0) / (height / 2.0), 4),
        'yaw_deg': round(math.degrees(yaw), 2), 'pitch_deg': round(math.degrees(pitch), 2),
        'area': int(d.area), 'radius': round(d.radius, 1), 'bbox': list(d.bbox),
    }


# ------------------------------------------------------------------ tuning

def hsv_range_from_roi(bgr, roi, pct=(5, 95), pad=(4, 30, 30)):
    """(lo, hi) HSV range covering the pct percentiles of a sample box roi = (x, y, w, h), widened by pad.

    Hue is circular: if the box straddles H = 0 (red), the result wraps (lo H > hi H).
    """
    x, y, w, h = [int(v) for v in roi]
    patch = bgr[y:y + h, x:x + w]
    if patch.size == 0:
        raise ValueError('sample box %r is empty or outside the %dx%d image' % (roi, bgr.shape[1], bgr.shape[0]))
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV).reshape(-1, 3).astype(np.int32)
    hue = hsv[:, 0]
    # Pick whichever of plain / shifted-by-90 hue has the narrower spread (shifted handles red)
    best = None
    for shift in (0, 90):
        hs = (hue + shift) % (H_MAX + 1)
        a, b = np.percentile(hs, pct)
        if best is None or b - a < best[2] - best[1]:
            best = (shift, a, b)
    shift, a, b = best
    h_lo = int(round(a - pad[0] - shift)) % (H_MAX + 1)
    h_hi = int(round(b + pad[0] - shift)) % (H_MAX + 1)
    if b - a + 2 * pad[0] >= H_MAX:
        h_lo, h_hi = 0, H_MAX
    lo, hi = [h_lo], [h_hi]
    for ch, p in ((1, pad[1]), (2, pad[2])):
        a, b = np.percentile(hsv[:, ch], pct)
        lo.append(int(max(0, round(a - p))))
        hi.append(int(min(SV_MAX, round(b + p))))
    return tuple(lo), tuple(hi)


def spec_yaml(name, lo, hi, bgr=(255, 255, 255)):
    """One colors-YAML entry as text, ready to paste."""
    return '%s:\n  lo: [%d, %d, %d]\n  hi: [%d, %d, %d]\n  bgr: [%d, %d, %d]\n' % ((name,) + tuple(lo) + tuple(hi) + tuple(bgr))


# ------------------------------------------------------------------ drawing

def draw(bgr, dets, specs, hfov_deg=None):
    """Copy of the image with each detection's circle, centroid and label; optional crosshair."""
    out = bgr.copy()
    h, w = out.shape[:2]
    cv2.line(out, (w // 2 - 10, h // 2), (w // 2 + 10, h // 2), (200, 200, 200), 1)
    cv2.line(out, (w // 2, h // 2 - 10), (w // 2, h // 2 + 10), (200, 200, 200), 1)
    for d in dets:
        col = specs[d.color].bgr if d.color in specs else (255, 255, 255)
        c = (int(round(d.cx)), int(round(d.cy)))
        cv2.circle(out, c, int(round(d.radius)), col, 2)
        cv2.circle(out, c, 3, (0, 0, 0), -1)
        label = d.color
        if hfov_deg:
            yaw, _ = pixel_to_angles(d.cx, d.cy, w, h, hfov_deg)
            label += ' %+.1f deg' % math.degrees(yaw)
        org = (max(0, c[0] - int(d.radius)), max(12, c[1] - int(d.radius) - 6))
        cv2.putText(out, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
        cv2.putText(out, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
    return out
