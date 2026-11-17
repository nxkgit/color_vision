# Color recognition for the Yahboom ROSMASTER X3 Plus (ROS 1 Noetic)

Finds colored objects in the robot's camera image. For each configured color
(red, yellow, green, blue by default) it reports up to `max_blobs` blobs, largest
first. Each blob has its pixel centroid, area, enclosing radius and bounding box,
plus its bearing from the optical axis (yaw/pitch in degrees, from a pinhole model
with the horizontal field of view `hfov_deg`). Output is JSON on a `std_msgs/String`
topic and an annotated debug image.

**Perception only.** Nothing in this package publishes `/cmd_vel` or moves the arm.

## Files

| File | Purpose |
|---|---|
| `src/color_geometry.py` | Pure Python (numpy + cv2 + yaml, no ROS): color specs, HSV masks, blobs, pixel → angles, HSV range from a sample box, drawing |
| `src/test_color_geometry.py` | `unittest` tests on synthetic images |
| `src/color_detector.py` | Node: camera `CompressedImage` → `~detections` (JSON), `~debug_image/compressed`, `~status` |
| `src/fake_camera.py` | Simulator: publishes a synthetic scene (moving colored disks, ground truth on `~truth`) or loops saved photos |
| `src/detection_check.py` | Sim check: hit rate and centroid error of detector vs. `fake_camera` truth |
| `src/detect_images.py` | Offline (no ROS): detect on image files/folders, save annotated PNGs + CSV; `--sample` prints an HSV range |
| `colors/colors.yaml` | HSV ranges per color: **starting values, tune under the lab lighting** |
| `launch/color_detector_sim.launch` | Simulation: fake_camera + color_detector (+ check, + viewer) |
| `launch/color_detector.launch` | Real robot: color_detector only (the camera runs on the robot) |
| `sim_env.sh` | Points ROS at a **local** master for simulation |

Build: `cd ~/catkin_ws && catkin_make && chmod +x src/color_vision/src/*.py`.
Scripts run from `src/`. Do **not** add `catkin_install_python`: its devel relay
scripts break the local `import color_geometry`.

Images are decoded with `cv2.imdecode`, not `cv_bridge`. On this VM, `cv2` is pip
OpenCV 4.8 (`~/.local`), and ROS's `cv_bridge` is built against OpenCV 4.2.

## Color format

```yaml
red:
  lo: [170, 120, 70]     # H, S, V  (OpenCV: H 0..179, S/V 0..255)
  hi: [8, 255, 255]      # lo H > hi H wraps through 0: H 170..179 and 0..8
  bgr: [40, 40, 230]     # drawing color only
```
Only H may wrap; S and V need `lo <= hi`. Out-of-range values are refused when the
file loads. A pixel matches if it is inside the range. The mask is then cleaned
with an open + close (`kernel` px), and every outside contour of at least
`min_area` px² counts as a blob. Areas are in pixels **after** resizing to
`resize_width`.

## color_detector parameters

| Param | Default | Meaning |
|---|---|---|
| `~image_topic` | `/usb_cam/image_raw/compressed` | `sensor_msgs/CompressedImage` (jpeg/png): **verify on the robot** |
| `~colors_file` | package `colors/colors.yaml` | |
| `~colors` | `''` | comma list to use a subset, e.g. `red,blue`; empty = all |
| `~min_area` | 300 | px², smallest blob reported |
| `~max_blobs` | 3 | per color |
| `~kernel` | 5 | open/close kernel (px); 1 = no cleanup |
| `~hfov_deg` | 60 | camera horizontal field of view: **measure it** |
| `~resize_width` | 640 | images are scaled to this width first; 0 = keep |
| `~image_timeout` | 1.0 | s without an image → status `stale: ...` |
| `~publish_debug` | true | annotated JPEG on `~debug_image/compressed` (only encoded while someone subscribes) |
| `~jpeg_quality` | 80 | |
| `~show` | false | also show a cv2 window on this machine |

Topics:
- `~detections` (`std_msgs/String`, one per frame). The JSON carries the image `stamp`, `frame_id`, `width`, `height`, `hfov_deg` and a `detections` list. Each detection has `color`, `cx`, `cy` (px), `nx`, `ny` (−1..1 from the image center, + = right/down), `yaw_deg` (+ = left), `pitch_deg` (+ = up), `area`, `radius`, `bbox` [x, y, w, h]. The list is largest first.
- `~debug_image/compressed`.
- `~status` (latched): `waiting for images on <topic>`, `running`, `stale: no image for X s`.

Exit 1 on bad parameters or colors file.

`fake_camera` params: `~topic` `/usb_cam/image_raw/compressed`, `~rate` 15, `~width` 640, `~height` 480,
`~radius` 35, `~period` 12 s, `~noise_std` 0 (per-pixel Gaussian), `~image` `''` (photo or folder to loop).
`detection_check` params: `~duration` 10 s, `~min_hit_rate` 0.95 (exit 2 below it).

## Simulation (local master only)

Every command starts with `source ~/catkin_ws/src/color_vision/sim_env.sh &&`.
Your normal shell points at the robot, so check that it echoes
`ROS_MASTER_URI=http://localhost:11311`.

```bash
# unit tests (no ROS)
cd ~/catkin_ws/src/color_vision/src && python3 -m unittest -v test_color_geometry
# run with the viewer
source ~/catkin_ws/src/color_vision/sim_env.sh && roslaunch color_vision color_detector_sim.launch view:=true
# measured check: prints hit % and centroid error, then stops the launch
# ("REQUIRED process [detection_check] has died" is the normal shutdown)
source ~/catkin_ws/src/color_vision/sim_env.sh && roslaunch color_vision color_detector_sim.launch check:=true noise_std:=25
# loop saved photos through the full node
source ~/catkin_ws/src/color_vision/sim_env.sh && roslaunch color_vision color_detector_sim.launch image:=$HOME/color_samples view:=true
# watch the output
source ~/catkin_ws/src/color_vision/sim_env.sh && rostopic echo /color_detector/detections
```

## Tuning colors offline

```bash
cd ~/catkin_ws/src/color_vision/src
python3 detect_images.py ~/color_samples --masks          # -> ~/color_runs/<time>_images/*_det.png, *_mask_<color>.png, detections.csv
python3 detect_images.py ~/color_samples/shot1.png --sample 300 200 40 40 --name red
```
`--sample X Y W H` uses a box inside the object, in pixels of the image **after**
resizing to `--resize-width` (640). It prints a YAML entry built from the 5th–95th
percentile of the box's H, S and V, padded by (4, 30, 30), and wraps the hue for red.
Paste it into `colors/colors.yaml`, then check the masks on **other** photos
(different distance and lighting) for false positives before trusting it.
Exit 1 on unknown colors or no images.

## Verified (2026-11-18, VM, local master)

| Test | Result |
|---|---|
| `test_color_geometry` | 20/20 pass |
| sim check, defaults, 10 s | 144 matched frames; all 4 colors 100 % hit; centroid error mean 0.35–0.43 px, max ≤ 0.78 px |
| sim check, `noise_std:=25` | 62 matched frames; 100 % hit; mean ≤ 0.43 px, max ≤ 0.78 px |
| processing time | ~23 ms/frame at 640×480, 4 colors, debug image on |
| watchdog | killing fake_camera → status `stale: no image for 3.7 s (> 1.0 s)` |
| motion | no `/cmd_vel` topic exists while running |
| `detect_images.py` | 1280×960 images → correct colors and centers; `--sample` on a box spanning H 177/2 gives a wrapping range `[173..7]` that detects the patch |

These come from synthetic, evenly lit, flat-colored disks. They say nothing about
real lighting, shadows, white balance or camera blur.

## Real robot (read-only: nothing moves)

1. **Link check.** Use a normal terminal (do **not** source `sim_env.sh`), with the same checks as `~/robot_teleop.sh`.
   `ROS_IP` must be a 192.168.x address, not 10.0.2.x (VirtualBox NAT).
2. **Camera on the robot.** Yahboom's color tracker starts it with
   `roslaunch usb_cam usb_cam-test.launch`: 640 px wide, `yuyv`, frame `usb_cam`.
   On the VM, run `rostopic list | grep -i image`, then `rostopic hz /usb_cam/image_raw/compressed`.
   If the topic has another name, pass `image_topic:=<topic>`. If only a raw `sensor_msgs/Image` exists, stop:
   this node takes `CompressedImage`.
3. **Detector.** `roslaunch color_vision color_detector.launch view:=true`.
   Status should go `waiting` → `running`.
   If frames arrive slowly over Wi-Fi, check `rostopic hz /color_detector/detections`.
4. **Save sample photos** of each object under the lab lighting, at several distances:
   `rosrun image_view image_saver image:=/usb_cam/image_raw _image_transport:=compressed _filename_format:=$HOME/color_samples/%04d.png`
   (Ctrl-C after a few frames). Then tune offline (above).
5. **Field of view.** Put an object 1.00 m in front of the camera, then move it sideways until it sits at the image edge.
   Measure the sideways offset `d`. Then `hfov_deg = 2·atan(d / 1.00)` in degrees. Pass `hfov_deg:=<value>`.
   Check it by putting the object at a known angle and comparing with `yaw_deg`.
