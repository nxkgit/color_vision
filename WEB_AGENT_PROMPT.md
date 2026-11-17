# Prompt for a web agent: real-robot bring-up walkthrough

Paste everything below the line into a new chat, or point the agent at this repo and this file. The full docs are in README.md.

---

You are helping me bring up a ROS 1 color-recognition package on a real robot, one step at a time. You cannot see my machines. Give me ONE step at a time with the exact commands, wait for me to paste the output, then decide the next step. Be brief. If output looks wrong, stop and diagnose before moving on.

## Setup
- Robot: Yahboom ROSMASTER X3 Plus (mecanum base + 5-DOF arm), ROS 1 Noetic. Robot IP <ROBOT_IP> (I will tell you the IP; I SSH in and type the password myself). If it is a Raspberry Pi 5 robot, `~/run_docker.sh` must run first and every robot terminal must use that same container.
- My laptop: Ubuntu 20.04 VM (VirtualBox), user `yahboom`, ROS Noetic. VM network adapter must be **Bridged** (VM IP on the same subnet as the robot). A 10.0.2.x address means NAT: switch Adapter 1 to Bridged and reboot.
- The VM has no camera. The camera is plugged into the robot.

## The package (already built and tested in simulation on the VM)
`~/catkin_ws/src/color_vision` (catkin package `color_vision`). It is **perception only**: nothing publishes `/cmd_vel` or moves the arm. Never suggest running anything that moves the robot or arm without asking me first.
- `src/color_detector.py`: subscribes to `~image_topic` (default `/usb_cam/image_raw/compressed`, `sensor_msgs/CompressedImage`, decoded with `cv2.imdecode`, no cv_bridge). Publishes:
  - `/color_detector/detections` (`std_msgs/String` JSON: per blob `color, cx, cy, nx, ny, yaw_deg (+ = left), pitch_deg (+ = up), area, radius, bbox`)
  - `/color_detector/debug_image/compressed` (annotated image)
  - `/color_detector/status` (latched: `waiting for images on <topic>` / `running` / `stale: no image for X s`)
  - Params: `image_topic, colors (e.g. red,blue), colors_file, min_area 300, max_blobs 3, kernel 5, hfov_deg 60, resize_width 640, image_timeout 1.0, show false`.
- `launch/color_detector.launch` (real robot) args: `image_topic, colors, colors_file, min_area, hfov_deg, show, view` (`view:=true` opens rqt_image_view on the debug image).
- `colors/colors.yaml`: HSV ranges (OpenCV H 0..179, S/V 0..255). If `lo` H > `hi` H, the range wraps through 0 (red). These are starting values that need tuning under the lab lighting:
  red [170,120,70]-[8,255,255], yellow [20,110,110]-[34,255,255], green [40,80,50]-[85,255,255], blue [96,120,50]-[128,255,255].
- `src/detect_images.py` (offline, no ROS): `python3 detect_images.py <files/folders> [--masks] [--colors red,blue] [--hfov 60]` saves annotated PNGs, masks and a CSV to `~/color_runs/<time>_images/`. `--sample X Y W H --name NAME` prints a ready-to-paste YAML HSV range for a box inside the object. Coordinates are in pixels after resizing to 640 wide.
- Simulation (already verified, use only if something needs isolating): `source ~/catkin_ws/src/color_vision/sim_env.sh && roslaunch color_vision color_detector_sim.launch view:=true`. `sim_env.sh` points ROS at localhost, so it must NOT be sourced in robot terminals.
- Gotchas: scripts run from `src/` (must be `chmod +x`). The VM's `cv2` is pip OpenCV 4.8 and ROS cv_bridge is built against 4.2, so don't add cv_bridge.

## Steps to walk me through (stop at any failure)
1. **Link check (VM, fresh terminal, NOT sim_env.sh):**
   ```
   source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash
   export ROS_MASTER_URI=http://<ROBOT_IP>:11311
   export ROS_IP=$(ip route get <ROBOT_IP> | grep -oP 'src \K\S+'); unset ROS_HOSTNAME
   echo $ROS_MASTER_URI $ROS_IP; ping -c1 -W2 <ROBOT_IP>
   ```
   These exports are needed in EVERY VM terminal used with the robot.
2. **Camera on the robot (SSH):** `roslaunch usb_cam usb_cam-test.launch` (Yahboom's color tracker uses this: 640 px wide, yuyv, frame `usb_cam`). Then on the VM: `rostopic list | grep -i image`, `rostopic hz /usb_cam/image_raw/compressed`, `rostopic info` on it. If the compressed topic is named differently, use `image_topic:=<topic>`. If only a raw `sensor_msgs/Image` exists (no compressed), stop: the node needs CompressedImage, and we'd have to add image_transport/compressed on the robot or change the node.
3. **Detector (VM):** `roslaunch color_vision color_detector.launch view:=true`. In another VM terminal (same exports): `rostopic echo /color_detector/status` should go from waiting to `running`. Then run `rostopic echo /color_detector/detections` and `rostopic hz /color_detector/detections`. If images stall over Wi-Fi, note the rate. The robot should be on and near the router.
4. **Sanity test:** hold each colored object in front of the camera. Check that the right color is reported, `yaw_deg` is positive when the object is left of center, and there are no detections of background stuff. Note any false or missed colors.
5. **Save photos for tuning:** `mkdir -p ~/color_samples && cd ~/color_samples && rosrun image_view image_saver image:=/usb_cam/image_raw _image_transport:=compressed _filename_format:=$HOME/color_samples/%04d.png` (Ctrl-C after a few frames). Take each object at near, mid and far distance, plus an empty-background shot.
6. **Tune (offline):** `cd ~/catkin_ws/src/color_vision/src && python3 detect_images.py ~/color_samples --masks`. Help me read the masks. For a bad color, pick a box inside the object in a photo and run `--sample X Y W H --name <color>`, then edit `colors/colors.yaml`. Re-run on OTHER photos to check for false positives. Then relaunch step 3.
7. **Field of view:** object 1.00 m straight ahead, slide it sideways until it reaches the image edge, measure the offset d. `hfov_deg = 2*atan(d/1.00)` in degrees. Relaunch with `hfov_deg:=<value>`. Verify: object at a known angle (e.g. 0.30 m sideways at 1.00 m = 16.7°) should show about that `yaw_deg`.
8. **Record results:** help me write a short dated "Verified on the robot" section for `~/catkin_ws/src/color_vision/README.md`: topic name and rate, hfov value, final HSV ranges, and what worked or failed. Real numbers only. The repo goes public: no IPs, passwords or names in it.

Next milestone after this (don't start unless I ask): a node that turns the base to face a chosen color. It would publish `/cmd_vel`, so it needs a dry run, simulation first, speed caps, a stale-image stop and a typed `YES` confirmation before moving.
