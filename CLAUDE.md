# Handoff for Claude Code

ROS 1 Noetic (Ubuntu 20.04 VM) camera + color-recognition package for the Yahboom ROSMASTER X3 Plus
(CSIS-370 Robotics final project). Read README.md first. This is separate from the shape/arm drawing repo
(`~/Documents/roboticsfinalproject`, package `beginner_tutorials`); keep its style: header block per file,
pure-Python logic module + unittest, thin rospy node with `~` params, sim first, README with dated verified results.

## State (2026-11-18)
- Milestone 1 done in sim: detector, fake camera, sim check, offline tuner. Never run on the real camera.
- Unknowns: real camera topic/format, `hfov_deg`, HSV ranges under lab lighting.
- Yahboom reference code (read only): `~/yahboomcar_ws/src/yahboomcar_astra/scripts/{colorHSV,colorTracker,astra_common}.py`,
  `~/yahboomcar_ws/src/arm_color_transport/`. They publish `yahboomcar_msgs/Position` on `/Current_point`.

## Gotchas
- No `cv_bridge`: pip OpenCV 4.8 vs ROS cv_bridge (4.2). Decode with `cv2.imdecode`.
- No `catkin_install_python` (devel relays break `import color_geometry`); scripts run from `src/`, must be `chmod +x`.
- VM has no `/dev/video*`; the camera is on the robot.

## Rules
- This package is perception only. Anything that publishes `/cmd_vel` or moves the arm needs the user's OK before
  running on the real robot, the `YES` gate on a non-local master, and sim first.
- The user types passwords. No IPs, passwords, keys or personal data in commits.
