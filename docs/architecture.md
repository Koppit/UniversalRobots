# Architecture

## Component map

```
vision/
  camera.py          BRIOCamera — background-threaded frame capture (1280×720)
  aruco_calibrator.py ArucoCalibrator — detects ArUco markers, builds homography correspondences
  homography.py      HomographyConverter — calibrates H matrix, converts Gemini→robot coords
  annotation.py      draw_boxes / draw_contours — annotates frames for display

ai/
  detection.py       detect_objects(client, frame) → [{label, box_2d}]  (Gemini API)
  gemini_agent.py    GeminiAgent — task-driven agent (not yet wired to web)

robot/
  ur3_controller.py  UR3Controller + RobotiqGripper — RTDE connection, moveL/moveJ, gripper
  transform.py       6-axis coordinate transform utilities
  calculate_distance.py  legacy pixel→meter helper (superseded by HomographyConverter)

tools/
  robot_tools.py     RobotActionTools — bridges Gemini coords to robot actions
                       move_to_object, pick_object_at, place_object_at

web/
  server.py          Flask server — camera, Gemini, homography, robot all live here as globals
  templates/index.html  Single-page UI — live feed, analysis, pick & place, calibration
```

## Data flow

```
Camera frame
  → detect_objects (Gemini API)  →  [{label, box_2d}]  (0-1000 normalised)
  → HomographyConverter          →  (rx, ry) in metres
  → RobotActionTools             →  UR3Controller.move_to_xyz / grab / release
```

## Key globals in server.py

| Variable | Type | Purpose |
|----------|------|---------|
| `_cam` | BRIOCamera | live frame source |
| `_client` | genai.Client | Gemini API client |
| `_homography` | HomographyConverter | pixel↔robot transform |
| `_aruco` | ArucoCalibrator | lazy-init, needs aruco_config.json |
| `_robot` | UR3Controller | None until `/api/robot/connect` |
| `_robot_tools` | RobotActionTools | None until robot connected |
| `_workspace_hull` | np.ndarray | frozen after ArUco calibration |

## Config files

| File | Created by | Purpose |
|------|-----------|---------|
| `aruco_config.json` | user | marker IDs → robot XY positions |
| `homography_matrix.json` | `calibrate_aruco()` | 3×3 H matrix + camera resolution |
| `zero_pose.json` | `set_robot_zero.py` | reference pose for relative moves |
| `.env` | user | `GEMINI_API_KEY`, `ROBOT_IP` |
