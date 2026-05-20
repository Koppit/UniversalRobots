# Architecture

## Component map

```
vision/
  camera.py             BRIOCamera — background-threaded frame capture (1280×720)
  aruco_calibrator.py   ArucoCalibrator — detects markers, builds homography correspondences
  homography.py         HomographyConverter — calibrates H, converts Gemini→robot coords
  annotation.py         draw_boxes / draw_contours / grabcut_mask — OpenCV annotation

ai/
  detection.py          detect_objects(client, frame) → [{label, box_2d}]  (Gemini API)
  gemini_agent.py       GeminiAgent — task-driven agent (natural language → robot action)
  mcp_server.py         FastMCP server exposing robot+vision as tools for Claude

robot/
  ur3_controller.py     UR3Controller + RobotiqGripper — RTDE connection, moveL/moveJ, gripper
  robotiq_preamble.py   ROBOTIQ_PREAMBLE URScript constant — imported by ur3_controller
  transform.py          6-axis coordinate transform (scale, rotate, translate)
  set_robot_zero.py     One-time script: capture current TCP pose as work area centre
  mcp_server.py         Early standalone MCP server (port 8001, direct robot) — superseded by ai/mcp_server.py

tools/
  robot_tools.py        RobotActionTools — bridges Gemini normalised coords to robot actions
                          move_to_object, pick_object_at, place_object_at

web/
  server.py             Flask server — all globals (camera, Gemini, homography, robot)
  templates/index.html  Single-page UI — live feed, analysis, pick & place, calibration
```

---

## Data flow

### Object detection → pick and place

```
Camera frame (1280×720)
  → detect_objects() via Gemini API        → [{label, box_2d}]  (0-1000 normalised)
  → HomographyConverter.convert_gemini_to_robot()  → (rx_m, ry_m) in metres
  → RobotActionTools.pick_object_at()
      ├─ move to hover height (0.15 m)
      ├─ open gripper
      ├─ descend to pick height (0.05 m)
      ├─ close gripper
      └─ lift back to hover height
  → RobotActionTools.place_object_at()
      ├─ move to destination hover
      ├─ descend to place height
      ├─ open gripper
      └─ lift back to hover height
```

### ArUco calibration

```
Camera frame
  → ArucoCalibrator.detect()               → {id: corners_4×2}
  → ArucoCalibrator.build_correspondences()→ (pixel_pts, robot_pts)
  → cv2.findHomography(RANSAC)             → 3×3 matrix H
  → HomographyConverter._save()            → homography_matrix.json
  → _freeze_workspace_hull()               → convex hull polygon (pixels)
```

### Coordinate verification (Kalibrering tab)

```
Browser click on live-img
  → px, py (0-1280, 0-720)
  → ny = py/720*1000, nx = px/1280*1000   (0-1000 normalised)
  → POST /api/preview_coords {ny, nx}
  → HomographyConverter.convert_gemini_to_robot()
  → display X=___mm Y=___mm in UI
  → optional: POST /api/robot/move {x=rx_mm, y=ry_mm, z}
```

---

## Key globals in server.py

| Variable | Type | Purpose |
|----------|------|---------|
| `_cam` | `BRIOCamera` | Live frame source; opened at startup |
| `_client` | `genai.Client` | Gemini API client; `None` if key missing |
| `_homography` | `HomographyConverter` | Holds H matrix; loaded from file at startup |
| `_aruco` | `ArucoCalibrator\|None` | Lazy-initialised from `aruco_config.json` |
| `_robot` | `UR3Controller\|None` | `None` until `/api/robot/connect` |
| `_robot_tools` | `RobotActionTools\|None` | `None` until robot connected |
| `_workspace_hull` | `np.ndarray\|None` | Frozen convex hull after calibration |
| `_last_capture` | `bytes\|None` | JPEG bytes from last analysis |
| `_calib_overlay` | `bool` | ArUco overlay visible in live feed |
| `_mask_workspace` | `bool` | Workspace masking applied before Gemini |

---

## Configuration files

| File | Created by | Purpose |
|------|-----------|---------|
| `.env` | user | `GEMINI_API_KEY` |
| `aruco_config.json` | user | Marker IDs → robot XY positions (mm), marker size, ArUco dict |
| `homography_matrix.json` | `calibrate_aruco()` | 3×3 H matrix + camera resolution |
| `robot/zero_pose.json` | `set_robot_zero.py` | Reference TCP pose for relative coordinate moves |

---

## MCP integration

`ai/mcp_server.py` is a FastMCP server that wraps the Flask API (requires `web/server.py` running on `localhost:5000`). It exposes:

- `robot_status` / `connect_robot` / `disconnect_robot`
- `detect_objects` — Gemini detection via HTTP
- `pick_and_place(label, dest_ny, dest_nx)` — full pick-and-place sequence
- `wait_for_robot(timeout)` — blocks until `robot_busy` is false
- Gripper controls, workspace/joint limits, zero pose management

Configure in Claude Code's MCP settings:
```json
{
  "mcpServers": {
    "robot": {
      "command": "python",
      "args": ["ai/mcp_server.py"],
      "cwd": "<repo root>"
    }
  }
}
```
