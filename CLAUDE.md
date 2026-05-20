# Project: UniversalRobots

UR3 robotic arm + Logitech BRIO camera + Google Gemini vision. A Flask web server at `web/server.py` ties everything together. The camera sees objects, Gemini identifies them, a homography matrix maps pixels to robot coordinates, and the robot picks and places.

## How to run

```bash
.venv\Scripts\python web\server.py   # starts Flask on localhost:5000
```

The robot IP defaults to `192.168.0.25`. Camera auto-detected at startup. See [README.md](README.md) for first-time setup.

---

## Key files

```
web/server.py                 Flask server — all API endpoints and global state
web/templates/index.html      Single-page UI (vanilla HTML/CSS/JS, no build step)

robot/ur3_controller.py       UR3Controller (RTDE) + RobotiqGripper (URScript)
robot/set_robot_zero.py       Run once: captures current TCP pose as work area centre
robot/transform.py            transform_robot_coordinates() — scale/rotate/translate

vision/camera.py              BRIOCamera — background-threaded BRIO capture (1280×720)
vision/aruco_calibrator.py    ArucoCalibrator — detects markers, builds correspondences
vision/homography.py          HomographyConverter — calibrate H, convert pixel→robot coords
vision/annotation.py          draw_boxes / draw_contours / grabcut_mask

ai/detection.py               detect_objects(client, frame) via Gemini API
ai/gemini_agent.py            GeminiAgent — natural language → robot action
ai/mcp_server.py              FastMCP server exposing robot+vision as MCP tools for Claude

tools/robot_tools.py          RobotActionTools — pick/place sequences (hover→grip→lift)

aruco_config.json             Marker IDs → robot XY positions (mm), marker size, dict type
homography_matrix.json        Auto-generated 3×3 H matrix — do NOT edit manually
robot/zero_pose.json          Reference TCP pose — written by set_robot_zero.py
```

---

## Coordinate systems (critical domain knowledge)

Three spaces exist. Every bug involving wrong robot positions traces back to confusion between them.

| Space | Range | Notes |
|-------|-------|-------|
| **Pixel** | 0–1280 × 0–720 | OpenCV frames, ArUco corners |
| **Gemini normalised** | 0–1000 × 0–1000 | Gemini API output, UI click coords sent to server |
| **Robot (metres)** | ~−0.5 to +0.5 | UR3Controller inputs/outputs, zero-pose relative |

### Gemini box_2d order — Y FIRST
```python
box_2d = [y_min, x_min, y_max, x_max]   # Gemini always returns Y before X
ny = (box[0] + box[2]) / 2              # centre: Y component
nx = (box[1] + box[3]) / 2              # centre: X component
```
`convert_gemini_to_robot(ny, nx)` takes `(y, x)` in that order. Getting this backwards causes
the robot to move to mirrored/transposed positions.

### Pixel ↔ Gemini normalised
```python
nx = x_pixel / 1280 * 1000
ny = y_pixel / 720  * 1000
```

### Gemini normalised → robot metres
`HomographyConverter.convert_gemini_to_robot(ny, nx)` applies the 3×3 H matrix.
Returns `(rx_metres, ry_metres)`. Multiply by 1000 to display as mm.

### mm → robot move (axis inversion)
```python
MM_SCALE = [0.001, -0.001, -0.001]   # Y and Z are INVERTED
```
`_mm_to_robot(x, y, z)` in `server.py` applies this scale via `transform_robot_coordinates()`.
When a user types Y=100mm in the UI, the robot receives Y=−0.1m. This is intentional — it
maps the visual top-down frame to the UR3's physical axis directions. Do not remove the negation.

---

## Calibration pipeline

1. **`set_robot_zero.py`** — Move robot to work area centre, run script. Saves `robot/zero_pose.json`.
   All XY mm coordinates in the system are relative to this pose.

2. **ArUco markers** — Four printed markers (DICT_4X4_50, IDs 1–4, 4 cm) placed at corners.
   Positions in `aruco_config.json`:
   ```
   ID 1: (−435, −285) mm    ID 4: (+435, −285) mm
   ID 2: (−435, +285) mm    ID 3: (+435, +285) mm
   ```

3. **`POST /api/calibrate/run`** — Captures frame, detects marker centres, calls
   `cv2.findHomography(pixel_pts, robot_pts, RANSAC)`, saves `homography_matrix.json`,
   freezes `_workspace_hull` (convex hull of marker corners in pixels).

4. **Auto-load** — `_homography.load()` is called at server startup. After calibration,
   every detected object gets `rx_mm`/`ry_mm` appended to its detection dict.

---

## Gotchas

**`is_calibrated()` only checks if the file exists, not if H is in memory.**
Always call `_homography.load()` after creating `HomographyConverter()`. The server already does
this at startup, but tests or scripts that create a fresh instance must call it explicitly.

**`get_pose()` returns metres, not mm.** `[x_m, y_m, z_m, rx_rad, ry_rad, rz_rad]`

**`move_to_xyz()` expects the full 6-element list from `_mm_to_robot()`**, not raw mm values.

**Camera device_index is public**, not `_device_index`. Access as `_cam.device_index`.

**`_freeze_workspace_hull()` requires a live camera frame and visible markers.** Call it only
after a successful calibration, not during startup.

**The web UI has no build step.** Edit `web/templates/index.html` directly. Reload the browser.

**ArUco IDs in `aruco_config.json` are 1–4**, not 0–3. The detector returns them as integers.
The `required_ids` set in `ArucoCalibrator` is built from the config keys.

---

## Flask server patterns

All state lives as module-level globals in `server.py` (prefixed with `_`). Background tasks
(pick-and-place, auto-calibration) run in daemon threads and update `_status_msg` / `_robot_busy`.

The MJPEG stream at `/stream` encodes the latest frame from `_cam.capture_frame()` as JPEG
and yields it as multipart. The ArUco overlay is applied per-frame if `_calib_overlay` is True.

Robot connection flow: `POST /api/robot/connect` → creates `UR3Controller`, calls `connect()`,
activates gripper (5 s), creates `RobotActionTools`. Sets `_robot` and `_robot_tools` globals.

---

## MCP tools (ai/mcp_server.py)

Wraps the Flask HTTP API. Requires `web/server.py` running on `localhost:5000`.
Main tools: `connect_robot`, `detect_objects`, `pick_and_place(label, dest_ny, dest_nx)`,
`wait_for_robot(timeout)`. Zero pose and workspace/joint limits also exposed.

Configure in Claude Code MCP settings with `"command": "python", "args": ["ai/mcp_server.py"]`.

---

## Documentation maintenance

Keep these files in sync as the codebase changes. Update them in the same session as the code change, while context is warm — not in a separate session.

| File | Update when |
|------|-------------|
| `CLAUDE.md` | New gotcha found, new module added, architectural pattern changes |
| `README.md` | First-time setup steps change, new major feature implemented |
| `docs/api.md` | Endpoint added, removed, or its request/response shape changes |
| `docs/architecture.md` | New module, new data flow, globals table changes |
| `docs/coordinates.md` | Coordinate conventions or Z heights change |
| `web/README.md` | Server patterns, globals, or data flow changes |

Do **not** update docs for UI copy tweaks, CSS changes, or bug fixes that don't change behaviour.

## Documentation

- [docs/api.md](docs/api.md) — All endpoints with request/response shapes
- [docs/architecture.md](docs/architecture.md) — Component map and data flow diagrams
- [docs/coordinates.md](docs/coordinates.md) — Coordinate system conversions in detail
- [web/README.md](web/README.md) — Web server internals
