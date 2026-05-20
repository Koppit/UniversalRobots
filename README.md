# UniversalRobots

Computer vision and robotic arm control system. A UR3 robot with a Robotiq gripper is controlled via a web UI that uses a Logitech BRIO camera and Google Gemini vision to detect objects and execute pick-and-place operations. ArUco fiducial markers calibrate the camera-to-robot coordinate transform automatically.

---

## Quick Start

> For first-time physical setup, see [First-time Setup](#first-time-setup) below.

```bash
# 1. Install dependencies
pip install -r requirements.txt

# 2. Create .env and add your API key
copy .env.example .env
# Edit .env: set GEMINI_API_KEY=...

# 3. Start the web server
.venv\Scripts\python web\server.py

# 4. Open in browser
# http://localhost:5000
```

- Connect to the robot in the **Operasjon** tab (default IP `192.168.0.25`)
- Run ArUco calibration in the **Kalibrering** tab before using pick-and-place

---

## What is implemented

### Camera
- **Logitech BRIO** USB camera with automatic detection (`BRIOCamera.find_brio()`)
- Background capture thread — frames are always available without blocking
- Runtime camera switching via web UI dropdown

### Object detection
- **Google Gemini** `gemini-robotics-er-1.6-preview` model
- Two annotation modes: bounding boxes and GrabCut contour segmentation
- Detected objects listed with their robot XY coordinates in mm (once calibrated)

### Calibration
- **ArUco automatic calibration** — four fiducial markers at the work area corners; one button press computes the full camera-to-robot perspective homography
- Calibration saved to `homography_matrix.json`, loaded automatically at startup
- **Coordinate verification tool** — click any point in the live feed to read its robot XY in mm; optionally move the robot to that exact point

### Robot control
- **UR3** 6-axis arm via `ur-rtde` (RTDE protocol over Ethernet)
- **Robotiq gripper** via URScript
- Manual jog: XY/Z/rotation in mm and degrees from the web UI
- Pick and place: select detected object → click destination → execute
- Emergency stop button

### MCP integration
- `ai/mcp_server.py` exposes the robot and camera as MCP tools usable directly from Claude
- Supports natural-language commands: *"pick the red cube and place it on the tray"*

---

## First-time Setup

### Step 1 — Define the robot's work area centre (`set_robot_zero`)

The very first thing to do when configuring this system is to define the **centre point of the robot's work area**. This is a manual one-time process.

**Why:** All robot XY coordinates in the system are relative to this centre point. The four ArUco markers are positioned at ±435 mm (X) and ±285 mm (Y) from it. If this pose is wrong, every coordinate will be off.

**How:**

1. Physically jog the robot arm (using the teach pendant or `web/server.py` manual jog) to the centre of the intended work surface — the point equidistant between where the four ArUco markers will be placed.
2. Run the zero-capture script:
   ```bash
   python robot/set_robot_zero.py
   ```
   This reads the current TCP pose via RTDE and writes it to `robot/zero_pose.json`. All future moves specified in mm are relative to this pose.

> You only need to repeat this if the robot is physically relocated or the work surface changes.

---

### Step 2 — Place the ArUco markers

Print four ArUco markers from the **DICT_4X4_50** dictionary, IDs 1–4, at **4 cm** physical size. Attach them flat to the work surface at these positions relative to the zero pose:

| Marker ID | X (mm) | Y (mm) | Corner |
|-----------|-------:|-------:|--------|
| 1 | −435 | −285 | back-left |
| 2 | −435 | +285 | front-left |
| 3 | +435 | +285 | front-right |
| 4 | +435 | −285 | back-right |

These values are defined in `aruco_config.json`. If your work area is a different size, edit the coordinates there to match.

The camera must have a clear overhead view of all four markers without obstruction.

---

### Step 3 — Run ArUco calibration

With the markers in place and the camera running:

1. Start the web server and open [http://localhost:5000](http://localhost:5000)
2. Go to the **Kalibrering** tab
3. The four marker-status chips show which IDs the camera currently sees (green = detected)
4. When all four are green, click **Kalibrer nå**
5. The system captures a frame, detects the marker centres, and computes the homography matrix via `cv2.findHomography` (RANSAC)
6. The result is written to `homography_matrix.json` and loaded automatically on all future server starts

After this step, the **Operasjon** tab will show robot XY coordinates (mm) next to each detected object.

---

### Step 4 — Verify the calibration

Use the **Verifiser kalibrering** panel in the **Kalibrering** tab to confirm accuracy:

1. Click **Aktiver klikk-verifisering**
2. Click a known physical point in the live feed (e.g. a marker corner)
3. The panel shows the mapped robot XY in mm
4. Optionally click **Flytt robot hit** to move the robot to that point and check physically

---

## Configuration files

| File | Created by | Purpose |
|------|-----------|---------|
| `.env` | user | `GEMINI_API_KEY` |
| `aruco_config.json` | user | ArUco marker IDs → robot XY positions (mm) and marker size |
| `homography_matrix.json` | calibration | 3×3 homography matrix + camera resolution |
| `robot/zero_pose.json` | `set_robot_zero.py` | Reference TCP pose for relative coordinate moves |

---

## File overview

See [file-Overview.md](file-Overview.md) for the complete listing with status (active / standalone / test / legacy).

Core runtime files loaded by `web/server.py`:

```
robot/
  ur3_controller.py     # UR3Controller + RobotiqGripper (RTDE)
  robotiq_preamble.py   # URScript preamble constant used by ur3_controller
  transform.py          # 6-axis coordinate transformation utilities
  zero_pose.json        # Written by set_robot_zero.py

vision/
  camera.py             # BRIOCamera — BRIO capture with auto-detection
  aruco_calibrator.py   # ArUco marker detection and homography building
  homography.py         # HomographyConverter — pixel-to-robot coordinate mapping
  annotation.py         # OpenCV draw helpers (boxes, contours, GrabCut)

ai/
  detection.py          # detect_objects() via Gemini API
  gemini_agent.py       # Task-driven agent (natural language → robot action)

tools/
  robot_tools.py        # RobotActionTools — pick/place sequences bridging Gemini→robot

web/
  server.py             # Flask server — all API endpoints
  templates/index.html  # Web UI (vanilla HTML/CSS/JS, no build step)

aruco_config.json       # Marker IDs and their robot XY positions (mm)
homography_matrix.json  # Auto-generated; do not edit manually
requirements.txt        # Python dependencies
```

Standalone scripts (not imported by the server):

```
robot/set_robot_zero.py   # Run once to capture work area centre as reference pose
ai/mcp_server.py          # FastMCP server wrapping the Flask API — for Claude MCP access
robot/mcp_server.py       # Early direct-robot MCP server (port 8001); superseded by ai/mcp_server.py
```

---

## Documentation

- [file-Overview.md](file-Overview.md) — Complete file listing with active/standalone/test/legacy status
- [docs/api.md](docs/api.md) — All Flask API endpoints with request/response shapes
- [docs/architecture.md](docs/architecture.md) — Component map and data flow
- [docs/coordinates.md](docs/coordinates.md) — Coordinate systems and conversions
- [web/README.md](web/README.md) — Web server internals and data flow
