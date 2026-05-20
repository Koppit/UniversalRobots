# web/

Flask web server for robot vision control. Provides a live camera feed, Gemini object detection, ArUco calibration, and robot pick-and-place via a browser UI.

## Running

```bash
.venv\Scripts\python web\server.py
```

Open [http://localhost:5000](http://localhost:5000).

---

## UI tabs

**Operasjon**
- Live camera feed (MJPEG stream)
- Analyze button — sends current frame to Gemini, lists detected objects with robot XY coordinates
- Object selection and destination click for pick-and-place
- Robot connection, manual jog, emergency stop

**Kalibrering**
- ArUco marker status (which of the four markers the camera currently sees)
- **Kalibrer nå** — runs `HomographyConverter.calibrate_aruco()`, saves `homography_matrix.json`
- **Verifiser kalibrering** — click in live feed to read robot XY; optionally move robot to that point
- Camera preview showing ArUco detections

**Debug**
- Scrolling log console (server-side messages, coordinate logs, errors)

---

## File structure

```
web/
├── server.py          # Flask application and all API endpoints
└── templates/
    └── index.html     # Single-page UI (vanilla HTML/CSS/JS, no build step)
```

---

## Dependencies

### Project modules

| Module | File | Used for |
|--------|------|----------|
| `ai.detection` | `ai/detection.py` | `make_client`, `detect_objects` — Gemini client and image analysis |
| `vision.camera` | `vision/camera.py` | `BRIOCamera` — background-threaded camera capture |
| `vision.homography` | `vision/homography.py` | `HomographyConverter` — pixel → robot XY mapping |
| `vision.aruco_calibrator` | `vision/aruco_calibrator.py` | `ArucoCalibrator` — marker detection and homography |
| `vision.annotation` | `vision/annotation.py` | `draw_boxes`, `draw_contours` — frame annotation |
| `robot.ur3_controller` | `robot/ur3_controller.py` | `UR3Controller`, `RobotiqGripper` |
| `robot.transform` | `robot/transform.py` | `transform_robot_coordinates` — mm/deg → m/rad |
| `tools.robot_tools` | `tools/robot_tools.py` | `RobotActionTools` — pick/place sequences |

### Config files read at startup

| File | Purpose |
|------|---------|
| `aruco_config.json` | Marker IDs → robot XY (mm), marker size, ArUco dictionary |
| `homography_matrix.json` | Saved homography matrix H (written by calibration, read at startup) |
| `robot/zero_pose.json` | Reference pose for relative XY moves |
| `.env` | `GEMINI_API_KEY` |

---

## Coordinate conversion in the server

User-facing moves in mm/degrees are converted before sending to the robot:

```python
MM_SCALE = [0.001, -0.001, -0.001]   # mm→m, Y and Z axes inverted

def _mm_to_robot(x, y, z, rx_deg=0, ry_deg=0, rz_deg=0):
    return transform_robot_coordinates([[x, y, z, rx_deg, ry_deg, rz_deg]], scale=MM_SCALE)[0]
```

The Y and Z sign flip maps the UI display frame to the UR3's physical axis convention.

---

## Data flow

```
Browser
  │
  ├─ GET /stream ──────────────► BRIOCamera.capture_frame()
  │                                   │ [if _calib_overlay]
  │                                   └─► ArucoCalibrator.draw_detections()
  │
  ├─ POST /api/analyze ────────► BRIOCamera.capture_frame()
  │                                   │ [if _mask_workspace]
  │                                   ├─► _apply_workspace_mask()
  │                                   └─► detect_objects() ──► Gemini API
  │                                           └─► draw_boxes / draw_contours()
  │
  ├─ POST /api/calibrate/run ──► HomographyConverter.calibrate_aruco()
  │                                   ├─► ArucoCalibrator.calibrate()
  │                                   ├─► cv2.findHomography(RANSAC)
  │                                   ├─► homography_matrix.json (written)
  │                                   └─► _freeze_workspace_hull()
  │
  └─ POST /api/robot/pick_and_place
          ├─► convert_gemini_to_robot(ny, nx)   [pick coords]
          ├─► convert_gemini_to_robot(ny, nx)   [place coords]
          └─► RobotActionTools.pick_object_at() + place_object_at()
```

---

## Key globals (server.py)

| Variable | Type | Purpose |
|----------|------|---------|
| `_cam` | `BRIOCamera` | Camera instance; opened at startup |
| `_client` | `genai.Client` | Gemini client; `None` if API key missing |
| `_homography` | `HomographyConverter` | H matrix; loaded from file at startup |
| `_aruco` | `ArucoCalibrator\|None` | Lazy-init from `aruco_config.json` |
| `_robot` | `UR3Controller\|None` | `None` until `/api/robot/connect` |
| `_robot_tools` | `RobotActionTools\|None` | `None` until robot connected |
| `_workspace_hull` | `np.ndarray\|None` | Convex hull polygon; set after calibration |
| `_last_capture` | `bytes\|None` | JPEG bytes from last analysis |
| `_calib_overlay` | `bool` | ArUco overlay active in live feed |
| `_mask_workspace` | `bool` | Workspace masking active for Gemini |

See [docs/api.md](../docs/api.md) for the full endpoint reference.
