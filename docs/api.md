# API Reference

Base URL: `http://localhost:5000`

All POST endpoints accept and return JSON. Errors return `{"error": "message"}` with an appropriate HTTP status.

---

## Display / status

| Method | Route | Body | Response |
|--------|-------|------|----------|
| GET | `/` | — | `index.html` |
| GET | `/stream` | — | MJPEG live camera stream |
| GET | `/api/last_capture` | — | JPEG bytes (204 if no capture yet) |
| GET | `/api/status` | — | `{msg, busy, has_capture, robot_connected, robot_busy}` |
| GET | `/api/logs` | `?since=<int>` | `{entries: [{ts,level,msg}], total}` |

`busy` is true while Gemini analysis is running. `robot_busy` is true while a pick-and-place is executing.

---

## Object detection / analysis

| Method | Route | Body | Response |
|--------|-------|------|----------|
| POST | `/api/analyze` | `{mode}` | `{count, detections}` |

`mode`: `"bbox"` (bounding boxes) or `"grabcut"` (contour segmentation).

Detection item shape:
```json
{
  "label": "red cube",
  "box_2d": [y_min, x_min, y_max, x_max],
  "rx_mm": 123.4,
  "ry_mm": 56.7
}
```
`box_2d` is in **0–1000 normalised** Gemini coordinates (Y first). `rx_mm` / `ry_mm` are robot XY in mm — only present when calibrated.

---

## Calibration

| Method | Route | Body | Response |
|--------|-------|------|----------|
| GET | `/api/calibrate/status` | — | `{detected, missing, needed, ready, calibrated}` |
| GET | `/api/calibrate/preview` | — | JPEG with ArUco overlays drawn |
| POST | `/api/calibrate/run` | — | `{success, markers_used}` or `{success:false, error}` |
| POST | `/api/calibrate/overlay` | — | `{overlay: bool}` — toggles ArUco overlay on live feed |
| POST | `/api/workspace/toggle` | — | `{mask: bool}` — toggles workspace mask for Gemini analysis |

`/api/calibrate/run` runs `HomographyConverter.calibrate_aruco()`, saves `homography_matrix.json`, and freezes the workspace polygon from detected marker corners.

---

## Coordinate preview

| Method | Route | Body | Response |
|--------|-------|------|----------|
| POST | `/api/preview_coords` | `{ny, nx}` | `{calibrated, rx_mm, ry_mm}` |

Converts a point in **0–1000 normalised** space to robot XY mm. Returns `{calibrated: false}` if no homography is loaded.

---

## Camera selection

| Method | Route | Body | Response |
|--------|-------|------|----------|
| GET | `/api/cameras` | — | `{cameras: [{index, label}], current}` |
| POST | `/api/cameras/select` | `{index}` | `{ok: true, index}` or `{error}` |

Switching cameras closes the current capture and reopens on the new device index.

---

## Robot control

| Method | Route | Body | Response |
|--------|-------|------|----------|
| POST | `/api/robot/connect` | — | `{connected: true}` or `{error}` |
| POST | `/api/robot/disconnect` | — | `{connected: false}` |
| POST | `/api/robot/stop` | — | `{ok: true}` — emergency stop |
| POST | `/api/robot/move` | `{x,y,z,rx,ry,rz}` | `{ok: true}` or `{error}` |
| POST | `/api/robot/pick_and_place` | `{pick:[ny,nx], place:[ny,nx]}` | `{status:"started"}` or `{error}` |

**`/api/robot/move`** coordinates are in **mm and degrees** (the server converts to metres/radians via `_mm_to_robot()`). All six fields are required.

**`/api/robot/pick_and_place`** takes Gemini-normalised `[ny, nx]` (0–1000) for both pick and place. The operation runs in a background thread — poll `/api/status` for `robot_busy` and `msg` to track progress.

Prerequisites for pick_and_place:
- Robot connected
- Homography calibrated

---

## AI assistant

| Method | Route | Body | Response |
|--------|-------|------|----------|
| POST | `/api/assistant/command` | `{text}` | `{status:"started"}` or `{error}` |

Runs the `GeminiAgent` with the given natural-language command. Executes in background; poll `/api/status`.
