# API endpoints

Base: `http://localhost:5000`

## Vision / analysis

| Method | Route | Body | Response |
|--------|-------|------|----------|
| GET | `/` | — | index.html |
| GET | `/stream` | — | MJPEG stream |
| GET | `/api/last_capture` | — | JPEG (204 if none) |
| POST | `/api/analyze` | `{mode: "bbox"\|"grabcut"}` | `{count, detections}` |
| GET | `/api/status` | — | `{msg, busy, has_capture, robot_connected, robot_busy}` |

`detections` item shape:
```json
{"label": "cube", "box_2d": [y_min, x_min, y_max, x_max]}
```
Coordinates are **0-1000 normalised** (Gemini format).

## Calibration

| Method | Route | Body | Response |
|--------|-------|------|----------|
| GET | `/api/calibrate/status` | — | `{detected, missing, needed, calibrated}` |
| GET | `/api/calibrate/preview` | — | JPEG with ArUco overlays |
| POST | `/api/calibrate/run` | — | `{success, markers_used}` or `{success:false, error}` |
| POST | `/api/calibrate/overlay` | — | `{overlay: bool}` |
| POST | `/api/workspace/toggle` | — | `{mask: bool}` |

## Robot

| Method | Route | Body | Response |
|--------|-------|------|----------|
| POST | `/api/robot/connect` | — | `{connected: true}` or `{error}` |
| POST | `/api/robot/disconnect` | — | `{connected: false}` |
| POST | `/api/robot/pick_and_place` | `{pick:[ny,nx], place:[ny,nx]}` | `{status:"started"}` or `{error}` |

`pick_and_place` runs in a background thread. Poll `/api/status` for progress (`robot_busy`, `msg`).

**Prerequisites for pick_and_place:**
- Robot connected (`/api/robot/connect`)
- Homography calibrated (`/api/calibrate/run`)
- Both `pick` and `place` are `[ny, nx]` in 0-1000 normalised space
