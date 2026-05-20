# Coordinate Systems

Three coordinate spaces are used in this system. Understanding how they relate is essential for calibration and debugging.

---

## The three spaces

| Space | Range | Origin | Used by |
|-------|-------|--------|---------|
| **Pixel** | 0–1280 × 0–720 | top-left | OpenCV, camera frames, ArUco detection |
| **Gemini normalised** | 0–1000 × 0–1000 | top-left | Gemini API responses, UI click coords |
| **Robot (metres)** | varies | robot base + zero pose | UR3Controller, RobotActionTools |

Note: Gemini uses `[y_min, x_min, y_max, x_max]` order in `box_2d` (Y before X).

---

## Conversions

### Pixel → Gemini normalised
```python
nx = x_pixel / frame_width  * 1000
ny = y_pixel / frame_height * 1000
```

### Gemini normalised → Robot metres
Done by `HomographyConverter.convert_gemini_to_robot(ny, nx)`:
```python
px = (nx / 1000) * cam_width
py = (ny / 1000) * cam_height
result = cv2.perspectiveTransform([[px, py]], H)
rx, ry = result[0][0]   # metres
```
`H` is the 3×3 homography matrix computed during ArUco calibration.

### Robot metres → display mm
```python
rx_mm = rx * 1000
ry_mm = ry * 1000
```
The web UI always displays coordinates in mm.

### UI click → Gemini normalised (browser JS)
```js
const img  = document.getElementById('live-img');
const rect = img.getBoundingClientRect();
const nx = Math.round((e.clientX - rect.left) / rect.width  * 1000);
const ny = Math.round((e.clientY - rect.top)  / rect.height * 1000);
```

### UI mm → robot move
The `/api/robot/move` endpoint accepts mm and degrees; `_mm_to_robot()` converts:
```python
MM_SCALE = [0.001, -0.001, -0.001]   # mm→m, Y and Z axes inverted
```
The Y and Z sign inversion maps the web UI's display frame to the UR3's physical axis directions.

---

## Zero pose and relative coordinates

All XY coordinates shown in the UI are **relative to the zero pose** defined by `set_robot_zero.py`. This is the TCP position saved in `robot/zero_pose.json`.

ArUco marker positions in `aruco_config.json` are also in this relative frame:

```
(-435, -285) ──────── (+435, -285)
      │                      │
      │   work area centre   │    ← zero pose (0, 0)
      │                      │
(-435, +285) ──────── (+435, +285)
```
Units: mm. X is lateral, Y is depth. Z is height above the table.

---

## Z heights

| Constant | Value | Location | Purpose |
|----------|-------|----------|---------|
| `z_height_hover` | 0.150 m | `RobotActionTools` | Travel height between pick and place |
| `z_height_pick` | 0.050 m | `RobotActionTools` | Height at which gripper closes/opens |
| Verify Z default | 0.050 m | Web UI input | Safe hover for coordinate verification moves |

---

## box_2d format (Gemini)

```
[y_min, x_min, y_max, x_max]   ← Y first
```

Object centre in normalised space:
```python
ny = (box[0] + box[2]) / 2
nx = (box[1] + box[3]) / 2
```

The centre is passed to `HomographyConverter.convert_gemini_to_robot(ny, nx)` and to `RobotActionTools.pick_object_at(ny, nx)`.
