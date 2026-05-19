# Coordinate systems

## The three spaces

| Space | Range | Origin | Used by |
|-------|-------|--------|---------|
| **Pixel** | 0–1280 × 0–720 | top-left | OpenCV, camera frames |
| **Gemini normalised** | 0–1000 × 0–1000 | top-left | Gemini API responses, UI click coords |
| **Robot (metres)** | ~0.1–0.5 × ~0.0–0.4 | robot base | UR3Controller, RobotActionTools |

## Conversions

### Pixel → Gemini normalised
```python
nx = x_pixel / frame_width  * 1000
ny = y_pixel / frame_height * 1000
```

### Gemini normalised → Robot metres
Done by `HomographyConverter.convert_gemini_to_robot(ny, nx)` using the 3×3 homography matrix H.
H is computed from ArUco marker correspondences (pixel centres → known robot XY positions).

### UI click → Gemini normalised (browser JS)
```js
const nx = (clickX - imgRect.left) / imgRect.width  * 1000;
const ny = (clickY - imgRect.top)  / imgRect.height * 1000;
```
The analysed image is served at full camera resolution, so displayed size cancels out.

## Z heights (metres, above table surface)

| Constant | Value | Purpose |
|----------|-------|---------|
| `z_height_hover` | 0.150 | travel height between points |
| `z_height_pick` | 0.050 | grip / release height |

Both defined in `RobotActionTools` (`tools/robot_tools.py`).

## box_2d format (Gemini)
```
[y_min, x_min, y_max, x_max]   ← note: Y first
```
Object centre:
```python
ny = (box[0] + box[2]) / 2
nx = (box[1] + box[3]) / 2
```
