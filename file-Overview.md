# File Overview

Status of every file in the project relative to the main entry point (`web/server.py`).

## Legend

| Symbol | Meaning |
|--------|---------|
| **ACTIVE** | Loaded at runtime by `web/server.py` (directly or via transitive import) |
| **STANDALONE** | Runnable independently; not imported by the server |
| **TEST** | Test or diagnostic script; not used in production |
| **LEGACY** | Not imported anywhere; predates the current architecture |
| **CONFIG** | Data/config file read at runtime |
| **META** | Project metadata, documentation, tooling |

---

## Web layer

| File | Status | Notes |
|------|--------|-------|
| `web/server.py` | **ACTIVE** | Flask entry point — all API endpoints and global state |
| `web/templates/index.html` | **ACTIVE** | Single-page UI — rendered by Flask on `GET /` |
| `web/README.md` | META | Web server internals reference |

---

## Vision layer

| File | Status | Notes |
|------|--------|-------|
| `vision/camera.py` | **ACTIVE** | `BRIOCamera` — background-threaded BRIO frame capture |
| `vision/homography.py` | **ACTIVE** | `HomographyConverter` — calibrate H, pixel→robot coord conversion |
| `vision/aruco_calibrator.py` | **ACTIVE** | `ArucoCalibrator` — marker detection, builds homography correspondences |
| `vision/annotation.py` | **ACTIVE** | `draw_boxes` / `draw_contours` / `estimate_object_angle` |
| `vision/test_camera.py` | TEST | Standalone camera diagnostic; `python vision/test_camera.py` |
| `vision/__init__.py` | META | Empty package marker |

---

## AI layer

| File | Status | Notes |
|------|--------|-------|
| `ai/detection.py` | **ACTIVE** | `make_client` + `detect_objects` via Gemini API |
| `ai/gemini_agent.py` | **ACTIVE** | `GeminiAgent` — lazily imported by `/api/assistant/command` |
| `ai/mcp_server.py` | STANDALONE | FastMCP server wrapping the Flask API; run separately for Claude MCP access |
| `ai/__init__.py` | META | Empty package marker |

---

## Robot layer

| File | Status | Notes |
|------|--------|-------|
| `robot/ur3_controller.py` | **ACTIVE** | `UR3Controller` + `RobotiqGripper` — lazily imported on `/api/robot/connect` |
| `robot/transform.py` | **ACTIVE** | `transform_robot_coordinates` — mm/deg → m/rad scaling and rotation |
| `robot/robotiq_preamble.py` | **ACTIVE** | `ROBOTIQ_PREAMBLE` URScript constant — imported by `ur3_controller.py` |
| `robot/set_robot_zero.py` | STANDALONE | One-time setup: captures current TCP pose as work area centre |
| `robot/mcp_server.py` | STANDALONE | Early direct-robot MCP server on port 8001; superseded by `ai/mcp_server.py` |
| `robot/get_pose.py` | TEST | Prints current TCP pose — diagnostic utility |
| `robot/test_robot.py` | TEST | Integration test: move sequence + gripper open/close |
| `robot/test_mcp_server.py` | TEST | Tests `robot/mcp_server.py` via HTTP on port 8001 |
| `robot/calculate_distance.py` | LEGACY | Early pixel→robot linear scaling helper; superseded by homography pipeline |
| `robot/zero_pose.json` | CONFIG | Written by `set_robot_zero.py`; read by `UR3Controller` at connect |
| `robot/__init__.py` | META | Empty package marker |

---

## Tools layer

| File | Status | Notes |
|------|--------|-------|
| `tools/robot_tools.py` | **ACTIVE** | `RobotActionTools` — hover/grip/lift pick-and-place sequences |
| `tools/__init__.py` | META | Empty package marker |

---

## Configuration files

| File | Status | Notes |
|------|--------|-------|
| `.env` | CONFIG | `GEMINI_API_KEY` — never commit to git |
| `aruco_config.json` | CONFIG | Marker IDs → robot XY (mm), marker size, ArUco dict type |
| `homography_matrix.json` | CONFIG | Auto-generated 3×3 H matrix — do not edit manually |
| `pyproject.toml` | META | Project config and dependency declarations |
| `requirements.txt` | META | Python dependencies for pip users |
| `uv.lock` | META | uv dependency lock file |
| `.python-version` | META | Python version pin |

---

## Documentation

| File | Status | Notes |
|------|--------|-------|
| `README.md` | META | Quick start and first-time physical setup guide |
| `CLAUDE.md` | META | AI assistant codebase instructions |
| `docs/api.md` | META | All Flask endpoints with request/response shapes |
| `docs/architecture.md` | META | Component map and data flow diagrams |
| `docs/coordinates.md` | META | Coordinate space conversions in detail |
| `web/README.md` | META | Web server internals, globals, and data flow |

---

## Other files

| File | Status | Notes |
|------|--------|-------|
| `AM_samling_2.pdf` | META | Course assignment document |
| `camera_test.jpg` | META | Static test image; not read at runtime |
| `.claude/settings.local.json` | META | Claude Code local permissions |
