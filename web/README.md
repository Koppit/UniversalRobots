# web/

Flask-basert webserver for robot vision-kontroll. Gir live kamerafeed, Gemini-gjenkjenning og ArUco-kalibrering via nettleser.

## Kjøring

```
.venv\Scripts\python web\server.py
```

Åpne nettleser: http://localhost:5000

## Filstruktur

```
web/
├── server.py          # Flask-applikasjon og alle API-endepunkter
└── templates/
    └── index.html     # Enkeltside-UI (vanilla HTML/CSS/JS, ingen build-steg)
```

## Avhengigheter

### Prosjektmoduler server.py importerer

| Modul | Fil | Hva som brukes |
|-------|-----|----------------|
| `ai.detection` | `ai/detection.py` | `make_client`, `detect_objects` — Gemini-klient og bildeanalyse |
| `vision.camera` | `vision/camera.py` | `BRIOCamera` — kameraopptak i bakgrunnstråd |
| `vision.homography` | `vision/homography.py` | `HomographyConverter` — piksel → robot XY-konvertering |
| `vision.aruco_calibrator` | `vision/aruco_calibrator.py` | `ArucoCalibrator` — ArUco-deteksjon og homografi-bygging |
| `vision.annotation` | `vision/annotation.py` | `draw_boxes`, `draw_contours` — OpenCV-annotasjon av deteksjoner |

### Konfigurasjonsfiler

| Fil | Beskrivelse |
|-----|-------------|
| `aruco_config.json` | Markør-ID → robot XY (meter), markørstørrelse og ArUco-ordbok |
| `homography_matrix.json` | Lagret homografi-matrise H (skrives av kalibrering, leses ved oppstart) |
| `.env` | `GEMINI_API_KEY` |

### Python-pakker

| Pakke | Brukes til |
|-------|-----------|
| `flask` | HTTP-server og template-rendering |
| `opencv-contrib-python` | MJPEG-encoding, ArUco-deteksjon, bildeprosessering |
| `numpy` | Matrise- og piksel-operasjoner |
| `python-dotenv` | Laste `.env` ved oppstart |
| `google-genai` | Gemini API-klient (via `ai.detection`) |

## API-endepunkter

### Visning

| Endepunkt | Metode | Beskrivelse |
|-----------|--------|-------------|
| `/` | GET | Serverer `index.html` |
| `/stream` | GET | MJPEG live-kamerafeed (multipart/x-mixed-replace) |
| `/api/last_capture` | GET | Siste analyserte bilde som JPEG |
| `/api/status` | GET | `{msg, busy, has_capture}` |

### Gjenkjenning

| Endepunkt | Metode | Body | Beskrivelse |
|-----------|--------|------|-------------|
| `/api/analyze` | POST | `{"mode": "bbox"\|"grabcut"}` | Fanger bilde, sender til Gemini, returnerer deteksjoner og lagrer annotert bilde |

### Kalibrering

| Endepunkt | Metode | Beskrivelse |
|-----------|--------|-------------|
| `/api/calibrate/run` | POST | Kjører ArUco-kalibrering, fryser arbeidsområde-polygon, lagrer `homography_matrix.json` |
| `/api/calibrate/status` | GET | `{detected, missing, needed, calibrated}` — hvilke markører kameraet ser nå |
| `/api/calibrate/preview` | GET | JPEG med markør-omriss tegnet på (for kalibreringsfeltet i UI) |
| `/api/calibrate/overlay` | POST | Toggle: vis/skjul markør-overlay i live-feeden |
| `/api/workspace/toggle` | POST | Toggle: masker arbeidsområdet i bilder sendt til Gemini |

## Tilstandsvariabler (server.py)

| Variabel | Type | Beskrivelse |
|----------|------|-------------|
| `_cam` | `BRIOCamera` | Kamerainstans, åpnes ved oppstart |
| `_client` | `genai.Client` | Gemini-klient, `None` hvis API-nøkkel mangler |
| `_homography` | `HomographyConverter` | Holder homografi-matrisen H |
| `_aruco` | `ArucoCalibrator\|None` | Lazy-initialisert fra `aruco_config.json` |
| `_workspace_hull` | `ndarray\|None` | Cachet polygon (piksler) — settes ved kalibrering, endres ikke under drift |
| `_last_capture` | `bytes\|None` | JPEG-bytes fra siste analyse |
| `_calib_overlay` | `bool` | Om markør-overlay er aktiv i live-feed |
| `_mask_workspace` | `bool` | Om arbeidsområde-masking er aktiv |

## Dataflyt

```
Nettleser
  │
  ├─ GET /stream ──────────────────► BRIOCamera.capture_frame()
  │                                       │ [hvis _calib_overlay]
  │                                       └─► ArucoCalibrator.draw_detections()
  │
  ├─ POST /api/analyze ────────────► BRIOCamera.capture_frame()
  │                                       │ [hvis _mask_workspace]
  │                                       ├─► _apply_workspace_mask()
  │                                       └─► ai.detection.detect_objects() ──► Gemini API
  │                                               └─► vision.annotation.draw_boxes/draw_contours()
  │
  └─ POST /api/calibrate/run ──────► HomographyConverter.calibrate_aruco()
                                          ├─► ArucoCalibrator.calibrate()
                                          ├─► cv2.findHomography()
                                          ├─► homography_matrix.json (skrives)
                                          └─► _freeze_workspace_hull()
```
