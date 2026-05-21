import base64
import hashlib
import sys
import subprocess
import time
import threading
import urllib.error
import urllib.request
import cv2
import numpy as np


class WristCamera:
    """Robotiq/UR wrist camera feed fetched from the robot HTTP image endpoint."""

    def __init__(self, robot_ip: str, timeout_s: float = 1.0, fps: int = 10):
        self.robot_ip = robot_ip
        self.timeout_s = timeout_s
        self.fps = fps
        self.url = f"http://{robot_ip}:4242/current.jpg?type=color"
        self.device_index = "wrist"
        self.width = 1280
        self.height = 720
        self._latest_frame: np.ndarray | None = None
        self._last_frame_at = 0.0
        self._frame_count = 0
        self._read_failures = 0
        self._last_error: str | None = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None

    def open(self) -> bool:
        frame = self._fetch_frame()
        if frame is None:
            print(f"[WristCam] FEIL: Kan ikke hente bilde fra {self.url}: {self._last_error}")
            return False

        self.height, self.width = frame.shape[:2]
        with self._lock:
            self._latest_frame = frame
            self._last_frame_at = time.monotonic()
            self._frame_count = 1
            self._read_failures = 0
        print(f"[WristCam] Åpnet {self.url} ({self.width}x{self.height})")

        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        return True

    def close(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        print("[WristCam] Lukket.")

    def _fetch_frame(self) -> np.ndarray | None:
        try:
            with urllib.request.urlopen(self.url, timeout=self.timeout_s) as response:
                if response.status != 200:
                    self._last_error = f"HTTP {response.status}"
                    return None
                data = response.read()
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            self._last_error = str(exc)
            return None

        img_array = np.asarray(bytearray(data), dtype=np.uint8)
        frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        if frame is None or frame.size == 0:
            self._last_error = "Ugyldig JPEG-data"
            return None
        self._last_error = None
        return frame

    def _capture_loop(self):
        delay_s = 1.0 / max(1, self.fps)
        while self._running:
            frame = self._fetch_frame()
            if frame is not None:
                self.height, self.width = frame.shape[:2]
                with self._lock:
                    self._latest_frame = frame
                    self._last_frame_at = time.monotonic()
                    self._frame_count += 1
                    self._read_failures = 0
            else:
                with self._lock:
                    self._read_failures += 1
            time.sleep(delay_s)

    def capture_frame(self) -> np.ndarray | None:
        with self._lock:
            return self._latest_frame.copy() if self._latest_frame is not None else None

    def stats(self) -> dict:
        with self._lock:
            age = time.monotonic() - self._last_frame_at if self._last_frame_at else None
            return {
                "device_index": self.device_index,
                "backend": "HTTP wristcam",
                "running": self._running,
                "frame_count": self._frame_count,
                "last_frame_age_s": round(age, 3) if age is not None else None,
                "read_failures": self._read_failures,
                "url": self.url,
                "last_error": self._last_error,
            }

    def capture_jpeg_b64(self, quality: int = 85) -> str | None:
        frame = self.capture_frame()
        if frame is None:
            return None
        _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return base64.b64encode(buf).decode("utf-8")


class BRIOCamera:
    """Logitech BRIO kameraopptak via OpenCV med bakgrunnstråd for frames.

    Bruk device_index=None (default) for automatisk deteksjon av BRIO.
    """

    def __init__(self, device_index: int | None = None, width: int = 1280, height: int = 720, fps: int = 30):
        self.device_index = device_index  # None = auto-detect ved open()
        self.width = width
        self.height = height
        self.fps = fps
        self._cap: cv2.VideoCapture | None = None
        self._backend_name: str | None = None
        self._latest_frame: np.ndarray | None = None
        self._last_frame_at = 0.0
        self._frame_count = 0
        self._read_failures = 0
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None

    @staticmethod
    def _candidate_backends() -> list[tuple[str, int]]:
        """Return OpenCV camera backends in a stable, Windows-friendly order."""
        if sys.platform.startswith("win"):
            return [
                ("DSHOW", cv2.CAP_DSHOW),
                ("MSMF", cv2.CAP_MSMF),
                ("ANY", cv2.CAP_ANY),
            ]
        return [("ANY", cv2.CAP_ANY)]

    @staticmethod
    def _open_capture(index: int, backend: int) -> cv2.VideoCapture:
        cap = cv2.VideoCapture()
        try:
            cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 2000)
            cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 2000)
        except Exception:
            pass
        if backend == cv2.CAP_ANY:
            cap.open(index)
        else:
            cap.open(index, backend)
        return cap

    @staticmethod
    def _configure_capture(cap: cv2.VideoCapture, width: int, height: int, fps: int | None = None) -> None:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if fps is not None:
            cap.set(cv2.CAP_PROP_FPS, fps)

    @staticmethod
    def _read_frame(cap: cv2.VideoCapture, attempts: int = 10, delay_s: float = 0.03) -> np.ndarray | None:
        for _ in range(attempts):
            ret, frame = cap.read()
            if ret and frame is not None and frame.size > 0:
                return frame
            time.sleep(delay_s)
        return None

    # ------------------------------------------------------------------
    # Auto-deteksjon
    # ------------------------------------------------------------------

    @staticmethod
    def list_cameras() -> list[str]:
        """Returnerer navn på tilkoblede kameraer via Windows WMI.

        Rask — åpner ingen kamera og leser ingen frames.
        Merk: WMI-rekkefølge samsvarer ikke alltid med OpenCV-indeks.
        Bruk find_brio() for å finne riktig indeks automatisk.
        """
        cmd = (
            "Get-PnpDevice -Status OK "
            "| Where-Object { $_.Class -in @('Camera','Image') } "
            "| Select-Object -ExpandProperty FriendlyName"
        )
        try:
            result = subprocess.run(
                ["powershell", "-NoProfile", "-Command", cmd],
                capture_output=True, text=True, timeout=5
            )
            names = [l.strip() for l in result.stdout.splitlines() if l.strip()]
        except Exception:
            names = []

        if names:
            for name in names:
                print(f"  - {name}")
        else:
            print("  Ingen kameraer funnet.")
        return names

    @staticmethod
    def find_brio(max_index: int = 4, sample_frames: int = 15) -> int | None:
        """Skanner kamera-indekser og returnerer den med flest unike frames.

        BRIO leverer kontinuerlig video → høyest andel unike frames.
        Innebygde/trege kameraer gjenbruker frames og scorer lavt.
        """
        print("[Kamera] Søker etter BRIO...")
        best_index  = None
        best_unique = -1

        for idx in range(max_index):
            for backend_name, backend in BRIOCamera._candidate_backends():
                cap = BRIOCamera._open_capture(idx, backend)
                if not cap.isOpened():
                    cap.release()
                    continue

                BRIOCamera._configure_capture(cap, 1280, 720)

                hashes = []
                for _ in range(sample_frames):
                    frame = BRIOCamera._read_frame(cap, attempts=1, delay_s=0.0)
                    if frame is not None:
                        hashes.append(hashlib.md5(frame.tobytes()).hexdigest())
                    time.sleep(0.04)

                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                cap.release()
                time.sleep(0.2)

                unique = len(set(hashes))
                print(f"  Index {idx} ({backend_name}): {unique}/{len(hashes)} unike frames  ({w}x{h})")

                if unique > best_unique:
                    best_unique = unique
                    best_index = idx
                if unique > 0:
                    break
            if best_unique >= sample_frames:
                break

        if best_index is not None:
            print(f"[Kamera] BRIO funnet på index {best_index} ({best_unique}/{sample_frames} unike frames)")
        else:
            print("[Kamera] Ingen kamera funnet.")
        return best_index

    # ------------------------------------------------------------------
    # Åpne / lukke
    # ------------------------------------------------------------------

    def _connect_capture(self) -> tuple[str, cv2.VideoCapture, np.ndarray] | None:
        if self.device_index is None:
            return None
        for backend_name, backend in self._candidate_backends():
            cap = self._open_capture(self.device_index, backend)
            if not cap.isOpened():
                cap.release()
                continue

            self._configure_capture(cap, self.width, self.height, self.fps)
            frame = self._read_frame(cap, attempts=10)
            if frame is not None:
                return backend_name, cap, frame

            cap.release()
        return None

    def open(self) -> bool:
        if self.device_index is None:
            self.device_index = self.find_brio()
            if self.device_index is None:
                return False

        selected = self._connect_capture()
        if selected is None:
            print(f"[Kamera] FEIL: Kan ikke åpne enhet {self.device_index}, eller den leverer ingen frames.")
            return False

        backend_name, cap, frame = selected
        self._cap = cap
        self._backend_name = backend_name
        with self._lock:
            self._latest_frame = frame
            self._last_frame_at = time.monotonic()
            self._frame_count = 1
            self._read_failures = 0
        actual_w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        print(f"[Kamera] Åpnet enhet {self.device_index} via {backend_name} ({actual_w}x{actual_h} @ {actual_fps:.0f}fps)")

        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        return True

    def _reopen_capture(self) -> bool:
        old_cap = self._cap
        if old_cap and old_cap.isOpened():
            old_cap.release()

        selected = self._connect_capture()
        if selected is None:
            return False

        backend_name, cap, frame = selected
        self._cap = cap
        self._backend_name = backend_name
        with self._lock:
            self._latest_frame = frame
            self._last_frame_at = time.monotonic()
            self._frame_count += 1
            self._read_failures = 0
        print(f"[Kamera] Koblet til kamera på nytt via {backend_name}.")
        return True

    def close(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        if self._cap and self._cap.isOpened():
            self._cap.release()
            cv2.destroyAllWindows()
            print("[Kamera] Lukket.")

    # ------------------------------------------------------------------
    # Frame-henting
    # ------------------------------------------------------------------

    def _capture_loop(self):
        """Bakgrunnstråd — leser frames kontinuerlig så hovedtråden aldri blokkerer."""
        while self._running:
            if not self._cap or not self._cap.isOpened():
                print("[Kamera] Kamerahandle er lukket. Prøver å åpne på nytt...")
                if not self._reopen_capture():
                    time.sleep(1.0)
                continue

            try:
                ret, frame = self._cap.read()
            except cv2.error as exc:
                print(f"[Kamera] FEIL ved lesing av frame: {exc}")
                ret, frame = False, None

            if ret and frame is not None and frame.size > 0:
                with self._lock:
                    self._latest_frame = frame
                    self._last_frame_at = time.monotonic()
                    self._frame_count += 1
                    self._read_failures = 0
            else:
                with self._lock:
                    self._read_failures += 1
                    failures = self._read_failures
                if failures >= max(30, self.fps * 2):
                    print(f"[Kamera] Ingen nye frames etter {failures} forsøk. Åpner kamera på nytt...")
                    if self._reopen_capture():
                        continue
                    with self._lock:
                        self._read_failures = 0
                time.sleep(0.03)

    def capture_frame(self) -> np.ndarray | None:
        """Returnerer siste frame fra bakgrunnstråden (blokkerer aldri)."""
        with self._lock:
            return self._latest_frame.copy() if self._latest_frame is not None else None

    def stats(self) -> dict:
        """Returnerer enkel kamerastatus for web/status og feilsøking."""
        with self._lock:
            age = time.monotonic() - self._last_frame_at if self._last_frame_at else None
            return {
                "device_index": self.device_index,
                "backend": self._backend_name,
                "running": self._running,
                "frame_count": self._frame_count,
                "last_frame_age_s": round(age, 3) if age is not None else None,
                "read_failures": self._read_failures,
            }

    def capture_jpeg_b64(self, quality: int = 85) -> str | None:
        """Fanger ett bilde og returnerer base64-kodet JPEG-string for Gemini API."""
        frame = self.capture_frame()
        if frame is None:
            return None
        _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return base64.b64encode(buf).decode("utf-8")

    def show_preview(self, window_name: str = "BRIO Preview - trykk Q for aa avslutte"):
        """Viser live-kameravisning i et OpenCV-vindu."""
        if not self._running:
            print("[Kamera] Ikke åpnet — kall open() først.")
            return
        while True:
            frame = self.capture_frame()
            if frame is None:
                continue
            cv2.imshow(window_name, frame)
            if cv2.waitKey(30) & 0xFF == ord("q"):
                break
        cv2.destroyWindow(window_name)
