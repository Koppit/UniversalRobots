import base64
import hashlib
import subprocess
import time
import threading
import cv2
import numpy as np


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
        self._latest_frame: np.ndarray | None = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None

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
            cap = cv2.VideoCapture()
            cap.open(idx + cv2.CAP_MSMF)
            if not cap.isOpened():
                cap.release()
                continue

            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

            hashes = []
            for _ in range(sample_frames):
                ret, frame = cap.read()
                if ret and frame is not None and frame.size > 0:
                    hashes.append(hashlib.md5(frame).hexdigest())
                time.sleep(0.04)

            cap.release()
            time.sleep(0.2)

            unique = len(set(hashes))
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"  Index {idx}: {unique}/{len(hashes)} unike frames  ({w}x{h})")

            if unique > best_unique:
                best_unique = unique
                best_index  = idx

        if best_index is not None:
            print(f"[Kamera] BRIO funnet på index {best_index} ({best_unique}/{sample_frames} unike frames)")
        else:
            print("[Kamera] Ingen kamera funnet.")
        return best_index

    # ------------------------------------------------------------------
    # Åpne / lukke
    # ------------------------------------------------------------------

    def open(self) -> bool:
        if self.device_index is None:
            self.device_index = self.find_brio()
            if self.device_index is None:
                return False

        cap = cv2.VideoCapture()
        cap.open(self.device_index + cv2.CAP_MSMF)
        if not cap.isOpened():
            print(f"[Kamera] FEIL: Kan ikke åpne enhet {self.device_index}.")
            return False

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)

        ok = False
        for _ in range(10):
            ret, frame = cap.read()
            if ret and frame is not None and frame.size > 0:
                ok = True
                break
        if not ok:
            cap.release()
            print(f"[Kamera] FEIL: Enhet {self.device_index} åpnet men leverer ingen frames.")
            return False

        self._cap = cap
        actual_w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        print(f"[Kamera] Åpnet enhet {self.device_index} ({actual_w}x{actual_h} @ {actual_fps:.0f}fps)")

        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
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
        while self._running and self._cap and self._cap.isOpened():
            ret, frame = self._cap.read()
            if ret and frame is not None and frame.size > 0:
                with self._lock:
                    self._latest_frame = frame

    def capture_frame(self) -> np.ndarray | None:
        """Returnerer siste frame fra bakgrunnstråden (blokkerer aldri)."""
        with self._lock:
            return self._latest_frame.copy() if self._latest_frame is not None else None

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
