"""Quick camera-output diagnostic for the BRIOCamera wrapper.

Usage examples:
    python vision/test_camera.py
    python vision/test_camera.py --device 0 --save camera_test.jpg
    python vision/test_camera.py --preview
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from vision.camera import BRIOCamera  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test camera output from BRIOCamera.")
    parser.add_argument(
        "--device",
        type=int,
        default=None,
        help="OpenCV camera index. Omit to auto-detect the BRIO.",
    )
    parser.add_argument("--width", type=int, default=1280, help="Requested frame width.")
    parser.add_argument("--height", type=int, default=720, help="Requested frame height.")
    parser.add_argument("--fps", type=int, default=30, help="Requested camera FPS.")
    parser.add_argument(
        "--warmup",
        type=float,
        default=1.0,
        help="Seconds to wait after opening before sampling frames.",
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=30,
        help="Number of frames to sample for diagnostics.",
    )
    parser.add_argument(
        "--save",
        type=Path,
        default=ROOT / "camera_test.jpg",
        help="Path to save one JPEG snapshot. Use --no-save to skip.",
    )
    parser.add_argument(
        "--no-save",
        action="store_true",
        help="Do not save a snapshot.",
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="Show live preview window. Press Q to close.",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=85,
        choices=range(1, 101),
        metavar="1-100",
        help="JPEG quality used for snapshot/base64 test.",
    )
    return parser.parse_args()


def wait_for_frame(camera: BRIOCamera, timeout_s: float) -> np.ndarray | None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        frame = camera.capture_frame()
        if frame is not None:
            return frame
        time.sleep(0.02)
    return None


def sample_frames(camera: BRIOCamera, count: int) -> list[np.ndarray]:
    frames: list[np.ndarray] = []
    for _ in range(count):
        frame = wait_for_frame(camera, timeout_s=1.0)
        if frame is not None:
            frames.append(frame)
        time.sleep(0.03)
    return frames


def print_frame_stats(frames: list[np.ndarray]) -> None:
    if not frames:
        print("[Test] FEIL: Ingen frames mottatt fra kamera.")
        return

    first = frames[0]
    hashes = {frame.tobytes()[:4096] for frame in frames}
    means = [float(frame.mean()) for frame in frames]
    stds = [float(frame.std()) for frame in frames]

    print("[Test] Kamera leverer frames.")
    print(f"  Opploesning: {first.shape[1]}x{first.shape[0]}")
    print(f"  Kanaler:     {first.shape[2] if first.ndim == 3 else 1}")
    print(f"  Dtype:       {first.dtype}")
    print(f"  Samples:     {len(frames)}")
    print(f"  Variasjon:   {len(hashes)}/{len(frames)} ulike frame-signaturer")
    print(f"  Mean:        {min(means):.1f}..{max(means):.1f}")
    print(f"  Stddev:      {min(stds):.1f}..{max(stds):.1f}")
    print(f"  Min/Max:     {int(first.min())}/{int(first.max())}")


def save_snapshot(path: Path, frame: np.ndarray, quality: int) -> bool:
    path.parent.mkdir(parents=True, exist_ok=True)
    ok = cv2.imwrite(
        str(path),
        frame,
        [cv2.IMWRITE_JPEG_QUALITY, quality],
    )
    if ok:
        print(f"[Test] Snapshot lagret: {path}")
    else:
        print(f"[Test] FEIL: Klarte ikke aa lagre snapshot: {path}")
    return ok


def main() -> int:
    args = parse_args()

    print("[Test] Tilkoblede kameraer:")
    BRIOCamera.list_cameras()

    camera = BRIOCamera(
        device_index=args.device,
        width=args.width,
        height=args.height,
        fps=args.fps,
    )

    try:
        if not camera.open():
            print("[Test] FEIL: Kamera kunne ikke aapnes.")
            return 1

        time.sleep(max(0.0, args.warmup))
        frames = sample_frames(camera, args.frames)
        print_frame_stats(frames)

        if not frames:
            return 1

        jpeg_b64 = camera.capture_jpeg_b64(quality=args.jpeg_quality)
        if jpeg_b64:
            print(f"[Test] JPEG/base64 OK ({len(jpeg_b64)} tegn).")
        else:
            print("[Test] FEIL: JPEG/base64 test feilet.")
            return 1

        if not args.no_save:
            save_snapshot(args.save, frames[-1], args.jpeg_quality)

        if args.preview:
            print("[Test] Viser preview. Trykk Q for aa avslutte.")
            camera.show_preview()

        return 0
    finally:
        camera.close()


if __name__ == "__main__":
    raise SystemExit(main())
