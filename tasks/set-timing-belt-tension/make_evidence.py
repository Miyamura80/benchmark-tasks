"""Encode recorded robot camera frames with the logged mechanism measurements."""
import argparse
import json
import subprocess
from pathlib import Path

import cv2
import numpy as np


def label(canvas, text, x, y, size=0.45, color=(232, 239, 245)):
    cv2.putText(canvas, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, size, color, 1, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("trial", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--title", default="Scripted YAM demonstration")
    args = parser.parse_args()
    streams = [sorted(args.trial.glob(f"frames/*/*~{camera}_cam_*.npy"))
               for camera in ("top", "left", "right")]
    if not streams[0] or len({len(stream) for stream in streams}) != 1:
        raise ValueError("Missing or mismatched saved camera streams")
    trace = [json.loads(line) for line in (args.trial / "mechanism.jsonl").read_text().splitlines()]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    ffmpeg = [
        "ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo",
        "-pixel_format", "rgb24", "-video_size", "640x438", "-framerate", "10", "-i", "-",
        "-c:v", "libx264", "-preset", "fast", "-crf", "27", "-pix_fmt", "yuv420p",
        "-movflags", "+faststart", str(args.output),
    ]
    with subprocess.Popen(ffmpeg, stdin=subprocess.PIPE) as encoder:
        for index in range(0, len(streams[0]), 2):
            frame = np.full((438, 640, 3), (22, 30, 42), dtype=np.uint8)
            state = trace[min(index, len(trace) - 1)]
            top, left, right = (np.load(stream[index])[..., :3] for stream in streams)
            frame[34:246, 128:512] = cv2.resize(top, (384, 212), interpolation=cv2.INTER_AREA)
            for x, camera in [(0, left), (320, right)]:
                frame[266:414, x:x + 320] = cv2.resize(camera, (320, 148), interpolation=cv2.INTER_AREA)
            label(frame, args.title, 12, 22, 0.54)
            label(frame, f"{state['t']:05.2f} s", 567, 22)
            label(frame, "OVERVIEW", 135, 52)
            label(frame, "LEFT WRIST", 12, 261)
            label(frame, "RIGHT WRIST", 331, 261)
            a, b = state["clamp_n"]
            gauge = "--" if state["gauge_mm"] is None else f"{state['gauge_mm']:.2f} mm"
            if state["lever_arm"] is not None and state["socket"] is not None:
                phase = f"Holding plate; turning bolt {state['socket'] + 1}"
            elif state["lever_arm"] is not None:
                phase = "Holding plate"
            elif state["test_loaded"]:
                phase = "Gauge loaded after plate release"
            elif state["gauge_mm"] is not None:
                phase = f"Gauge reading retained; score {state['score']:g}"
            else:
                phase = "Plate free"
            label(frame, "Simulator readings", 9, 76, 0.37)
            label(frame, f"{state['span_mm']:.2f} mm span", 9, 101, 0.37)
            label(frame, f"{a:.0f} / {b:.0f} N clamp", 9, 124, 0.37)
            label(frame, "2.5 N gauge", 520, 76, 0.37)
            label(frame, gauge, 520, 101)
            label(frame, "1.40-1.95 mm", 520, 124, 0.37, (135, 237, 160))
            label(frame, phase, 12, 430)
            encoder.stdin.write(frame.tobytes())
        encoder.stdin.close()
        if encoder.wait():
            raise RuntimeError("Failed to encode video")


if __name__ == "__main__":
    main()
