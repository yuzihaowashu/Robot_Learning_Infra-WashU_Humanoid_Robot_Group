#!/usr/bin/env python3
"""
Lightweight camera streaming server for Unitree G1.
Runs on the robot, captures from /dev/video* and sends JPEG frames over ZMQ.

Only requires: opencv-python, pyzmq (both pre-installed on the G1 Jetson).

Usage (on robot):
    python3 robot_camera_server.py --device 4 --port 5555
"""

import argparse
import base64
import json
import signal
import sys
import time

import cv2
import zmq


def find_working_cameras(max_check=6):
    """Probe /dev/video0..N and return list of working device indices."""
    working = []
    for i in range(max_check):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                working.append(i)
            cap.release()
    return working


def main():
    parser = argparse.ArgumentParser(description="G1 Camera ZMQ Server")
    parser.add_argument("--device", type=int, default=-1,
                        help="Camera device index (default: auto-detect)")
    parser.add_argument("--port", type=int, default=5555,
                        help="ZMQ PUB port (default: 5555)")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--quality", type=int, default=70,
                        help="JPEG quality 1-100 (default: 70)")
    args = parser.parse_args()

    # Auto-detect camera if not specified
    if args.device < 0:
        print("Auto-detecting cameras...")
        working = find_working_cameras()
        if not working:
            print("ERROR: No working cameras found!")
            sys.exit(1)
        print(f"  Found working cameras: {working}")
        args.device = working[-1]  # usually the highest index is the head camera
        print(f"  Using device {args.device}")

    cap = cv2.VideoCapture(args.device)
    if not cap.isOpened():
        print(f"ERROR: Cannot open /dev/video{args.device}")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera: /dev/video{args.device} "
          f"({actual_w}x{actual_h})")

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 5)
    sock.setsockopt(zmq.LINGER, 0)
    sock.bind(f"tcp://0.0.0.0:{args.port}")
    print(f"ZMQ PUB on tcp://0.0.0.0:{args.port}")
    print("Streaming... (Ctrl+C to stop)")

    running = True

    def handler(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)

    frame_count = 0
    t_start = time.time()
    interval = 1.0 / args.fps

    while running:
        t0 = time.time()
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.01)
            continue

        _, buf = cv2.imencode(
            ".jpg", frame,
            [cv2.IMWRITE_JPEG_QUALITY, args.quality]
        )
        b64 = base64.b64encode(buf).decode("ascii")

        msg = json.dumps({
            "images": {"head_camera": b64},
            "timestamps": {"head_camera": time.time()},
            "shape": [frame.shape[0], frame.shape[1]],
        })

        try:
            sock.send_string(msg, zmq.NOBLOCK)
        except zmq.Again:
            pass

        frame_count += 1
        if frame_count % 60 == 0:
            elapsed = time.time() - t_start
            print(f"  {frame_count} frames, {frame_count/elapsed:.1f} fps")

        sleep_time = interval - (time.time() - t0)
        if sleep_time > 0:
            time.sleep(sleep_time)

    print("\nStopping...")
    cap.release()
    sock.close()
    ctx.term()


if __name__ == "__main__":
    main()
