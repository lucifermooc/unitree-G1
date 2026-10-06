#!/usr/bin/env python3
"""Sample frames from a ROS 2 bag's color topic into PPM files (the bag is only read).

Runs under the system ROS 2 Python in its OWN shell -- never the server's:

    bash -c 'source /opt/ros/jazzy/setup.bash && /usr/bin/python3.12 extract_bag_frames.py \\
        /opt/G1/bags/d435_ghost_0922 /home/unitree/lightnav/bag_frames/d435_ghost_0922 \\
        --hz 4 --count 60 --start 5'

Then: python bench_ws.py --frames <out dir> --instruction "..."
"""

import argparse
import os

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("out")
    ap.add_argument("--topic", default="/camera/camera/color/image_raw")
    ap.add_argument("--hz", type=float, default=4.0)
    ap.add_argument("--count", type=int, default=60)
    ap.add_argument("--start", type=float, default=0.0, help="seconds after the first message")
    ap.add_argument("--storage", default="mcap")
    args = ap.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id=args.storage),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.topic]))
    os.makedirs(args.out, exist_ok=True)

    period = int(1e9 / args.hz)
    next_t = None
    written = 0
    while reader.has_next() and written < args.count:
        _, data, t = reader.read_next()
        if next_t is None:
            next_t = t + int(args.start * 1e9)
        if t < next_t:
            continue
        msg = deserialize_message(data, Image)
        img = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
        img = img[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding.lower() == "bgr8":
            img = img[:, :, ::-1]
        elif msg.encoding.lower() != "rgb8":
            raise SystemExit(f"unsupported encoding {msg.encoding}")
        path = os.path.join(args.out, f"frame_{written:04d}.ppm")
        with open(path, "wb") as f:
            f.write(f"P6\n{msg.width} {msg.height}\n255\n".encode())
            f.write(np.ascontiguousarray(img).tobytes())
        written += 1
        next_t += period
    print(f"wrote {written} frames ({args.hz} Hz, encoding {msg.encoding if written else '?'}) to {args.out}")
    return 0 if written else 1


if __name__ == "__main__":
    raise SystemExit(main())
