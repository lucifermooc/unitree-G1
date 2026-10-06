#!/usr/bin/env python3
"""Replay frames against a running lightnav-serve and summarise the replies.

Same request loop as ``python -m lightnav.cli.ws_client`` (login, reset, one ``next``
per frame, one request outstanding), but it keeps every reply field a bring-up check
needs: ``latency_ms`` and ``timings_ms`` (server side), the client round trip,
``visible`` / ``stop`` and ``pointing.opos_*`` (can the model see the target?).

    # first-person viewer of the repo's MuJoCo demo gif, sampled at 4 Hz
    python bench_ws.py --gif docs/assets/mujoco_demo.gif --crop 20,112,730,510 --fps 4 \\
        --instruction "move forward, then go to the trashcan on the right"
    python bench_ws.py --frames DIR --instruction "..." --jsonl steps.jsonl

Needs numpy, Pillow and websockets>=12 (all present in the server venv).
"""

from __future__ import annotations

import argparse
import base64
import bisect
import collections
import io
import json
import statistics
import time
import uuid
from pathlib import Path

import numpy as np
from PIL import Image, ImageSequence

KEY_TIMINGS = ("vit_ms", "llm_ms", "build_sample_ms", "queue_wait_ms", "batch_total_ms",
               "vit_cache_misses")


def _crop_box(spec: str | None):
    if not spec:
        return None
    x0, y0, x1, y1 = (int(v) for v in spec.split(","))
    return (x0, y0, x1, y1)


def frames_from_gif(path: str, fps: float | None, crop) -> list[np.ndarray]:
    im = Image.open(path)
    frames, starts, t = [], [], 0.0
    for fr in ImageSequence.Iterator(im):
        starts.append(t)
        t += (fr.info.get("duration") or 100) / 1000.0
        frames.append(fr.convert("RGB"))
    if fps:  # resample to the checkpoint's video_fps by presentation time
        n = max(1, int(round(t * fps)))
        frames = [frames[bisect.bisect_right(starts, k / fps) - 1] for k in range(n)]
    return [np.asarray(f.crop(crop) if crop else f, dtype=np.uint8) for f in frames]


def frames_from_dir(path: str, crop) -> list[np.ndarray]:
    exts = {".jpg", ".jpeg", ".png", ".bmp", ".ppm"}
    files = sorted(p for p in Path(path).iterdir() if p.suffix.lower() in exts)
    out = []
    for f in files:
        img = Image.open(f).convert("RGB")
        out.append(np.asarray(img.crop(crop) if crop else img, dtype=np.uint8))
    return out


def frames_from_video(path: str, fps: float | None, crop) -> list[np.ndarray]:
    import cv2

    cap = cv2.VideoCapture(path)
    src_fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
    stride = max(1, round(src_fps / fps)) if (fps and src_fps > 0) else 1
    out, i = [], 0
    while True:
        ok, bgr = cap.read()
        if not ok:
            break
        if i % stride == 0:
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            if crop:
                x0, y0, x1, y1 = crop
                rgb = rgb[y0:y1, x0:x1]
            out.append(np.ascontiguousarray(rgb))
        i += 1
    cap.release()
    return out


def jpeg_b64(rgb: np.ndarray, quality: int) -> str:
    buf = io.BytesIO()
    Image.fromarray(rgb).save(buf, format="JPEG", quality=quality)
    return base64.b64encode(buf.getvalue()).decode()


def pct(xs: list[float], q: float) -> float:
    xs = sorted(xs)
    k = (len(xs) - 1) * q
    lo = int(k)
    hi = min(lo + 1, len(xs) - 1)
    return xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def main() -> int:
    from websockets.sync.client import connect

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--server", default="ws://127.0.0.1:8050")
    ap.add_argument("--instruction", required=True)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--gif")
    src.add_argument("--frames")
    src.add_argument("--video")
    ap.add_argument("--fps", type=float, default=4.0, help="resample gif/video to this rate (0 = every frame)")
    ap.add_argument("--crop", help="x0,y0,x1,y1 crop applied to every frame")
    ap.add_argument("--hz", type=float, default=0.0, help="pace requests at this rate (0 = back to back)")
    ap.add_argument("--episodes", type=int, default=1, help="replay the frame list N times, reset in between")
    ap.add_argument("--skip", type=int, default=2, help="predicted steps of episode 0 left out of the stats")
    ap.add_argument("--quality", type=int, default=90, help="JPEG quality")
    ap.add_argument("--jsonl", help="write one JSON record per step here")
    ap.add_argument("--quiet", action="store_true", help="summary only")
    args = ap.parse_args()

    crop = _crop_box(args.crop)
    fps = args.fps or None
    if args.gif:
        frames = frames_from_gif(args.gif, fps, crop)
    elif args.video:
        frames = frames_from_video(args.video, fps, crop)
    else:
        frames = frames_from_dir(args.frames, crop)
    if not frames:
        raise SystemExit("no frames")
    h, w = frames[0].shape[:2]
    print(f"[bench] {len(frames)} frames {w}x{h} x {args.episodes} episode(s) -> {args.server}")
    print(f"[bench] instruction: {args.instruction!r}")

    records = []
    with connect(args.server, max_size=64 * 1024 * 1024, open_timeout=30) as ws:
        def call(action: str, data: dict) -> dict:
            ws.send(json.dumps({"action": action, "data": data}))
            return json.loads(ws.recv()).get("data", {})

        assert call("login", {"clientId": f"bench_{uuid.uuid4().hex[:6]}"}).get("rc") == 0, "login failed"
        for ep in range(args.episodes):
            assert call("reset", {}).get("rc") == 0, "reset failed"
            for seq, frame in enumerate(frames):
                t0 = time.monotonic()
                data = call("next", {"seq": seq, "image": jpeg_b64(frame, args.quality),
                                     "instruction": args.instruction})
                rtt_ms = (time.monotonic() - t0) * 1000.0
                pointing = data.get("pointing") or {}
                actions = (data.get("actions") or {}).get("actions") or []
                rec = {
                    "episode": ep, "seq": seq, "rc": data.get("rc"), "msg": data.get("msg"),
                    "step": (data.get("actions") or {}).get("step"),
                    "latency_ms": data.get("latency_ms"), "rtt_ms": round(rtt_ms, 2),
                    "timings_ms": data.get("timings_ms") or {},
                    "stop": data.get("stop"), "visible": data.get("visible"),
                    "opos_state": pointing.get("opos_state"), "opos_px": pointing.get("opos_px"),
                    "apos_state": pointing.get("apos_state"), "apos_px": pointing.get("apos_px"),
                    "frame_size": pointing.get("frame_size"),
                    "wp0": [round(float(v), 3) for v in actions[0]] if actions else None,
                    "raw_text": data.get("raw_text"),
                }
                records.append(rec)
                if not args.quiet:
                    if rec["rc"] != 0 or rec["latency_ms"] is None:
                        print(f"[ep{ep} seq {seq:>3}] rc={rec['rc']} {rec['msg']}")
                    else:
                        tm = rec["timings_ms"]
                        tstr = " ".join(f"{k.replace('_ms', '')}={tm[k]:.0f}" for k in KEY_TIMINGS if k in tm)
                        print(f"[ep{ep} seq {seq:>3}] {rec['latency_ms']:6.1f} ms (rtt {rtt_ms:6.1f})  {tstr}  "
                              f"vis={rec['visible']} stop={rec['stop']} opos={rec['opos_state']}@{rec['opos_px']} "
                              f"apos={rec['apos_state']}@{rec['apos_px']} wp0={rec['wp0']} raw={rec['raw_text']}")
                if args.hz:
                    time.sleep(max(0.0, 1.0 / args.hz - (time.monotonic() - t0)))

    if args.jsonl:
        with open(args.jsonl, "w") as f:
            for r in records:
                f.write(json.dumps(r) + "\n")

    ok = [r for r in records if r["rc"] == 0 and r["latency_ms"] is not None]
    errors = [r for r in records if r["rc"] != 0]
    stats = ok[args.skip:] if len(ok) > args.skip else ok
    print(f"\n[bench] summary: {len(ok)} predictions, {len(errors)} errors, stats over {len(stats)} "
          f"(first {min(args.skip, len(ok))} skipped)")
    if stats:
        lat = [r["latency_ms"] for r in stats]
        rtt = [r["rtt_ms"] for r in stats]
        print(f"  latency_ms  median {statistics.median(lat):.1f}  p90 {pct(lat, 0.9):.1f}  "
              f"mean {statistics.fmean(lat):.1f}  min {min(lat):.1f}  max {max(lat):.1f}  "
              f"-> {1000.0 / statistics.median(lat):.2f} Hz")
        print(f"  rtt_ms      median {statistics.median(rtt):.1f}  p90 {pct(rtt, 0.9):.1f}")
        keys = sorted({k for r in stats for k, v in r["timings_ms"].items() if isinstance(v, (int, float))})
        print("  timings_ms median: " + "  ".join(
            f"{k}={statistics.median([r['timings_ms'][k] for r in stats if k in r['timings_ms']]):.1f}"
            for k in keys))
    print(f"  visible: {dict(collections.Counter(str(r['visible']) for r in ok))}  "
          f"stop: {sum(1 for r in ok if r['stop'])}/{len(ok)}")
    print(f"  opos_state: {dict(collections.Counter(str(r['opos_state']) for r in ok))}  "
          f"apos_state: {dict(collections.Counter(str(r['apos_state']) for r in ok))}")
    return 0 if ok and not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
