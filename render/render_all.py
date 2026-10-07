#!/usr/bin/env python
"""Render all final images / motion sequence / videos / contact sheet / manifest into web/renders/.

    /tmp/claude-0/venv/bin/python render/render_all.py stills [names]  # hero, front, iso_off, detail_drive, detail_figure
    /tmp/claude-0/venv/bin/python render/render_all.py motion     # 24 frames f00..f23
    /tmp/claude-0/venv/bin/python render/render_all.py post       # mp4/webm (ffmpeg), motion_sheet.jpg (PIL), manifest.json
Each stage updates web/renders/manifest.json incrementally (timings are measured wall-clock per frame).
"""
import json
import math
import os
import shutil
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import render_scene as rs  # noqa: E402
import bpy  # noqa: E402

OUT = os.path.join(rs.ROOT, "web", "renders")
MANIFEST = os.path.join(OUT, "manifest.json")

STILLS = [
    # name, view, theta_deg, on, samples, res
    ("hero.jpg", "hero", 35, True, 64, (1600, 900)),
    ("front.jpg", "front", 35, True, 64, (1600, 900)),
    ("iso_off.jpg", "iso", 35, False, 48, (1280, 720)),
    ("detail_drive.jpg", "detail_drive", 35, True, 64, (1280, 720)),
    ("detail_figure.jpg", "detail_figure", 35, True, 64, (1280, 720)),
]
MOTION_SAMPLES = 32
MOTION_RES = (960, 540)
THETAS = list(range(0, 360, 15))


def load_manifest():
    if os.path.exists(MANIFEST):
        with open(MANIFEST, encoding="utf-8") as f:
            return json.load(f)
    return {"files": {}}


def save_manifest(m):
    with open(MANIFEST, "w", encoding="utf-8") as f:
        json.dump(m, f, indent=2, ensure_ascii=False)


def pose_info(theta_deg, on):
    p = rs.kinematics.pose(math.radians(theta_deg), on)
    return {"theta_deg": theta_deg, "switch_on": on,
            "carriage_x_mm": round(p["carriage"]["loc"][0], 3),
            "conn_rod_rot_deg": round(math.degrees(p["conn_rod"]["rot"]), 3),
            "gear_G3_rot_deg": round(math.degrees(p["gear_G3"]["rot"]), 3),
            "gear_P1_rot_deg": round(math.degrees(p["gear_P1"]["rot"]), 3),
            "figure_dx_mm": round(p["figure_body"]["loc"][0] - 26.68, 3),
            "switch_knob_x_mm": round(p["switch_knob"]["loc"][0], 3)}


def stills(only=None):
    m = load_manifest()
    for name, view, th, on, spl, res in STILLS:
        if only and name not in only:
            continue
        rs.build_scene(th, on, view, spl, res)
        t = rs.render(os.path.join(OUT, name), spl, res)
        m["files"][name] = {"view": view, "resolution": list(res), "samples": spl, "seconds": round(t, 1),
                            "pose": pose_info(th, on)}
        save_manifest(m)
        print(f"{name}: {t:.1f}s", flush=True)


def motion():
    m = load_manifest()
    rs.build_scene(0, True, "motion", MOTION_SAMPLES, MOTION_RES)
    os.makedirs(os.path.join(OUT, "motion"), exist_ok=True)
    frames = []
    for i, th in enumerate(THETAS):
        rs.set_pose(math.radians(th), True)
        name = f"motion/f{i:02d}.jpg"
        t = rs.render(os.path.join(OUT, name), MOTION_SAMPLES, MOTION_RES)
        info = {"view": "motion", "resolution": list(MOTION_RES), "samples": MOTION_SAMPLES,
                "seconds": round(t, 1), "pose": pose_info(th, True)}
        m["files"][name] = info
        frames.append(info)
        save_manifest(m)
        print(f"{name}: theta={th} {t:.1f}s", flush=True)
    xs = [f["pose"]["carriage_x_mm"] for f in frames]
    m["motion"] = {"frames": len(frames), "theta_step_deg": 15, "fps": 12,
                   "carriage_x_range_mm_observed": [min(xs), max(xs)],
                   "mean_seconds_per_frame": round(sum(f["seconds"] for f in frames) / len(frames), 1)}
    save_manifest(m)


def post():
    m = load_manifest()
    lo, hi = rs.kinematics.stroke()
    m["kinematics"] = {"carriage_x_full_stroke_mm": [round(lo, 3), round(hi, 3)],
                       "carriage_x_pose0_mm": round(rs.kinematics.X0, 3)}
    gaps_rs = None
    try:
        rs.build_scene(0, True, "hero", 4, (160, 90))
        gaps, _ = rs.rod_gap_check(30)
        m["rod_attachment_gap_mm"] = {"theta_step_deg": 30, "per_theta": {str(k): round(v, 5) for k, v in gaps.items()},
                                      "max": round(max(gaps.values()), 5)}
    except Exception as e:  # pragma: no cover
        print("gap check failed", e)
    # contact sheet
    try:
        from PIL import Image
        cols, rows, tw, th = 6, 4, 320, 180
        sheet = Image.new("RGB", (cols * tw, rows * th), (255, 255, 255))
        for i in range(len(THETAS)):
            im = Image.open(os.path.join(OUT, f"motion/f{i:02d}.jpg")).convert("RGB").resize((tw, th), Image.LANCZOS)
            sheet.paste(im, ((i % cols) * tw, (i // cols) * th))
        sheet.save(os.path.join(OUT, "motion_sheet.jpg"), quality=88)
        m["files"]["motion_sheet.jpg"] = {"resolution": [cols * tw, rows * th], "grid": "6x4", "frames": "f00..f23"}
    except Exception as e:
        print("contact sheet failed", e)
    # video: forward then drop last duplicate -> loop-friendly (theta 345 -> 0 is one 15 deg step)
    if shutil.which("ffmpeg"):
        src = os.path.join(OUT, "motion", "f%02d.jpg")
        mp4 = os.path.join(OUT, "motion.mp4")
        webm = os.path.join(OUT, "motion.webm")
        common = ["ffmpeg", "-y", "-loglevel", "error", "-framerate", "12", "-stream_loop", "2", "-i", src]
        # loop 3x inside the file so players that do not loop still show a few cycles
        subprocess.run(common + ["-vf", "scale=960:540,format=yuv420p", "-c:v", "libx264", "-crf", "20",
                                 "-preset", "slow", "-movflags", "+faststart", "-r", "12", mp4], check=True)
        subprocess.run(common + ["-vf", "scale=960:540,format=yuv420p", "-c:v", "libvpx-vp9", "-crf", "30", "-b:v", "0",
                                 "-r", "12", webm], check=True)
        for f in (mp4, webm):
            m["files"][os.path.basename(f)] = {"resolution": [960, 540], "fps": 12, "frames": 72,
                                               "note": "24 motion frames looped 3x (one frame = 15 deg of crank)",
                                               "bytes": os.path.getsize(f)}
    save_manifest(m)


if __name__ == "__main__":
    stage = sys.argv[1] if len(sys.argv) > 1 else "stills"
    if stage == "stills":
        stills(sys.argv[2:] or None)
    else:
        {"motion": motion, "post": post}[stage]()
