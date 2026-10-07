# Blender render pipeline

Headless Cycles (CPU) renders of the left-right moving automaton, driven by `tools/kinematics.py`
(the same contract as `spec/kinematics.json`).

## Files

| file | purpose |
|---|---|
| `render/render_scene.py` | reusable module + CLI: clean scene, import 5 GLBs, pose, materials, lights, floor, camera, render |
| `render/render_all.py` | batch driver: final stills, 24-frame motion sequence, mp4/webm, contact sheet, `manifest.json` |
| `web/renders/` | outputs (`hero.jpg`, `front.jpg`, `iso_off.jpg`, `detail_*.jpg`, `motion/f00..f23.jpg`, `motion.mp4/.webm`, `motion_sheet.jpg`, `manifest.json`) |

## Running

bpy 5.2.2 lives in a venv; run everything with that interpreter and **not** from inside a directory that contains
files named like stdlib modules (a stray `inspect.py` next to a script makes `import bpy` die with a glog error).

```
PY=/tmp/claude-0/venv/bin/python
$PY render/render_scene.py --view hero --theta-deg 35 --on --samples 64 --res 1600x900 --out web/renders/hero.jpg
$PY render/render_scene.py --view front --theta-deg 120 --off --samples 32 --res 960x540 --out /tmp/x.jpg
```

Views: `hero front iso top side detail_drive detail_figure motion` (see `VIEWS` in `render_scene.py`; each is
azimuth/elevation/focal length plus a box in mm that is auto-fitted into frame with a margin).
Extra flags: `--save-blend path.blend`.

Batch: `render_all.py stills [name.jpg ...]`, `render_all.py motion`, `render_all.py post`
(post = rod-attachment check, contact sheet via Pillow, ffmpeg mp4/webm, manifest).

From other Python code:

```python
import sys; sys.path.insert(0, "render")
import render_scene as rs, math
rs.build_scene(theta_deg=35, switch_on=True, view="hero", samples=48, res=(960, 540))
for th in range(0, 360, 15):
    rs.set_pose(math.radians(th), True)       # re-pose without rebuilding
    rs.render(f"/tmp/f{th:03d}.jpg")
print(rs.rod_gap_check(30))                    # (gaps_mm_by_theta, far_vertex)
```

## How the pose is applied

The glTF importer already puts each object origin at the GLB node translation (converted to Z-up metres), with
quaternion rotation mode and identity rotation.  `set_pose()` sets `rotation_mode='XYZ'`,
`location = loc/1000` and `rotation_euler.y = -rot` for every node returned by `kinematics.pose()`;
everything not listed there stays static.  The LED lens material and a tiny red point light (the "glow") follow
`switch_on`.

Verification (`rod_gap_check`): the far end of `conn_rod` (baked pose0 eye centre, 92.5/0/38 mm in its local frame, 100 mm from
its origin; confirmed to be the centre of the eye's bbox) is transformed by the rod's world matrix and compared with
the carriage origin: max gap 0.0014 mm (float rounding) for theta = 0..330 step 30, and also for step 5.

## Look

* Opaque materials: glTF Principled BSDF kept, alpha forced to 1, a shader-only **Bevel node** (0.4 mm, 4 samples)
  perturbs the normal so edges catch light without touching geometry.
* `acrylic_clear` / `acrylic_smoke`: custom shader = Mix(Transparent*tint, Glossy(rough 0.02/0.04)) with a Schlick
  Fresnel (F0 0.039, IOR 1.49, boosted x1.3, reduced to x0.55 in the `front` view).  Reason: real Principled
  *transmission* blocks shadow rays so the interior would be unlit and noisy; Transparent BSDF lets the key light
  into the box, and it is far cheaper.  The Fresnel *node* is deliberately not used: with the glTF normals/back faces it
  returns total internal reflection beyond ~42 deg and blacks out half the panel.  Smoke tint 0.62/0.66/0.72 (the
  original dark alpha-0.4 smoke hid the 2nd/3rd gear stage).
* LED: ON = emission (1.0, 0.08, 0.04) x 8 + small point light; OFF = dark red, no emission.
* Studio: gradient world (brighter for camera rays than for lighting), 4 area lights (warm key front-right,
  cool fill front-left, two rim lights behind), 40 m matte floor at z = -3.1 mm fading into the backdrop colour.
  AgX "High Contrast", exposure -1.
* Cycles CPU, adaptive sampling (threshold 0.02), OpenImageDenoise (albedo+normal), bounces 12 total /
  transmission 8 / transparent 24, indirect clamp 8, caustics off.

## Timings (4 CPU cores, per frame, wall-clock incl. denoise)

| image | res | samples | seconds |
|---|---|---|---|
| hero | 1600x900 | 64 | ~125 |
| front | 1600x900 | 64 | ~160 |
| iso_off | 1280x720 | 48 | ~55 |
| detail_drive | 1280x720 | 64 | ~200 (close-up of many semi-transparent plates) |
| detail_figure | 1280x720 | 64 | ~91 |
| motion f00..f23 | 960x540 | 32 | ~30 each (~12 min total) |

Exact numbers are in `web/renders/manifest.json`.

## Known limitations

* Glass is a thin-surface approximation: no refraction/offset through the 4 mm panel and no caustics.  Its presence
  reads through soft Fresnel reflections of the lit table and the edge screws/frames, deliberately subtle so the
  mechanism stays readable.
* Antenna does not swing (rot 0, follows dx only) as in the kinematics contract.
* Render times are dominated by transparent layers (gear plates); `detail_drive` is the slowest.
* Wood grain/normal detail is whatever the GLB materials provide (flat colours).
* `read_factory_settings(use_empty=True)` is called by `build_scene`, so call it once per process (or accept a reset).
