#!/usr/bin/env python
"""Blender (bpy) render pipeline for the left-right moving automaton.

Usage (CLI):
    /tmp/claude-0/venv/bin/python render/render_scene.py --view hero --theta-deg 35 --on \
        --samples 64 --res 1600x900 --out web/renders/hero.jpg

Usage (module):
    import render_scene as rs
    rs.build_scene(theta_deg=35, switch_on=True)      # clean scene + 5 GLBs + materials + lights + floor
    rs.set_pose(math.radians(60), True)               # re-pose without rebuilding
    rs.setup_camera("hero")                           # (re)place camera
    rs.render("/path/out.jpg", samples=48, res=(960, 540))

Coordinate conventions: assembly frame is Z-up, +x right, -y toward the viewer (front).  The GLB importer
converts glTF Y-up -> Blender Z-up and object origins already equal the GLB node translations (metres).
pose() from tools/kinematics.py gives absolute node origin locations in mm and a front-CCW rotation, which
is applied as location = loc/1000 and rotation_euler.y = -rot.
"""
import argparse
import math
import os
import sys
import time

import bpy
from mathutils import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "tools"))
import kinematics  # noqa: E402

PARTS_DIR = os.path.join(ROOT, "assets", "parts")
PARTS = ["housing", "drive", "slide", "electronics", "figure"]
MM = 0.001

BBOX_MIN = (-134.0, -47.0, -3.0)
BBOX_MAX = (134.0, 42.0, 172.75)
FLOOR_Z_MM = -3.1
LIGHT_SCALE = 0.11
WORLD_STRENGTH = 0.75
EXPOSURE = -1.0

# ---------------------------------------------------------------------------------------------- views
# azim: degrees from the front (-Y) axis, positive toward +X.  elev: degrees above the horizon.
# fit: (min, max) box in mm that must fit into the frame (None -> whole model).  shift: extra target offset (mm).
VIEWS = {
    "hero":          dict(azim=32, elev=21, focal=50, margin=1.10, fit=None, shift=(0, 0, 0)),
    "front":         dict(azim=0, elev=1.5, focal=110, margin=1.06, fit=None, shift=(0, 0, 0)),
    "iso":           dict(azim=-38, elev=28, focal=42, margin=1.10, fit=None, shift=(0, 0, 0)),
    "top":           dict(azim=0, elev=78, focal=70, margin=1.10, fit=None, shift=(0, 0, 0)),
    "side":          dict(azim=88, elev=6, focal=80, margin=1.10, fit=None, shift=(0, 0, 0)),
    "motion":        dict(azim=16, elev=13, focal=65, margin=1.07, fit=None, shift=(0, 0, 0)),
    "detail_drive":  dict(azim=14, elev=8, focal=100, margin=1.12,
                          fit=((-108, -47, 14), (-3, -43, 72)), shift=(0, 0, 0)),
    "detail_figure": dict(azim=26, elev=14, focal=85, margin=1.10,
                          fit=((-42, -50, 100), (96, -22, 176)), shift=(0, 0, 0)),
}


# ---------------------------------------------------------------------------------------------- scene build
def _clean_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    return bpy.context.scene


def _import_parts():
    objs = {}
    for part in PARTS:
        before = set(bpy.data.objects.keys())
        bpy.ops.import_scene.gltf(filepath=os.path.join(PARTS_DIR, part + ".glb"))
        for n in set(bpy.data.objects.keys()) - before:
            o = bpy.data.objects[n]
            o.rotation_mode = "XYZ"
            objs[o.name] = o
            o["part"] = part
    return objs


def _principled(mat):
    for n in mat.node_tree.nodes:
        if n.type == "BSDF_PRINCIPLED":
            return n
    return None


def _base(mat):
    return mat.name.split(".")[0]


def _build_acrylic(mat, tint, tint_strength_color, rough, reflect_boost=1.0):
    """Cheap-but-convincing acrylic: Mix(Transparent*tint, Glossy(IOR 1.49)) by Fresnel.
    Transparent BSDF lets shadow rays through, so the interior is lit by the key light (real
    transmission would block it)."""
    nt = mat.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    transp = nt.nodes.new("ShaderNodeBsdfTransparent")
    transp.inputs["Color"].default_value = tint
    glossy = nt.nodes.new("ShaderNodeBsdfGlossy")
    glossy.inputs["Roughness"].default_value = rough
    glossy.inputs["Color"].default_value = tint_strength_color
    # Schlick Fresnel (F0 = 0.039 for IOR 1.49) on |N.I| -- robust for flipped glTF normals / back faces,
    # unlike the Fresnel node which hits total internal reflection on back faces.
    geo = nt.nodes.new("ShaderNodeNewGeometry")
    dot = nt.nodes.new("ShaderNodeVectorMath")
    dot.operation = "DOT_PRODUCT"
    nt.links.new(geo.outputs["Normal"], dot.inputs[0])
    nt.links.new(geo.outputs["Incoming"], dot.inputs[1])
    ab = nt.nodes.new("ShaderNodeMath")
    ab.operation = "ABSOLUTE"
    nt.links.new(dot.outputs["Value"], ab.inputs[0])
    om = nt.nodes.new("ShaderNodeMath")
    om.operation = "SUBTRACT"
    om.inputs[0].default_value = 1.0
    nt.links.new(ab.outputs["Value"], om.inputs[1])
    pw = nt.nodes.new("ShaderNodeMath")
    pw.operation = "POWER"
    pw.inputs[1].default_value = 5.0
    nt.links.new(om.outputs["Value"], pw.inputs[0])
    sch = nt.nodes.new("ShaderNodeMath")
    sch.operation = "MULTIPLY_ADD"
    sch.inputs[1].default_value = 0.961
    sch.inputs[2].default_value = 0.039
    nt.links.new(pw.outputs["Value"], sch.inputs[0])
    # boost a bit: a real slab has two reflecting surfaces and the panel should read as glass head-on
    mul = nt.nodes.new("ShaderNodeMath")
    mul.operation = "MULTIPLY"
    mul.inputs[1].default_value = 1.0 * reflect_boost
    mul.use_clamp = True
    nt.links.new(sch.outputs["Value"], mul.inputs[0])
    mix = nt.nodes.new("ShaderNodeMixShader")
    nt.links.new(mul.outputs["Value"], mix.inputs["Fac"])
    nt.links.new(transp.outputs["BSDF"], mix.inputs[1])
    nt.links.new(glossy.outputs["BSDF"], mix.inputs[2])
    nt.links.new(mix.outputs["Shader"], out.inputs["Surface"])
    mat.blend_method = "OPAQUE" if hasattr(mat, "blend_method") else None


def _add_bevel(mat, radius_m=0.0004):
    """Shader-only bevel (Bevel node on the Principled normal) -> soft edge highlights without geometry change."""
    nt = mat.node_tree
    p = _principled(mat)
    if p is None or p.inputs["Normal"].is_linked:
        return
    bev = nt.nodes.new("ShaderNodeBevel")
    bev.inputs["Radius"].default_value = radius_m
    bev.samples = 4
    nt.links.new(bev.outputs["Normal"], p.inputs["Normal"])


def _fixup_materials():
    for mat in list(bpy.data.materials):
        if not mat.use_nodes:
            continue
        b = _base(mat)
        if b == "acrylic_clear":
            _build_acrylic(mat, tint=(0.90, 0.97, 1.0, 1.0), tint_strength_color=(1.0, 1.0, 1.0, 1.0),
                           rough=0.02)
        elif b == "acrylic_smoke":
            _build_acrylic(mat, tint=(0.30, 0.33, 0.38, 1.0), tint_strength_color=(0.9, 0.95, 1.0, 1.0),
                           rough=0.04, reflect_boost=0.9)
        else:
            p = _principled(mat)
            if p is None:
                continue
            p.inputs["Alpha"].default_value = 1.0
            if hasattr(mat, "blend_method"):
                mat.blend_method = "OPAQUE"
            # slightly calmer specular so painted/wooden parts don't sparkle
            if b.startswith(("wood", "gear", "tin", "plastic", "rubber")):
                pass
            if b == "led_lens":
                continue
            _add_bevel(mat)


def set_led(on):
    for mat in bpy.data.materials:
        if _base(mat) != "led_lens" or not mat.use_nodes:
            continue
        p = _principled(mat)
        if on:
            p.inputs["Emission Color"].default_value = (1.0, 0.08, 0.04, 1.0)
            p.inputs["Emission Strength"].default_value = 8.0
            p.inputs["Base Color"].default_value = (0.6, 0.03, 0.02, 1.0)
        else:
            p.inputs["Emission Color"].default_value = (0.19, 0.01, 0.01, 1.0)
            p.inputs["Emission Strength"].default_value = 0.0
            p.inputs["Base Color"].default_value = (0.19, 0.01, 0.01, 1.0)
    glow = bpy.data.objects.get("led_glow")
    if glow:
        glow.hide_render = not on
        glow.hide_viewport = not on


def _setup_world(scene):
    w = bpy.data.worlds.new("studio")
    scene.world = w
    w.use_nodes = True
    nt = w.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputWorld")
    bg = nt.nodes.new("ShaderNodeBackground")
    coord = nt.nodes.new("ShaderNodeTexCoord")
    sep = nt.nodes.new("ShaderNodeSeparateXYZ")
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    mr = nt.nodes.new("ShaderNodeMapRange")
    mr.inputs["From Min"].default_value = -1.0
    mr.inputs["From Max"].default_value = 1.0
    nt.links.new(coord.outputs["Generated"], sep.inputs["Vector"])
    # Generated coords of the world are in 0..1 range for direction? use object-space direction instead
    nt.links.remove(sep.inputs["Vector"].links[0])
    nt.links.new(coord.outputs["Object"], sep.inputs["Vector"])
    nt.links.new(sep.outputs["Z"], mr.inputs["Value"])
    nt.links.new(mr.outputs["Result"], ramp.inputs["Fac"])
    cr = ramp.color_ramp
    cr.elements[0].position = 0.0
    cr.elements[0].color = (0.50, 0.46, 0.42, 1)    # below horizon: warm grey
    cr.elements[1].position = 1.0
    cr.elements[1].color = (0.78, 0.82, 0.88, 1)    # zenith: cool light grey
    e = cr.elements.new(0.50)
    e.color = (0.90, 0.86, 0.80, 1)                 # horizon: warm light
    e2 = cr.elements.new(0.75)
    e2.color = (0.85, 0.86, 0.88, 1)
    nt.links.new(ramp.outputs["Color"], bg.inputs["Color"])
    bg.inputs["Strength"].default_value = WORLD_STRENGTH
    nt.links.new(bg.outputs["Background"], out.inputs["Surface"])


def _setup_floor():
    z = FLOOR_Z_MM * MM
    bpy.ops.mesh.primitive_plane_add(size=40.0, location=(0, 0, z))
    fl = bpy.context.active_object
    fl.name = "floor"
    mat = bpy.data.materials.new("floor_mat")
    mat.use_nodes = True
    nt = mat.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    diff = nt.nodes.new("ShaderNodeBsdfDiffuse")
    diff.inputs["Color"].default_value = (0.72, 0.69, 0.65, 1)
    diff.inputs["Roughness"].default_value = 0.9
    # slight satin sheen so the box gets a faint soft reflection on the table
    glossy = nt.nodes.new("ShaderNodeBsdfGlossy")
    glossy.inputs["Roughness"].default_value = 0.35
    glossy.inputs["Color"].default_value = (0.9, 0.9, 0.9, 1)
    sheen = nt.nodes.new("ShaderNodeMixShader")
    sheen.inputs["Fac"].default_value = 0.06
    nt.links.new(diff.outputs["BSDF"], sheen.inputs[1])
    nt.links.new(glossy.outputs["BSDF"], sheen.inputs[2])
    # distance fade into the world horizon colour
    emi = nt.nodes.new("ShaderNodeEmission")
    emi.inputs["Color"].default_value = (0.90 * WORLD_STRENGTH, 0.86 * WORLD_STRENGTH, 0.80 * WORLD_STRENGTH, 1)
    emi.inputs["Strength"].default_value = 1.0
    geo = nt.nodes.new("ShaderNodeNewGeometry")
    vl = nt.nodes.new("ShaderNodeVectorMath")
    vl.operation = "LENGTH"
    nt.links.new(geo.outputs["Position"], vl.inputs[0])
    mr = nt.nodes.new("ShaderNodeMapRange")
    mr.inputs["From Min"].default_value = 0.8
    mr.inputs["From Max"].default_value = 6.0
    mr.clamp = True
    nt.links.new(vl.outputs["Value"], mr.inputs["Value"])
    fade = nt.nodes.new("ShaderNodeMixShader")
    nt.links.new(mr.outputs["Result"], fade.inputs["Fac"])
    nt.links.new(sheen.outputs["Shader"], fade.inputs[1])
    nt.links.new(emi.outputs["Emission"], fade.inputs[2])
    nt.links.new(fade.outputs["Shader"], out.inputs["Surface"])
    fl.data.materials.append(mat)
    return fl


def _setup_reflection_card():
    """Glossy-only emissive strip on the table in front of the box.  It is invisible to camera, diffuse light and
    shadows, but the acrylic front panel mirrors it (when seen from above) as a soft bright band -> reads as glass."""
    bpy.ops.mesh.primitive_plane_add(size=1.0, location=(-0.12, -0.30, FLOOR_Z_MM * MM + 0.0005))
    ob = bpy.context.active_object
    ob.name = "reflection_card"
    ob.scale = (0.70, 0.05, 1.0)
    mat = bpy.data.materials.new("reflection_card_mat")
    mat.use_nodes = True
    nt = mat.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    emi = nt.nodes.new("ShaderNodeEmission")
    emi.inputs["Color"].default_value = (1.0, 0.97, 0.92, 1)
    tc = nt.nodes.new("ShaderNodeTexCoord")
    sep = nt.nodes.new("ShaderNodeSeparateXYZ")
    nt.links.new(tc.outputs["UV"], sep.inputs["Vector"])
    # triangular profile across the strip width (v), squared -> soft falloff
    sub = nt.nodes.new("ShaderNodeMath")
    sub.operation = "SUBTRACT"
    sub.inputs[1].default_value = 0.5
    nt.links.new(sep.outputs["Y"], sub.inputs[0])
    ab = nt.nodes.new("ShaderNodeMath")
    ab.operation = "ABSOLUTE"
    nt.links.new(sub.outputs["Value"], ab.inputs[0])
    inv = nt.nodes.new("ShaderNodeMath")
    inv.operation = "MULTIPLY_ADD"
    inv.inputs[1].default_value = -2.0
    inv.inputs[2].default_value = 1.0
    nt.links.new(ab.outputs["Value"], inv.inputs[0])
    sq = nt.nodes.new("ShaderNodeMath")
    sq.operation = "MULTIPLY"
    nt.links.new(inv.outputs["Value"], sq.inputs[0])
    nt.links.new(inv.outputs["Value"], sq.inputs[1])
    st = nt.nodes.new("ShaderNodeMath")
    st.operation = "MULTIPLY"
    st.inputs[1].default_value = 7.0
    nt.links.new(sq.outputs["Value"], st.inputs[0])
    nt.links.new(st.outputs["Value"], emi.inputs["Strength"])
    nt.links.new(emi.outputs["Emission"], out.inputs["Surface"])
    ob.data.materials.append(mat)
    ob.visible_camera = False
    ob.visible_diffuse = False
    ob.visible_transmission = False
    ob.visible_shadow = False
    ob.visible_volume_scatter = False
    return ob


def _area_light(name, loc, target, size, power, color=(1, 1, 1), shape="RECTANGLE", size_y=None):
    ld = bpy.data.lights.new(name, "AREA")
    ld.shape = shape
    ld.size = size
    if shape == "RECTANGLE":
        ld.size_y = size_y or size
    ld.energy = power * LIGHT_SCALE
    ld.color = color
    ob = bpy.data.objects.new(name, ld)
    bpy.context.scene.collection.objects.link(ob)
    ob.location = loc
    d = Vector(target) - Vector(loc)
    ob.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
    return ob


def _setup_lights():
    c = (0.0, -0.002, 0.075)  # box centre (m)
    # large soft key: front-right-above
    _area_light("key", (0.55, -0.80, 0.70), c, 0.60, 520, color=(1.0, 0.96, 0.90), size_y=0.45)
    # cool fill from left-front, lower power
    _area_light("fill", (-0.80, -0.60, 0.30), c, 0.60, 160, color=(0.88, 0.93, 1.0), size_y=0.45)
    # rim from behind-left, high
    _area_light("rim", (-0.35, 0.65, 0.55), c, 0.45, 300, color=(1.0, 0.98, 0.95), size_y=0.30)
    # top-back soft rim on the right to define lid edge
    _area_light("rim2", (0.60, 0.50, 0.40), c, 0.35, 160, color=(1.0, 0.97, 0.92), size_y=0.25)
    # LED glow (only shown when switch on)
    pl = bpy.data.lights.new("led_glow", "POINT")
    pl.energy = 0.05
    pl.shadow_soft_size = 0.004
    pl.color = (1.0, 0.1, 0.04)
    ob = bpy.data.objects.new("led_glow", pl)
    bpy.context.scene.collection.objects.link(ob)
    ob.location = (0.100, 0.012, 0.126)


def _setup_render(scene, samples=64, res=(1600, 900)):
    r = scene.render
    r.engine = "CYCLES"
    r.resolution_x, r.resolution_y = res
    r.resolution_percentage = 100
    r.threads_mode = "AUTO"
    cy = scene.cycles
    cy.device = "CPU"
    cy.samples = samples
    cy.use_adaptive_sampling = True
    cy.adaptive_threshold = 0.02
    cy.adaptive_min_samples = 8
    cy.use_denoising = True
    try:
        cy.denoiser = "OPENIMAGEDENOISE"
    except Exception:
        pass
    try:
        cy.denoising_input_passes = "RGB_ALBEDO_NORMAL"
        cy.denoising_prefilter = "ACCURATE"
    except Exception:
        pass
    cy.max_bounces = 12
    cy.diffuse_bounces = 4
    cy.glossy_bounces = 6
    cy.transmission_bounces = 8
    cy.volume_bounces = 0
    cy.transparent_max_bounces = 24
    cy.caustics_reflective = False
    cy.caustics_refractive = False
    cy.sample_clamp_indirect = 8.0
    cy.sample_clamp_direct = 0.0
    cy.film_exposure = 1.0
    cy.use_light_tree = True
    scene.view_settings.view_transform = "AgX"
    for look in ("AgX - High Contrast", "AgX - Medium High Contrast", "None"):
        try:
            scene.view_settings.look = look
            break
        except TypeError:
            continue
    scene.view_settings.exposure = EXPOSURE
    im = r.image_settings
    im.file_format = "JPEG"
    im.quality = 90
    im.color_mode = "RGB"
    r.use_persistent_data = False


def build_scene(theta_deg=0.0, switch_on=True, view="hero", samples=64, res=(1600, 900)):
    scene = _clean_scene()
    objs = _import_parts()
    _fixup_materials()
    _setup_world(scene)
    _setup_floor()
    _setup_reflection_card()
    _setup_lights()
    cam_data = bpy.data.cameras.new("cam")
    cam = bpy.data.objects.new("cam", cam_data)
    scene.collection.objects.link(cam)
    scene.camera = cam
    _setup_render(scene, samples, res)
    set_pose(math.radians(theta_deg), switch_on)
    setup_camera(view)
    return objs


# ---------------------------------------------------------------------------------------------- pose
def set_pose(theta_rad, switch_on):
    """Apply kinematics.pose() to the Blender objects. Returns the pose dict."""
    p = kinematics.pose(theta_rad, switch_on)
    for name, d in p.items():
        o = bpy.data.objects.get(name)
        if o is None:
            raise RuntimeError("node not found in scene: " + name)
        o.rotation_mode = "XYZ"
        o.location = (d["loc"][0] * MM, d["loc"][1] * MM, d["loc"][2] * MM)
        o.rotation_euler = (0.0, -d["rot"], 0.0)
    set_led(switch_on)
    bpy.context.view_layer.update()
    return p


# ---------------------------------------------------------------------------------------------- camera
def _fit_camera(direction, target, focal, aspect, pts, margin, sensor=36.0):
    """Return (cam_location, target) so all pts fit in frame with `margin`, centred."""
    direction = Vector(direction).normalized()
    target = Vector(target)
    for _ in range(6):
        q = (-direction).to_track_quat("-Z", "Y")
        inv = q.to_matrix().transposed()
        right = q @ Vector((1, 0, 0))
        up = q @ Vector((0, 1, 0))
        k = focal / (sensor / 2.0)

        def extent(d):
            cam = target + direction * d
            umin = vmin = 1e9
            umax = vmax = -1e9
            for p in pts:
                c = inv @ (p - cam)
                if c.z >= -1e-6:
                    return 1e9, 0, 0
                u = k * c.x / (-c.z)
                v = k * c.y / (-c.z) * aspect
                umin, umax = min(umin, u), max(umax, u)
                vmin, vmax = min(vmin, v), max(vmax, v)
            return max(umax - umin, vmax - vmin) / 2.0, (umin + umax) / 2, (vmin + vmax) / 2

        lo, hi = 0.05, 20.0
        for _ in range(50):
            mid = (lo + hi) / 2
            e, _, _ = extent(mid)
            if e * margin > 1.0:
                lo = mid
            else:
                hi = mid
        d = hi
        e, cu, cv = extent(d)
        half = d * (sensor / 2.0) / focal
        target = target + right * (cu * half) + up * (cv * half / aspect)
    return target + direction * d, target


def setup_camera(view="hero", scene=None):
    scene = scene or bpy.context.scene
    cfg = VIEWS[view]
    cam = scene.camera
    cd = cam.data
    cd.lens = cfg["focal"]
    cd.sensor_fit = "AUTO"
    cd.sensor_width = 36.0
    cd.clip_start = 0.01
    cd.clip_end = 100.0
    fit = cfg["fit"]
    if fit is None:
        fit = (BBOX_MIN, BBOX_MAX)
    lo, hi = fit
    pts = [Vector((x * MM, y * MM, z * MM)) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    centre = sum(pts, Vector()) / len(pts) + Vector(cfg["shift"]) * MM
    az, el = math.radians(cfg["azim"]), math.radians(cfg["elev"])
    direction = Vector((math.sin(az) * math.cos(el), -math.cos(az) * math.cos(el), math.sin(el)))
    aspect = scene.render.resolution_x / scene.render.resolution_y
    loc, tgt = _fit_camera(direction, centre, cfg["focal"], aspect, pts, cfg["margin"])
    cam.location = loc
    cam.rotation_euler = (tgt - loc).to_track_quat("-Z", "Y").to_euler()
    return loc, tgt


# ---------------------------------------------------------------------------------------------- render
def render(out_path, samples=None, res=None, scene=None, view=None):
    scene = scene or bpy.context.scene
    if samples is not None:
        scene.cycles.samples = samples
    if res is not None:
        scene.render.resolution_x, scene.render.resolution_y = res
        if view:
            setup_camera(view, scene)
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    ext = os.path.splitext(out_path)[1].lower()
    scene.render.image_settings.file_format = "PNG" if ext == ".png" else "JPEG"
    scene.render.filepath = out_path
    t0 = time.time()
    bpy.ops.render.render(write_still=True)
    return time.time() - t0


def rod_gap_check(step_deg=30):
    """Max distance (mm) between the baked far end of conn_rod and the carriage origin over theta."""
    rod = bpy.data.objects["conn_rod"]
    car = bpy.data.objects["carriage"]
    # far end of the rod = carriage pin position in pose0, expressed in the rod's local frame (100 mm away)
    k = kinematics.K
    ro, co = k["rod"]["origin_pose0"], k["carriage"]["origin_pose0"]
    far = Vector(((co[0] - ro[0]) * MM, (co[1] - ro[1]) * MM, (co[2] - ro[2]) * MM))
    gaps = {}
    for d in range(0, 360, step_deg):
        set_pose(math.radians(d), True)
        pe = rod.matrix_world @ far
        gaps[d] = (pe - car.matrix_world.translation).length * 1000.0
    return gaps, far


# ---------------------------------------------------------------------------------------------- CLI
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--view", default="hero", choices=sorted(VIEWS))
    ap.add_argument("--theta-deg", type=float, default=35.0)
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--on", dest="on", action="store_true", default=True)
    g.add_argument("--off", dest="on", action="store_false")
    ap.add_argument("--samples", type=int, default=64)
    ap.add_argument("--res", default="1600x900")
    ap.add_argument("--out", default=os.path.join(ROOT, "web", "renders", "test.jpg"))
    ap.add_argument("--save-blend", default=None)
    a = ap.parse_args(argv)
    res = tuple(int(v) for v in a.res.lower().split("x"))
    build_scene(a.theta_deg, a.on, a.view, a.samples, res)
    if a.save_blend:
        bpy.ops.wm.save_as_mainfile(filepath=a.save_blend)
    t = render(a.out, a.samples, res)
    print(f"rendered {a.out} {res[0]}x{res[1]} samples={a.samples} in {t:.1f}s")


if __name__ == "__main__":
    main(sys.argv[1:])
