"""오토마타 기구학 (spec/kinematics.json 기준). 웹(web/kinematics.js)과 동일한 수식.

pose(theta, switch_on) -> {노드이름: {"loc": (x,y,z) mm Z-up 절대위치, "rot": 정면 CCW rad (노드 원점 기준)}}
Blender 적용: rotation_euler.y = -rot (앞 -y에서 본 CCW), location = loc/1000 (GLB 임포트 후 단위 m).
"""
import json
import math
import os

_SPEC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "spec", "kinematics.json")
with open(_SPEC, encoding="utf-8") as _f:
    K = json.load(_f)

_cx, _cz = K["crank"]["center"]
_R = K["crank"]["radius"]
_L = K["crank"]["rod_length"]
_ZR = K["crank"]["carriage_pin_z"]


def carriage_x(theta):
    pz = _cz + _R * math.sin(theta)
    px = _cx + _R * math.cos(theta)
    return px + math.sqrt(_L * _L - (_ZR - pz) ** 2)


def rod_phi(theta):
    pz = _cz + _R * math.sin(theta)
    return math.atan2(_ZR - pz, carriage_x(theta) - (_cx + _R * math.cos(theta)))


X0 = carriage_x(0.0)
PHI0 = rod_phi(0.0)


def stroke():
    xs = [carriage_x(math.radians(d)) for d in range(0, 360)]
    return min(xs), max(xs)


def pose(theta, switch_on=False):
    out = {}
    for name, g in K["gears"].items():
        if name.startswith("_"):
            continue
        out[name] = {"loc": tuple(g["origin"]), "rot": g["ratio"] * theta}
    pin = (_cx + _R * math.cos(theta), K["rod"]["origin_pose0"][1], _cz + _R * math.sin(theta))
    out["conn_rod"] = {"loc": pin, "rot": rod_phi(theta) - PHI0}
    xc = carriage_x(theta)
    c0 = K["carriage"]["origin_pose0"]
    out["carriage"] = {"loc": (xc, c0[1], c0[2]), "rot": 0.0}
    dx = xc - X0
    fig = K["figure"]
    for name in fig["follow_carriage_dx"]:
        o = {"figure_body": (26.68, -36.0, 110.0), "figure_wheel_front": (26.68, -43.3, 125.0),
             "figure_wheel_back": (26.68, -28.7, 125.0), "figure_antenna": (26.68, -36.0, 164.2)}[name]
        rot = -dx / fig["wheels"]["radius"] if name in fig["wheels"]["nodes"] else 0.0
        out[name] = {"loc": (o[0] + dx, o[1], o[2]), "rot": rot}
    k = K["electronics"]["switch_knob"]
    o = k["origin"]
    out["switch_knob"] = {"loc": (o[0] + (k["travel_x_on"] if switch_on else 0.0), o[1], o[2]), "rot": 0.0}
    return out


if __name__ == "__main__":
    lo, hi = stroke()
    print(f"carriage x range {lo:.2f} .. {hi:.2f}  stroke {hi - lo:.2f} mm, x0={X0:.3f}, phi0={math.degrees(PHI0):.2f} deg")
    for d in (0, 90, 180, 270):
        p = pose(math.radians(d))
        print(d, "carriage", round(p["carriage"]["loc"][0], 2), "rod rot deg", round(math.degrees(p["conn_rod"]["rot"]), 2))
