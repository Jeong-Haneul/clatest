"""조립 간섭 검사. 0~360° 크랭크 각마다 모든 부품 쌍의 침투 깊이를 fcl(트라이앵글 메시 정밀 충돌)로 측정.

실행:  /tmp/.../venv/bin/python tools/check_assembly.py [--step 10] [--min-depth 0.05]
필요:  pip install trimesh python-fcl networkx numpy
출력:  부품 쌍별 (충돌한 각도 수, 최대 침투 깊이 mm). 의도된 접촉(축-구멍, 기어 맞물림 등)은 ALLOW 표로 분류.
"""
import argparse
import itertools
import math
import os
import re
import sys

import numpy as np
import trimesh

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import kinematics as KIN  # noqa: E402

PARTS = ["housing", "drive", "slide", "electronics", "figure"]
MOVING = {"gear_P1", "gear_G1P2", "gear_G2P3", "gear_G3", "crank_pin_assy", "conn_rod", "carriage",
          "figure_body", "figure_wheel_front", "figure_wheel_back", "figure_antenna", "switch_knob"}
# 노드 원점 (pose0). 정점은 노드 로컬 → 이 원점을 더해 조립 좌표로.
G2Z = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float) * 1000.0   # glTF(x,y,z) -> Zup mm (x,-z,y)


def load_nodes():
    out = {}
    for p in PARTS:
        sc = trimesh.load(os.path.join(ROOT, "assets", "parts", p + ".glb"), force="scene")
        for nname in sc.graph.nodes_geometry:
            T, gname = sc.graph[nname]
            g = sc.geometry[gname].copy()
            v = np.c_[g.vertices, np.ones(len(g.vertices))] @ T.T      # 노드 월드(glTF)
            v = v[:, :3] @ G2Z.T
            origin = (np.array(T[:3, 3]) @ G2Z.T)                     # 노드 원점 (Zup mm)
            base = re.sub(r"_[0-9a-f]{6}$", "", nname)      # trimesh 가 재질 프리미티브마다 붙인 해시 제거
            out.setdefault(base, dict(vs=[], fs=[], n=0, origin=origin, part=p))
            e = out[base]
            e["fs"].append(g.faces + e["n"]); e["vs"].append(v); e["n"] += len(v)
    for e in out.values():
        e["mesh"] = trimesh.Trimesh(np.vstack(e["vs"]), np.vstack(e["fs"]), process=False)
        del e["vs"], e["fs"]
    return out


def rot_y_front_ccw(a):
    # 앞(-y)에서 본 CCW = -Y축 오른손 회전
    c, s = math.cos(a), math.sin(a)
    # 회전축 (0,-1,0): x' = c x - s z ; z' = s x + c z   (x→z 방향이 CCW)
    return np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])


def node_transform(node, loc, rot):
    R = np.eye(4)
    R[:3, :3] = rot_y_front_ccw(rot)
    T = np.eye(4)
    T[:3, 3] = np.array(loc) - 0           # 목표 원점 위치
    O = np.eye(4)
    O[:3, 3] = -node["origin"]             # 원점을 (0,0,0)으로
    return T @ R @ O


# 의도된 접촉(축이 구멍을 통과, 체결 나사, 기어 이 맞물림 등) — 깊이가 이 값 이하면 허용
ALLOW = {
    frozenset(("gear_P1", "gear_G1P2")): 0.8, frozenset(("gear_G1P2", "gear_G2P3")): 0.8, frozenset(("gear_G2P3", "gear_G3")): 0.8,
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", type=float, default=10.0)
    ap.add_argument("--min-depth", type=float, default=0.05)
    a = ap.parse_args()
    N = load_nodes()
    print(f"{len(N)} nodes loaded: " + ", ".join(sorted(N)[:6]) + " ...")
    names = sorted(N)
    res = {}
    for deg in np.arange(0, 360, a.step):
        th = math.radians(deg)
        for on in (False,):
            P = KIN.pose(th, on)
            mgr = trimesh.collision.CollisionManager()
            for n in names:
                p = P.get(n)
                T = node_transform(N[n], p["loc"], p["rot"]) if p and n in MOVING else np.eye(4)
                mgr.add_object(n, N[n]["mesh"], T)
            hit, pairs, data = mgr.in_collision_internal(return_names=True, return_data=True)
            for pr, d in zip(pairs, data):
                pr = frozenset(pr)
                a_, b_ = sorted(pr)
                if N[a_]["part"] == N[b_]["part"] and a_ not in MOVING and b_ not in MOVING:
                    continue            # 같은 부품의 정적 노드끼리는 설계상 이미 접촉 가능 (하우징 판 맞물림 등)
                depth = max(c.depth for c in d) if hasattr(d, "__iter__") else d.depth
                r = res.setdefault(pr, [0, 0.0, []])
                r[0] += 1; r[1] = max(r[1], depth); r[2].append(int(deg))
    total = len(np.arange(0, 360, a.step))
    print(f"\n충돌 쌍 {len(res)}개 (step {a.step}°, 깊이 {a.min_depth} mm 이상). fcl 깊이는 메시 BVH 근사라 참고용입니다.")
    static, dyn = [], []
    for pr, (cnt, dep, degs) in res.items():
        if dep < a.min_depth:
            continue
        (static if cnt == total else dyn).append((pr, cnt, dep, degs))
    print("\n[각도 무관 접촉 = 삽입/체결 등 설계된 정적 접촉으로 간주]")
    for pr, cnt, dep, degs in sorted(static, key=lambda t: -t[2]):
        print(f"  {' × '.join(sorted(pr)):52s} {dep:5.2f} mm")
    print("\n[각도 의존 접촉 = 운동 중 간섭 가능성 → 검토]")
    if not dyn:
        print("  없음")
    for pr, cnt, dep, degs in sorted(dyn, key=lambda t: -t[2]):
        lim = ALLOW.get(pr, 0.0)
        flag = "OK(기어 맞물림)" if dep <= lim else "**검토**"
        print(f"  {flag:12s} {' × '.join(sorted(pr)):48s} {dep:5.2f} mm  {cnt}/{total}개 각도 ({min(degs)}..{max(degs)}°)")


if __name__ == "__main__":
    main()
