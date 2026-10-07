// 오토마타 기구학 — tools/kinematics.py 와 같은 수식. 스펙: ../spec/kinematics.json
// 좌표: 조립체 Z-up mm → three.js(glTF Y-up, m)로 바꾸는 건 toGltf() 가 담당.

export async function loadSpec(url = '../spec/kinematics.json') {
  const K = await (await fetch(url)).json();
  const [cx, cz] = K.crank.center;
  const R = K.crank.radius, L = K.crank.rod_length, ZR = K.crank.carriage_pin_z;

  const pinX = (t) => cx + R * Math.cos(t);
  const pinZ = (t) => cz + R * Math.sin(t);
  const carriageX = (t) => pinX(t) + Math.sqrt(L * L - (ZR - pinZ(t)) ** 2);
  const rodPhi = (t) => Math.atan2(ZR - pinZ(t), carriageX(t) - pinX(t));
  const X0 = carriageX(0), PHI0 = rodPhi(0);

  let lo = Infinity, hi = -Infinity;
  for (let d = 0; d < 360; d += 0.5) { const x = carriageX(d * Math.PI / 180); lo = Math.min(lo, x); hi = Math.max(hi, x); }

  const figOrigin = {
    figure_body: [26.68, -36.0, 110.0], figure_wheel_front: [26.68, -43.3, 125.0],
    figure_wheel_back: [26.68, -28.7, 125.0], figure_antenna: [26.68, -36.0, 164.2],
  };

  // theta: 크랭크 각(정면 CCW, rad), knobT: 0(OFF)..1(ON)
  function pose(theta, knobT = 0) {
    const out = {};
    for (const [name, g] of Object.entries(K.gears)) {
      if (name.startsWith('_')) continue;
      out[name] = { loc: g.origin, rot: g.ratio * theta };
    }
    out.conn_rod = { loc: [pinX(theta), K.rod.origin_pose0[1], pinZ(theta)], rot: rodPhi(theta) - PHI0 };
    const xc = carriageX(theta), c0 = K.carriage.origin_pose0;
    out.carriage = { loc: [xc, c0[1], c0[2]], rot: 0 };
    const dx = xc - X0;
    for (const name of K.figure.follow_carriage_dx) {
      const o = figOrigin[name];
      out[name] = { loc: [o[0] + dx, o[1], o[2]], rot: K.figure.wheels.nodes.includes(name) ? -dx / K.figure.wheels.radius : 0 };
    }
    const k = K.electronics.switch_knob.origin;
    out.switch_knob = { loc: [k[0] + K.electronics.switch_knob.travel_x_on * knobT, k[1], k[2]], rot: 0 };
    return out;
  }

  const toGltf = ([x, y, z]) => [x / 1000, z / 1000, -y / 1000];
  return { K, pose, carriageX, rodPhi, X0, stroke: [lo, hi], toGltf, R, L };
}
