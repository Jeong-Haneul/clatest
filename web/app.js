import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { loadSpec } from './kinematics.js';

const $ = (s, r = document) => r.querySelector(s);
const PARTS = ['housing', 'drive', 'slide', 'electronics', 'figure'];
const DEG = Math.PI / 180;

// ---------------------------------------------------------------- 렌더러 / 장면
const canvas = $('#gl');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 0.9;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;

const scene = new THREE.Scene();
const pmrem = new THREE.PMREMGenerator(renderer);
scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
scene.environmentIntensity = 0.5;
const stageColor = () => getComputedStyle(document.documentElement).getPropertyValue('--stage').trim() || '#d9dce0';
scene.background = new THREE.Color(stageColor());
matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => { scene.background.set(stageColor()); });

const camera = new THREE.PerspectiveCamera(32, 16 / 10, 0.01, 10);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.minDistance = 0.08;
controls.maxDistance = 1.2;

const key = new THREE.DirectionalLight(0xffffff, 2.4);
key.position.set(0.3, 0.6, 0.5);
key.castShadow = true;
key.shadow.mapSize.set(2048, 2048);
Object.assign(key.shadow.camera, { left: -0.25, right: 0.25, top: 0.25, bottom: -0.25, near: 0.05, far: 2 });
key.shadow.bias = -0.0004;
scene.add(key, new THREE.HemisphereLight(0xffffff, 0x8a8f99, 0.35));

const floor = new THREE.Mesh(new THREE.PlaneGeometry(3, 3), new THREE.ShadowMaterial({ opacity: 0.22 }));
floor.rotation.x = -Math.PI / 2;
floor.position.y = -0.0031;
floor.receiveShadow = true;
scene.add(floor);

function resize() {
  const r = canvas.parentElement.getBoundingClientRect();
  renderer.setSize(r.width, r.height, false);
  camera.aspect = r.width / r.height;
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(canvas.parentElement);

// ---------------------------------------------------------------- 시점 프리셋 (glTF Y-up, m)
const VIEWS = {
  three:  { pos: [0.20, 0.17, 0.30],  tgt: [0.00, 0.060, -0.01] },
  front:  { pos: [0.00, 0.060, 0.55], tgt: [0.00, 0.060, 0.00] },
  top:    { pos: [0.00, 0.60, 0.02],  tgt: [0.00, 0.00, -0.01] },
  side:   { pos: [0.55, 0.07, 0.00],  tgt: [0.00, 0.070, 0.00] },
  gears:  { pos: [-0.04, 0.07, 0.22], tgt: [-0.055, 0.045, 0.00] },
  figure: { pos: [0.07, 0.19, 0.17],  tgt: [0.00, 0.14, -0.035] },
};
let tween = null;
function goView(name, instant = false) {
  const v = VIEWS[name];
  if (!v) return;
  if (instant) { camera.position.set(...v.pos); controls.target.set(...v.tgt); controls.update(); return; }
  tween = { t: 0, p0: camera.position.clone(), t0: controls.target.clone(), p1: new THREE.Vector3(...v.pos), t1: new THREE.Vector3(...v.tgt) };
}
document.querySelectorAll('.views button').forEach((b) => b.addEventListener('click', () => goView(b.dataset.view)));
controls.addEventListener('start', () => { tween = null; });

// ---------------------------------------------------------------- 상태
const state = { on: false, motor: 0, theta: 0, knob: 0, speedRpm: 12, explode: 0, lastTheta: 0, antA: 0, antV: 0, lastDx: 0, lastVx: 0 };
const nodes = {};     // 이름 → three Object3D
const groups = {};    // 부품 이름 → 루트 그룹
let sim = null, ledMat = null;

// ---------------------------------------------------------------- 로드
const manager = new THREE.LoadingManager();
manager.onProgress = (_u, i, n) => { $('#loadbar').style.width = (100 * i / n) + '%'; $('#loadtxt').textContent = `부품 불러오는 중… ${i}/${n}`; };
const loader = new GLTFLoader(manager);

const ghostable = [];   // 하우징 메시: 고스트 모드용 재질 복제본
async function load() {
  sim = await loadSpec();
  $('#stroke').textContent = (sim.stroke[1] - sim.stroke[0]).toFixed(1);
  $('#stat').textContent = `감속 4×4×5 = 80:1 · 크랭크 반지름 ${sim.R} mm · 로드 ${sim.L} mm · 행정 ${(sim.stroke[1] - sim.stroke[0]).toFixed(1)} mm`;
  await Promise.all(PARTS.map((p) => loader.loadAsync(`../assets/parts/${p}.glb`).then((g) => {
    const root = g.scene;
    root.name = p;
    root.traverse((o) => {
      if (o.name) nodes[o.name] = o;
      if (o.isMesh) {
        o.castShadow = true; o.receiveShadow = true;
        const mats = Array.isArray(o.material) ? o.material : [o.material];
        mats.forEach((m) => {
          if (m.transparent) { m.depthWrite = false; m.side = THREE.DoubleSide; o.renderOrder = 2; o.castShadow = false; m.envMap = scene.environment; m.envMapIntensity = 0.12; }
          m.envMapIntensity = 1.0;
        });
        if (p === 'housing') {
          o.material = Array.isArray(o.material) ? o.material.map((m) => m.clone()) : o.material.clone();
          ghostable.push(o);
        }
        if (p === 'electronics' && o.name === 'led_lens') {
          const m = (Array.isArray(o.material) ? o.material : [o.material]).find((x) => x.name === 'led_lens');
          if (m) ledMat = m;
        }
      }
    });
    groups[p] = root;
    scene.add(root);
  })));
  if (ledMat) { ledMat = ledMat.clone(); nodes.led_lens.traverse((o) => { if (o.isMesh) o.material = ledMat; }); }
}

// ---------------------------------------------------------------- 분해도 (부품 그룹 이동, glTF m)
const EXPLODE = {
  housing: [0, 0, 0],
  drive: [0, 0, 0.070],
  slide: [0, 0, 0.035],
  electronics: [0, -0.001, -0.050],
  figure: [0, 0.06, 0],
};
const housingParts = { housing_front: [0, 0, 0.12], housing_lid: [0, 0.045, 0], housing_walls: [0, 0, 0], housing_base: [0, -0.02, 0] };
function applyExplode() {
  const e = state.explode;
  for (const p of PARTS) groups[p]?.position.set(...EXPLODE[p].map((v) => v * e));
  for (const [n, o] of Object.entries(housingParts)) nodes[n]?.position.set(...o.map((v) => v * e));
}

// ---------------------------------------------------------------- 포즈 적용
function applyPose() {
  const P = sim.pose(state.theta, state.knob);
  for (const [name, p] of Object.entries(P)) {
    const o = nodes[name];
    if (!o) continue;
    const [x, y, z] = sim.toGltf(p.loc);
    o.position.set(x, y, z);
    o.rotation.set(0, 0, p.rot);
  }
  // 안테나: 캐리지 가속에 따른 관성 흔들림 (노드 원점이 안테나 밑동)
  const ant = nodes.figure_antenna;
  if (ant) ant.rotation.z = state.antA;
  return P;
}

// ---------------------------------------------------------------- UI
const sw = $('#sw');
function setSwitch(on) {
  state.on = on;
  sw.setAttribute('aria-checked', on);
  $('.lbl', sw).textContent = on ? 'ON' : 'OFF';
  $('#theta').disabled = on;
}
sw.addEventListener('click', () => setSwitch(!state.on));
addEventListener('keydown', (e) => { if (e.code === 'Space' && !/INPUT|BUTTON/.test(document.activeElement.tagName)) { e.preventDefault(); setSwitch(!state.on); } });
$('#speed').addEventListener('input', (e) => { state.speedRpm = +e.target.value; $('#o-speed').textContent = `${e.target.value} rpm`; });
$('#theta').addEventListener('input', (e) => { state.theta = +e.target.value * DEG; $('#o-theta').textContent = `${e.target.value}°`; });
$('#explode').addEventListener('input', (e) => { state.explode = +e.target.value / 100; $('#o-explode').textContent = `${e.target.value}%`; applyExplode(); });
document.querySelectorAll('[data-part]').forEach((c) => c.addEventListener('change', () => { if (groups[c.dataset.part]) groups[c.dataset.part].visible = c.checked; }));
$('#ghost').addEventListener('change', (e) => {
  for (const o of ghostable) {
    (Array.isArray(o.material) ? o.material : [o.material]).forEach((m) => {
      if (m.name.startsWith('acrylic')) return;
      m.transparent = e.target.checked; m.opacity = e.target.checked ? 0.14 : 1; m.depthWrite = !e.target.checked;
    });
    o.renderOrder = e.target.checked ? 1 : 0;
  }
});

// ---------------------------------------------------------------- 그래프
function drawGraph() {
  const c = $('#graph'), g = c.getContext('2d');
  const cs = getComputedStyle(document.documentElement);
  const ink = cs.getPropertyValue('--ink').trim(), muted = cs.getPropertyValue('--muted').trim(), acc = cs.getPropertyValue('--accent').trim(), line = cs.getPropertyValue('--line').trim();
  const W = c.width, H = c.height, pad = { l: 56, r: 16, t: 16, b: 36 };
  g.clearRect(0, 0, W, H);
  const [lo, hi] = sim.stroke;
  const X = (deg) => pad.l + (W - pad.l - pad.r) * deg / 360;
  const Y = (x) => pad.t + (H - pad.t - pad.b) * (1 - (x - (lo - 4)) / (hi - lo + 8));
  g.font = '12px system-ui, sans-serif'; g.fillStyle = muted; g.strokeStyle = line; g.lineWidth = 1;
  for (let d = 0; d <= 360; d += 90) { g.beginPath(); g.moveTo(X(d), pad.t); g.lineTo(X(d), H - pad.b); g.stroke(); g.textAlign = 'center'; g.fillText(d + '°', X(d), H - 14); }
  for (const v of [Math.round(lo), 0, Math.round(hi)]) { g.beginPath(); g.moveTo(pad.l, Y(v)); g.lineTo(W - pad.r, Y(v)); g.stroke(); g.textAlign = 'right'; g.fillText(v + ' mm', pad.l - 6, Y(v) + 4); }
  g.strokeStyle = acc; g.lineWidth = 2.5; g.beginPath();
  for (let d = 0; d <= 360; d += 2) { const x = sim.carriageX(d * DEG); d ? g.lineTo(X(d), Y(x)) : g.moveTo(X(d), Y(x)); }
  g.stroke();
  const deg = ((state.theta / DEG) % 360 + 360) % 360, x = sim.carriageX(state.theta);
  g.fillStyle = ink; g.beginPath(); g.arc(X(deg), Y(x), 5.5, 0, 7); g.fill();
  g.fillStyle = muted; g.textAlign = 'left'; g.fillText('캐리지 x (mm) / 크랭크 각도', pad.l + 6, pad.t + 12);
}
matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => sim && drawGraph());

// ---------------------------------------------------------------- 메인 루프
const clock = new THREE.Clock();
let hudT = 0;
function tick() {
  const dt = Math.min(clock.getDelta(), 0.05);
  // 모터: 스위치에 따라 1차 지연으로 가속/감속
  const target = state.on ? 1 : 0;
  state.motor += (target - state.motor) * (1 - Math.exp(-dt / (state.on ? 0.35 : 0.7)));
  if (state.motor < 0.002 && !state.on) state.motor = 0;
  const w = state.motor * state.speedRpm * 2 * Math.PI / 60;       // rad/s (크랭크)
  state.theta += w * dt;
  state.knob += ((state.on ? 1 : 0) - state.knob) * (1 - Math.exp(-dt / 0.05));
  if (!state.on && state.motor === 0) { /* 수동 모드: 슬라이더 값 유지 */ } else { $('#theta').value = ((state.theta / DEG) % 360 + 360) % 360; $('#o-theta').textContent = `${Math.round(+$('#theta').value)}°`; }

  const P = applyPose();
  // 안테나 스프링-댐퍼 (캐리지 가속이 입력)
  const dx = P.carriage.loc[0];
  const vx = (dx - state.lastDx) / Math.max(dt, 1e-4);
  const ax = (vx - state.lastVx) / Math.max(dt, 1e-4);
  state.lastDx = dx; state.lastVx = vx;
  if (isFinite(ax)) {
    const k = 260, c = 7, drive = THREE.MathUtils.clamp(ax * 0.0000018, -0.5, 0.5);
    state.antV += (-k * state.antA - c * state.antV + drive * k * 60) * dt;
    state.antA = THREE.MathUtils.clamp(state.antA + state.antV * dt, -0.35, 0.35);
  }
  // LED
  if (ledMat) {
    const lit = state.on ? Math.min(1, state.motor * 3 + 0.2) : state.motor * 0.4;
    ledMat.emissive = ledMat.emissive || new THREE.Color();
    ledMat.emissive.setRGB(1.0, 0.08, 0.04);
    ledMat.emissiveIntensity = lit * 3.2;
  }

  if (tween) {
    tween.t = Math.min(1, tween.t + dt / 0.7);
    const k = tween.t * tween.t * (3 - 2 * tween.t);
    camera.position.lerpVectors(tween.p0, tween.p1, k);
    controls.target.lerpVectors(tween.t0, tween.t1, k);
    if (tween.t >= 1) tween = null;
  }
  controls.update();
  renderer.render(scene, camera);

  hudT += dt;
  if (hudT > 0.08 && sim) {
    hudT = 0;
    const deg = ((state.theta / DEG) % 360 + 360) % 360;
    const rpm = w * 60 / (2 * Math.PI);
    $('#h-theta').textContent = `${deg.toFixed(0)}°`;
    $('#h-x').textContent = `${sim.carriageX(state.theta).toFixed(1)} mm`;
    $('#h-rpm').textContent = `${rpm.toFixed(0)} rpm`;
    $('#h-motor').textContent = `${(rpm * 80).toFixed(0)} rpm`;
    drawGraph();
  }
  requestAnimationFrame(tick);
}

// ---------------------------------------------------------------- 렌더 갤러리: 없는 이미지는 숨김
document.querySelectorAll('#r-grid img[data-src]').forEach((img) => {
  const io = new IntersectionObserver((es) => es.forEach((e) => { if (e.isIntersecting) { img.src = img.dataset.src; io.disconnect(); } }));
  io.observe(img);
  img.addEventListener('error', () => img.closest('figure').classList.add('missing'));
});
const mv = $('#motion');
mv.querySelectorAll('source').forEach((s) => s.addEventListener('error', () => { if (mv.networkState === 3) mv.closest('figure').classList.add('missing'); }));
mv.addEventListener('error', () => mv.closest('figure').classList.add('missing'), true);
new IntersectionObserver((es) => es.forEach((e) => { if (e.isIntersecting) mv.play().catch(() => {}); else mv.pause(); })).observe(mv);

// ---------------------------------------------------------------- 시작
resize();
goView('three', true);
load().then(() => {
  applyPose(); applyExplode(); drawGraph();
  $('#loading').classList.add('done');
  window.__automaton = { state, nodes, groups, sim, setSwitch, goView, THREE, camera, scene, renderer };
  tick();
}).catch((err) => { console.error(err); $('#loadtxt').textContent = '불러오기 실패: ' + err.message + ' (로컬 서버로 여세요: python3 -m http.server)'; });
