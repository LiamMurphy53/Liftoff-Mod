// Geometry Lab drone editor. Served by the Liftoff plugin on 127.0.0.1; talks to it over /api.
//
// Coordinates. Every point the user sees or types is in the CAD model's own coordinates and units
// (so Onshape's numbers can be pasted as-is). The import settings (units, up axis, forward axis) map
// those to the profile's axes: X left, Y back, Z up, in meters. On save the editor also shifts the
// profile origin to the motor centre (the plugin only uses positions relative to each other) and
// remembers that shift so the numbers come back unchanged.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { DIRS, RHO, blend, shepard, median, silhouettes, runResult, describe, parseForces, parseRunsCsv, drawPolar } from './drag.js';

const $ = id => document.getElementById(id);
const G = 9.80665;
const AXES = { '+X': [1, 0, 0], '-X': [-1, 0, 0], '+Y': [0, 1, 0], '-Y': [0, -1, 0], '+Z': [0, 0, 1], '-Z': [0, 0, -1] };
const UNIT = { mm: 0.001, cm: 0.01, m: 1, in: 0.0254 };
const DECIMALS = { mm: 2, cm: 3, m: 5, in: 4 };
const IUNIT = { 'kg·m²': 1, 'kg·mm²': 1e-6, 'g·mm²': 1e-9, 'lb·in²': 0.45359237 * 0.0254 * 0.0254 };
const MOTORS = ['FL', 'FR', 'RL', 'RR'];
const MOTOR_NAMES = ['front-left', 'front-right', 'rear-left', 'rear-right'];
const BODY_FILE = 'body.glmb';
const MAX_VERTICES = 3_000_000;
const DEFAULT_ED = { units: 'mm', up: '+Z', forward: '-Y', shift: [0, 0, 0], iunit: 'kg·m²', prop: 5, tint: false, tintColor: '#2b2f36', source: '' };
// Drag source: Liftoff's own, the CAD silhouette times one Cd, or CFD runs ({name, u: CAD-axis m/s, f: CAD-axis N}).
const DEFAULT_DRAG = { mode: 'liftoff', cd: 1.1, rho: RHO, mirror: true, runs: [] };
const freshEd = (saved = {}) => ({ ...DEFAULT_ED, ...saved, drag: { ...DEFAULT_DRAG, runs: [], ...(saved.drag || {}) } });

// ---------------------------------------------------------------- math

const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const mul = (M, v) => M.map(r => r[0] * v[0] + r[1] * v[1] + r[2] * v[2]);
const tr = M => [0, 1, 2].map(i => M.map(r => r[i]));
const mm = (A, B) => A.map(r => [0, 1, 2].map(j => r[0] * B[0][j] + r[1] * B[1][j] + r[2] * B[2][j]));
const add = (a, b) => a.map((v, i) => v + b[i]);
const sub = (a, b) => a.map((v, i) => v - b[i]);
const scale = (a, k) => a.map(v => v * k);
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const norm = a => Math.hypot(...a);

let ed = freshEd();
// Rows: the profile's left, back and up axes written in CAD coordinates.
const basis = () => { const u = AXES[ed.up], f = AXES[ed.forward]; return [cross(u, f), scale(f, -1), u]; };
const toProfile = p => add(scale(mul(basis(), p), UNIT[ed.units]), ed.shift);
const fromProfile = q => scale(mul(tr(basis()), sub(q, ed.shift)), 1 / UNIT[ed.units]);
// three.js world: x = right (−left), y = up, z = back. A proper rotation of the profile axes.
const VIEW = [[-1, 0, 0], [0, 0, 1], [0, 1, 0]];
const cadToWorld = p => new THREE.Vector3(...mul(VIEW, toProfile(p)));
const worldToCad = w => fromProfile(mul(tr(VIEW), [w.x, w.y, w.z]));

// Circle through three points in 3D: the centre of a motor bell from three clicks on its rim.
function circleCentre(a, b, c) {
  const ab = sub(b, a), ac = sub(c, a), n = cross(ab, ac), n2 = dot(n, n);
  if (n2 < 1e-18) return null;
  const t = add(scale(cross(n, ab), dot(ac, ac)), scale(cross(ac, n), dot(ab, ab)));
  return add(a, scale(t, 1 / (2 * n2)));
}

// Monotone cubic (PCHIP) through the measured points; the same curve the plugin flies.
function pchip(xs, ys) {
  const x = [...xs], y = [...ys];
  if (x[0] > 0) { x.unshift(0); y.unshift(0); }
  const n = x.length, d = [], m = new Array(n);
  for (let i = 0; i < n - 1; i++) d.push((y[i + 1] - y[i]) / (x[i + 1] - x[i]));
  m[0] = d[0]; m[n - 1] = d[n - 2];
  for (let i = 1; i < n - 1; i++) {
    if (d[i - 1] * d[i] <= 0) { m[i] = 0; continue; }
    const h0 = x[i] - x[i - 1], h1 = x[i + 1] - x[i];
    m[i] = 3 * (h0 + h1) / ((2 * h1 + h0) / d[i - 1] + (h1 + 2 * h0) / d[i]);
  }
  const f = t => {
    t = Math.max(0, Math.min(x[n - 1], t));
    let i = 0; while (i < n - 2 && t > x[i + 1]) i++;
    const h = x[i + 1] - x[i], s = (t - x[i]) / h, s2 = s * s, s3 = s2 * s;
    return (2 * s3 - 3 * s2 + 1) * y[i] + (s3 - 2 * s2 + s) * h * m[i] + (-2 * s3 + 3 * s2) * y[i + 1] + (s3 - s2) * h * m[i + 1];
  };
  f.inverse = target => {
    if (target <= 0) return 0;
    if (target >= y[n - 1]) return x[n - 1];
    let lo = 0, hi = x[n - 1];
    for (let k = 0; k < 30; k++) { const mid = (lo + hi) / 2; if (f(mid) < target) lo = mid; else hi = mid; }
    return hi;
  };
  f.max = y[n - 1];
  return f;
}

// ---------------------------------------------------------------- state

let profile = null;               // last profile from the plugin
let parts = null;                 // [{geometry (CAD coords), color: THREE.Color}], with a unique .id
let partsSeq = 0;
let baked = null;                 // settings baked into the uploaded body file (null: needs upload)
const cad = { motors: [[60, -60, 0], [-60, -60, 0], [60, 60, 0], [-60, 60, 0]], cg: [0, 0, 20], camera: [0, -70, 20], inertia: [[0.003, 0, 0], [0, 0.003, 0], [0, 0, 0.005]] };
let rows = [];                    // thrust curve: [{pct, g}]
let pick = null;                  // {target: 'm0'…'m3' | 'cg' | 'camera', clicks: []}
let dirty = false;

// ---------------------------------------------------------------- 3D view

const canvas = $('canvas'), view = $('view');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(40, 1, 0.001, 100);
camera.position.set(0.25, 0.2, 0.35);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true; controls.dampingFactor = 0.12;
scene.add(new THREE.HemisphereLight(0xdbe6ff, 0x1a1d22, 1.1));
const sun = new THREE.DirectionalLight(0xffffff, 1.6); sun.position.set(0.6, 1, 0.4); scene.add(sun);
const rim = new THREE.DirectionalLight(0x9fb4ff, 0.6); rim.position.set(-0.7, 0.3, -0.8); scene.add(rim);
const grid = new THREE.GridHelper(1, 20, 0x2b323c, 0x1b2027); scene.add(grid);
const modelGroup = new THREE.Group(); modelGroup.matrixAutoUpdate = false; scene.add(modelGroup);
const markers = new THREE.Group(); scene.add(markers);
const hover = new THREE.Mesh(new THREE.SphereGeometry(1, 16, 12), new THREE.MeshBasicMaterial({ color: 0xffffff, depthTest: false, transparent: true, opacity: 0.9 }));
hover.renderOrder = 10; hover.visible = false; scene.add(hover);
const clickDots = new THREE.Group(); scene.add(clickDots);

function resize() {
  const r = view.getBoundingClientRect();
  renderer.setSize(r.width, r.height, false);
  camera.aspect = r.width / Math.max(1, r.height); camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(view);

function sceneSize() {
  const box = new THREE.Box3();
  if (modelGroup.children.length) box.setFromObject(modelGroup);
  const prop = ed.prop * 0.0254 / 2;
  cad.motors.forEach(p => { const w = cadToWorld(p); box.expandByPoint(w.clone().addScalar(prop)); box.expandByPoint(w.clone().addScalar(-prop)); });
  const sphere = box.getBoundingSphere(new THREE.Sphere());
  return { box, centre: sphere.center, radius: Math.max(sphere.radius, 0.05) };
}

function frame(kind = 'iso') {
  const { centre, radius: r } = sceneSize();
  const at = { iso: [1.3, 1.0, 1.75], top: [0, 3.0, 0.001], front: [0, 0.4, -2.9], side: [2.9, 0.4, 0] }[kind];
  camera.position.copy(centre).add(new THREE.Vector3(...at).multiplyScalar(r));
  camera.near = r / 100; camera.far = r * 100; camera.updateProjectionMatrix();
  controls.target.copy(centre); controls.update();
}

function applyModelMatrix() {
  const A = mm(mm(VIEW, basis()), [[UNIT[ed.units], 0, 0], [0, UNIT[ed.units], 0], [0, 0, UNIT[ed.units]]]);
  const shift = mul(VIEW, ed.shift);
  modelGroup.matrix.set(A[0][0], A[0][1], A[0][2], shift[0], A[1][0], A[1][1], A[1][2], shift[1], A[2][0], A[2][1], A[2][2], shift[2], 0, 0, 0, 1);
  modelGroup.matrixWorldNeedsUpdate = true;
}

function showModel() {
  modelGroup.clear();
  if (parts) for (const part of parts) {
    const color = ed.tint ? new THREE.Color(ed.tintColor) : part.color;
    modelGroup.add(new THREE.Mesh(part.geometry, new THREE.MeshStandardMaterial({ color, roughness: 0.62, metalness: 0.15, side: THREE.DoubleSide })));
  }
  applyModelMatrix();
  $('drop').classList.toggle('hidden', !!parts);
  const tris = parts ? parts.reduce((n, p) => n + p.geometry.index.count / 3, 0) : 0;
  $('model-sum').textContent = parts ? `${ed.source || 'Body'} · ${Math.round(tris).toLocaleString()} triangles` : 'No model';
}

// Markers: motors with prop discs, CG, the camera's view cone and a "front" arrow.
const labels = [];
function tag(text, cls, at) { const el = document.createElement('div'); el.className = `tag ${cls}`; el.textContent = text; $('labels').append(el); labels.push({ el, at }); }
function line(points, color, opacity = 1) {
  const g = new THREE.BufferGeometry().setFromPoints(points);
  return new THREE.Line(g, new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity, depthTest: false }));
}
function drawMarkers() {
  markers.clear(); $('labels').innerHTML = ''; labels.length = 0;
  const { box, radius, centre } = sceneSize();
  const s = Math.max(radius * 0.018, 0.002);
  hover.scale.setScalar(s * 0.6);
  const dot = (p, color, size = s) => { const m = new THREE.Mesh(new THREE.SphereGeometry(size, 20, 14), new THREE.MeshBasicMaterial({ color, depthTest: false })); m.position.copy(p); m.renderOrder = 5; markers.add(m); return m; };
  const motorColor = 0x38bdf8, prop = ed.prop * 0.0254 / 2;
  cad.motors.forEach((p, i) => {
    const w = cadToWorld(p);
    dot(w, motorColor);
    const disc = new THREE.Mesh(new THREE.CircleGeometry(prop, 48), new THREE.MeshBasicMaterial({ color: motorColor, transparent: true, opacity: 0.08, side: THREE.DoubleSide, depthWrite: false }));
    disc.rotation.x = -Math.PI / 2; disc.position.copy(w); markers.add(disc);
    const ring = new THREE.LineLoop(new THREE.EdgesGeometry(new THREE.CircleGeometry(prop, 48)), new THREE.LineBasicMaterial({ color: motorColor, transparent: true, opacity: 0.45 }));
    ring.rotation.x = -Math.PI / 2; ring.position.copy(w); markers.add(ring);
    tag(MOTORS[i], 'motor', w);
  });
  const cg = cadToWorld(cad.cg);
  dot(cg, 0xfb7a1c, s * 1.25); tag('CG', 'cg', cg);
  const down = cg.clone(); down.y = Math.min(box.min.y, cg.y - radius * 0.3);
  markers.add(line([cg, down], 0xfb7a1c, 0.5));
  if ($('use-cam').checked) {
    const c = cadToWorld(cad.camera), t = THREE.MathUtils.degToRad(+$('tilt-n').value || 0);
    const dir = new THREE.Vector3(0, Math.sin(t), -Math.cos(t)), right = new THREE.Vector3(1, 0, 0), up = new THREE.Vector3().crossVectors(right, dir).negate();
    const L = radius * 0.6, w = L * Math.tan(THREE.MathUtils.degToRad(55)), h = w * 9 / 16;
    const corners = [[-1, -1], [1, -1], [1, 1], [-1, 1]].map(([a, b]) => c.clone().add(dir.clone().multiplyScalar(L)).add(right.clone().multiplyScalar(a * w)).add(up.clone().multiplyScalar(b * h)));
    markers.add(line([...corners, corners[0]], 0xc084fc, 0.8));
    corners.forEach(k => markers.add(line([c, k], 0xc084fc, 0.5)));
    dot(c, 0xc084fc); tag('CAM', 'cam', c);
  }
  // Front arrow, from the motor centre forward along the ground.
  const mid = cad.motors.reduce((a, p) => a.add(cadToWorld(p)), new THREE.Vector3()).multiplyScalar(0.25);
  const base = new THREE.Vector3(mid.x, box.min.y, mid.z);
  const arrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), base, radius * 1.1, 0x4ade80, radius * 0.12, radius * 0.06);
  markers.add(arrow); tag('FRONT', 'fwd', base.clone().add(new THREE.Vector3(0, 0, -radius * 1.1)));
  grid.scale.setScalar(Math.max(radius * 3, 0.1)); grid.position.set(mid.x, box.min.y - radius * 0.002, mid.z);
  // CFD runs: the airflow each one hits the drone with, so a wrong axis shows up at a glance.
  const cfd = ed.drag.mode === 'cfd';
  $('legend-air').style.display = cfd && ed.drag.runs.length ? '' : 'none';
  if (cfd) ed.drag.runs.forEach((run, i) => {
    const r = runResult(run, ed.drag.rho, dirToProfile);
    if (r.problem) return;
    const travel = new THREE.Vector3(...mul(VIEW, r.t)), from = centre.clone().addScaledVector(travel, radius * 1.75);
    markers.add(new THREE.ArrowHelper(travel.clone().negate(), from, radius * 0.95, 0xf472b6, radius * 0.14, radius * 0.07));
    tag(String(i + 1), 'air', from);
  });
}

const projected = new THREE.Vector3();
function loop() {
  controls.update();
  renderer.render(scene, camera);
  const r = canvas.getBoundingClientRect();
  for (const { el, at } of labels) {
    projected.copy(at).project(camera);
    const hidden = projected.z > 1;
    el.style.display = hidden ? 'none' : '';
    el.style.left = `${(projected.x + 1) / 2 * r.width}px`; el.style.top = `${(1 - projected.y) / 2 * r.height}px`;
  }
  requestAnimationFrame(loop);
}

// ---------------------------------------------------------------- picking

const ray = new THREE.Raycaster(), pointer = new THREE.Vector2();
function hit(e) {
  const r = canvas.getBoundingClientRect();
  pointer.set((e.clientX - r.left) / r.width * 2 - 1, -(e.clientY - r.top) / r.height * 2 + 1);
  ray.setFromCamera(pointer, camera);
  return ray.intersectObjects(modelGroup.children, false)[0] || null;
}
function pickLabel(target) {
  if (target === 'cg') return 'the centre of gravity';
  if (target === 'camera') return 'the front of the FPV lens';
  return `${MOTOR_NAMES[+target[1]]} motor${$('rim').checked ? '' : ' (top of the bell)'}`;
}
function startPick(target) {
  if (!parts) { toast('Load a CAD model first, or type the coordinates.', 'bad'); return; }
  pick = pick && pick.target === target ? null : { target, clicks: [] };
  updatePickUI();
}
function updatePickUI() {
  view.classList.toggle('picking', !!pick);
  document.querySelectorAll('[data-pick]').forEach(b => b.classList.toggle('on', !!pick && b.dataset.pick === pick.target));
  clickDots.clear();
  if (!pick) { $('hint').hidden = true; hover.visible = false; return; }
  const rimMode = pick.target[0] === 'm' && $('rim').checked;
  $('hint').hidden = false;
  $('hint').textContent = rimMode ? `Click point ${pick.clicks.length + 1} of 3 on the ${pickLabel(pick.target)} rim · Esc cancels` : `Click ${pickLabel(pick.target)} · Esc cancels`;
  for (const p of pick.clicks) { const d = hover.clone(); d.material = hover.material.clone(); d.material.color.set(0xfb7a1c); d.position.copy(cadToWorld(p)); d.visible = true; clickDots.add(d); }
}
let downAt = null;
canvas.addEventListener('pointerdown', e => { downAt = [e.clientX, e.clientY]; });
canvas.addEventListener('pointermove', e => {
  if (!pick) return;
  const h = hit(e); hover.visible = !!h; if (h) hover.position.copy(h.point);
});
canvas.addEventListener('pointerup', e => {
  if (!pick || !downAt || Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]) > 4) return;
  const h = hit(e); if (!h) return;
  const p = worldToCad(h.point);
  const rimMode = pick.target[0] === 'm' && $('rim').checked;
  if (rimMode) {
    pick.clicks.push(p);
    if (pick.clicks.length < 3) { updatePickUI(); return; }
    const c = circleCentre(...pick.clicks);
    if (!c) { toast('Those three points are in a line. Try again around the rim.', 'bad'); pick.clicks = []; updatePickUI(); return; }
    setPoint(pick.target, c);
  } else setPoint(pick.target, p);
  pick = null; updatePickUI();
});
addEventListener('keydown', e => { if (e.key === 'Escape' && pick) { pick = null; updatePickUI(); } });
$('rim').addEventListener('change', () => { if (pick) { pick.clicks = []; updatePickUI(); } });

function setPoint(target, p, typing = false) {
  if (target === 'cg') cad.cg = p; else if (target === 'camera') cad.camera = p; else cad.motors[+target[1]] = p;
  changed(!typing);
}

// ---------------------------------------------------------------- model files

async function loadFile(file) {
  try {
    const ext = file.name.split('.').pop().toLowerCase();
    const buffer = await file.arrayBuffer();
    let root;
    if (ext === 'stl') root = new THREE.Mesh(new STLLoader().parse(buffer), new THREE.MeshStandardMaterial({ color: 0x9aa3ad }));
    else if (ext === 'obj') root = new OBJLoader().parse(new TextDecoder().decode(buffer));
    else if (ext === 'glb' || ext === 'gltf') root = (await new GLTFLoader().parseAsync(buffer, '')).scene;
    else throw new Error('Use an STL, OBJ, GLB or glTF file.');
    const loaded = collectParts(root);
    const size = new THREE.Box3(); loaded.forEach(p => { p.geometry.computeBoundingBox(); size.union(p.geometry.boundingBox); });
    const span = Math.max(...size.getSize(new THREE.Vector3()).toArray());
    // glTF is meters and Y-up by definition; mesh files are usually millimetres from CAD.
    if (ext === 'glb' || ext === 'gltf') { ed.units = 'm'; if (ed.up[1] !== 'Y') { ed.up = '+Y'; ed.forward = '+Z'; } }
    else if (span > 20 && ed.units === 'm') ed.units = 'mm';
    else if (span < 2 && ed.units === 'mm') ed.units = 'm';
    parts = loaded; parts.id = ++partsSeq; baked = null; ed.source = file.name;
    syncImportControls(); showModel(); changed(); frame('iso');
    toast(`Loaded ${file.name}: ${(span).toFixed(span < 2 ? 3 : 0)} ${ed.units} across. Check the units and axes, then pick the motors.`, 'good');
  } catch (e) { toast(`Couldn't load ${file.name}: ${e.message}`, 'bad'); }
}

// Flatten a loaded scene into one indexed geometry per colour, in the file's own coordinates.
function collectParts(root) {
  root.updateMatrixWorld(true);
  const byColour = new Map();
  root.traverse(o => {
    if (!o.isMesh) return;
    const g = o.geometry.clone();
    for (const name of Object.keys(g.attributes)) if (name !== 'position' && name !== 'normal') g.deleteAttribute(name);
    g.morphAttributes = {};
    g.applyMatrix4(o.matrixWorld);
    if (!g.attributes.normal) g.computeVertexNormals();
    if (!g.index) g.setIndex([...Array(g.attributes.position.count).keys()]);
    const m = Array.isArray(o.material) ? o.material[0] : o.material;
    const colour = m && m.color ? m.color.getHex() : 0x9aa3ad;
    if (!byColour.has(colour)) byColour.set(colour, []);
    byColour.get(colour).push(g);
  });
  const out = [];
  let vertices = 0;
  for (const [colour, list] of byColour) {
    const geometry = list.length === 1 ? list[0] : mergeGeometries(list, false);
    vertices += geometry.attributes.position.count;
    out.push({ geometry, color: new THREE.Color(colour) });
  }
  if (!out.length) throw new Error('No meshes in that file.');
  if (vertices > MAX_VERTICES) throw new Error(`${vertices.toLocaleString()} vertices is too detailed; export it coarser (under 3 million).`);
  return out;
}

// GLMB: what the plugin reads. Positions in profile axes and meters, already shifted.
function exportBody() {
  const B = basis(), s = UNIT[ed.units];
  let size = 12;
  for (const p of parts) size += 20 + p.geometry.attributes.position.count * 24 + p.geometry.index.count * 4;
  const buf = new ArrayBuffer(size), v = new DataView(buf);
  let o = 0;
  const u32 = x => { v.setUint32(o, x, true); o += 4; }, f32 = x => { v.setFloat32(o, x, true); o += 4; };
  u32(0x424d4c47); u32(1); u32(parts.length);
  for (const p of parts) {
    const c = (ed.tint ? new THREE.Color(ed.tintColor) : p.color).clone().convertLinearToSRGB();
    f32(c.r); f32(c.g); f32(c.b);
    const pos = p.geometry.attributes.position, nrm = p.geometry.attributes.normal, idx = p.geometry.index;
    u32(pos.count); u32(idx.count);
    for (let i = 0; i < pos.count; i++) { const q = add(scale(mul(B, [pos.getX(i), pos.getY(i), pos.getZ(i)]), s), ed.shift); f32(q[0]); f32(q[1]); f32(q[2]); }
    for (let i = 0; i < pos.count; i++) { const n = mul(B, [nrm.getX(i), nrm.getY(i), nrm.getZ(i)]); f32(n[0]); f32(n[1]); f32(n[2]); }
    for (let i = 0; i < idx.count; i++) u32(idx.getX(i));
  }
  return buf;
}

function importBody(buf) {
  const v = new DataView(buf); let o = 0;
  const u32 = () => { const x = v.getUint32(o, true); o += 4; return x; }, f32 = () => { const x = v.getFloat32(o, true); o += 4; return x; };
  if (u32() !== 0x424d4c47 || u32() !== 1) throw new Error('Not a Geometry Lab body file.');
  const out = [], T = tr(basis()), s = UNIT[ed.units];
  for (let n = u32(); n > 0; n--) {
    const color = new THREE.Color().setRGB(f32(), f32(), f32(), THREE.SRGBColorSpace);
    const count = u32(), indices = u32();
    const pos = new Float32Array(count * 3), nrm = new Float32Array(count * 3), idx = new Uint32Array(indices);
    for (let i = 0; i < count; i++) { const p = scale(mul(T, sub([f32(), f32(), f32()], ed.shift)), 1 / s); pos.set(p, i * 3); }
    for (let i = 0; i < count; i++) { const q = mul(T, [f32(), f32(), f32()]); nrm.set(q, i * 3); }
    for (let i = 0; i < indices; i++) idx[i] = u32();
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3)); g.setAttribute('normal', new THREE.BufferAttribute(nrm, 3)); g.setIndex(new THREE.BufferAttribute(idx, 1));
    out.push({ geometry: g, color });
  }
  return out;
}

// ---------------------------------------------------------------- panels

const fmt = x => String(+(+x).toFixed(DECIMALS[ed.units]));
function pointRow(who, cls, target, get) {
  const row = document.createElement('div'); row.className = 'pt';
  row.innerHTML = `<span class="who ${cls}">${who}</span>` + [0, 1, 2].map(k => `<input type="number" step="any" data-k="${k}" aria-label="${who} ${'XYZ'[k]}">`).join('') + `<button data-pick="${target}">Pick</button>`;
  row.querySelectorAll('input').forEach(inp => {
    inp.value = fmt(get()[+inp.dataset.k]);
    inp.addEventListener('input', () => { const p = [...get()]; const x = parseFloat(inp.value); if (Number.isFinite(x)) { p[+inp.dataset.k] = x; setPoint(target, p, true); } });
  });
  row.querySelector('button').addEventListener('click', () => startPick(target));
  return row;
}
function head() { const h = document.createElement('div'); h.className = 'pts-head'; h.innerHTML = `<span></span><span>X ${ed.units}</span><span>Y ${ed.units}</span><span>Z ${ed.units}</span><span></span>`; return h; }
function buildPoints() {
  $('motors').replaceChildren(head(), ...cad.motors.map((_, i) => pointRow(MOTORS[i], 'motor', `m${i}`, () => cad.motors[i])));
  $('cg').replaceChildren(head(), pointRow('CG', 'cg', 'cg', () => cad.cg));
  $('camera').replaceChildren(head(), pointRow('Lens', 'cam', 'camera', () => cad.camera));
  updatePickUI();
}

const TENSOR = [[0, 0], [0, 1], [0, 2], [1, 0], [1, 1], [1, 2], [2, 0], [2, 1], [2, 2]];
function buildTensor() {
  const names = ['Ixx', 'Ixy', 'Ixz', 'Iyx', 'Iyy', 'Iyz', 'Izx', 'Izy', 'Izz'];
  $('tensor').replaceChildren(...TENSOR.map(([i, j], k) => {
    const inp = document.createElement('input'); inp.type = 'number'; inp.step = 'any'; inp.title = names[k]; inp.placeholder = names[k];
    inp.value = +cad.inertia[i][j].toPrecision(6);
    if (j < i) { inp.className = 'mirror'; inp.readOnly = true; inp.tabIndex = -1; inp.title += ' (mirrors the upper triangle)'; }
    else inp.addEventListener('input', () => { const x = parseFloat(inp.value); if (Number.isFinite(x)) { cad.inertia[i][j] = cad.inertia[j][i] = x; refreshMirror(); changed(false); } });
    return inp;
  }));
}
function refreshMirror() { [...$('tensor').children].forEach((inp, k) => { const [i, j] = TENSOR[k]; if (j < i) inp.value = +cad.inertia[i][j].toPrecision(6); }); }

function buildRows() {
  $('rows').replaceChildren(...rows.map((r, i) => {
    const tr = document.createElement('tr');
    tr.innerHTML = `<td><input type="number" step="any" min="0" max="100" value="${+r.pct.toFixed(2)}"></td><td><input type="number" step="any" min="0" value="${+r.g.toFixed(1)}"></td><td><button title="Remove">×</button></td>`;
    const [a, b] = tr.querySelectorAll('input');
    a.addEventListener('input', () => { r.pct = parseFloat(a.value); drawChart(); changed(false); });
    b.addEventListener('input', () => { r.g = parseFloat(b.value); drawChart(); changed(false); });
    tr.querySelector('button').addEventListener('click', () => { rows.splice(i, 1); buildRows(); drawChart(); changed(false); });
    return tr;
  }));
}

function curveProblem() {
  if (rows.length < 2) return 'Add at least two points, or import a thrust stand CSV.';
  for (let i = 0; i < rows.length; i++) {
    const r = rows[i];
    if (!Number.isFinite(r.pct) || !Number.isFinite(r.g)) return 'Every point needs a throttle and a thrust.';
    if (r.pct < 0 || r.pct > 100) return 'Throttle must be 0–100%.';
    if (i && r.pct <= rows[i - 1].pct) return 'Throttle points must increase down the table.';
    if (i && r.g < rows[i - 1].g) return `Thrust drops at ${r.pct}%. It must not fall as throttle rises.`;
  }
  if (rows[rows.length - 1].pct < 99.9) return `The curve stops at ${rows[rows.length - 1].pct}%. Measure 100% or use Extend to 100%.`;
  return null;
}

function drawChart() {
  const c = $('chart'), ctx = c.getContext('2d'), dpr = devicePixelRatio || 1;
  const W = c.clientWidth, H = c.clientHeight; c.width = W * dpr; c.height = H * dpr; ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, W, H);
  const problem = curveProblem(), massG = +$('mass').value || 0, hover = massG / 4;
  const ok = rows.filter(r => Number.isFinite(r.pct) && Number.isFinite(r.g));
  const top = Math.max(hover * 1.2, ...ok.map(r => r.g), 100) * 1.08;
  const L = 38, R = 10, T = 10, Bm = 24, x = p => L + p / 100 * (W - L - R), y = g => H - Bm - g / top * (H - T - Bm);
  ctx.font = '10px ui-sans-serif, system-ui'; ctx.fillStyle = '#8b95a3'; ctx.strokeStyle = '#1f252d'; ctx.lineWidth = 1;
  const raw = top / 5, mag = Math.pow(10, Math.floor(Math.log10(raw))), step = [1, 2, 5, 10].find(k => k * mag >= raw) * mag;
  for (let g = 0; g <= top; g += step) { ctx.beginPath(); ctx.moveTo(L, y(g)); ctx.lineTo(W - R, y(g)); ctx.stroke(); ctx.fillText(g >= 1000 ? `${g / 1000}k` : g, 4, y(g) + 3); }
  ctx.textAlign = 'center';
  for (let p = 0; p <= 100; p += 25) { ctx.textAlign = p === 0 ? 'left' : p === 100 ? 'right' : 'center'; ctx.fillText(`${p}%`, x(p), H - 8); }
  ctx.textAlign = 'left';
  if (hover > 0) {
    ctx.setLineDash([4, 4]); ctx.strokeStyle = '#fb7a1c'; ctx.beginPath(); ctx.moveTo(L, y(hover)); ctx.lineTo(W - R, y(hover)); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = '#fb7a1c'; ctx.fillText('hover', W - R - 30, y(hover) - 4);
  }
  let stats = [['—', 'max per motor'], ['—', 'thrust : weight'], ['—', 'hover throttle']];
  if (!problem) {
    const f = pchip(rows.map(r => r.pct), rows.map(r => r.g));
    ctx.strokeStyle = '#38bdf8'; ctx.lineWidth = 2; ctx.beginPath();
    for (let p = 0; p <= 100; p += 0.5) { const yy = y(f(p)); p ? ctx.lineTo(x(p), yy) : ctx.moveTo(x(p), yy); }
    ctx.stroke();
    const tw = massG ? 4 * f.max / massG : 0, h = hover < f.max ? f.inverse(hover) : null;
    if (h != null) { ctx.fillStyle = '#fb7a1c'; ctx.beginPath(); ctx.arc(x(h), y(hover), 4, 0, 7); ctx.fill(); }
    stats = [[`${Math.round(f.max)} g`, 'max per motor'], [tw ? `${tw.toFixed(1)} : 1` : '—', 'thrust : weight'], [h != null ? `${h.toFixed(0)}%` : 'can\'t hover', 'hover throttle']];
  }
  ctx.fillStyle = '#e6e9ee';
  for (const r of ok) { ctx.beginPath(); ctx.arc(x(r.pct), y(r.g), 2.6, 0, 7); ctx.fill(); }
  $('thrust-stats').innerHTML = stats.map(([v, l]) => `<div class="stat"><b>${v}</b><span>${l}</span></div>`).join('');
  const use = $('use-curve').checked;
  $('thrust-err').hidden = !(use && problem); $('thrust-err').textContent = problem || '';
  $('thrust-sum').textContent = use ? (problem ? 'Needs fixing' : stats[0][0] + ' · hover ' + stats[2][0]) : 'Liftoff propulsion';
  $('thrust-sum').className = use && problem ? 'bad' : '';
}

// Thrust-stand CSV: a sweep (one row per step) or a raw log (binned per 1% throttle, median).
function importCsv(text, name) {
  const lines = text.trim().split(/\r?\n/), headers = lines[0].split(',').map(h => h.trim().toLowerCase());
  const col = n => headers.indexOf(n);
  const pc = col('throttle_pct'), us = col('throttle_us'), g = col('thrust_g'), n = col('thrust_n');
  if ((pc < 0 && us < 0) || (g < 0 && n < 0)) throw new Error('Needs throttle_pct (or throttle_us) and thrust_g (or thrust_n) columns.');
  const data = lines.slice(1).map(l => l.split(',')).map(c => ({
    pct: pc >= 0 ? parseFloat(c[pc]) : (parseFloat(c[us]) - 1000) / 10,
    g: g >= 0 ? parseFloat(c[g]) : parseFloat(c[n]) / G * 1000,
  })).filter(r => Number.isFinite(r.pct) && Number.isFinite(r.g));
  const bins = new Map();
  for (const r of data) { const k = data.length > 80 ? Math.round(r.pct) : r.pct; if (!bins.has(k)) bins.set(k, []); bins.get(k).push(r.g); }
  const median = a => { const s = [...a].sort((x, y) => x - y); return s[s.length >> 1]; };
  let out = [...bins].filter(([, v]) => data.length <= 80 || v.length >= 3).map(([k, v]) => ({ pct: Math.max(0, Math.min(100, k)), g: Math.max(0, median(v)) })).sort((a, b) => a.pct - b.pct);
  if (out.length < 2) throw new Error('Not enough throttle steps in that file.');
  let fixed = 0;
  for (let i = 1; i < out.length; i++) if (out[i].g < out[i - 1].g) { out[i].g = out[i - 1].g; fixed++; }
  rows = out; $('use-curve').checked = true;
  buildRows(); syncSections(); drawChart(); changed(false);
  const end = out[out.length - 1].pct;
  toast(`Imported ${out.length} points from ${name}` + (fixed ? `, flattened ${fixed} small dips` : '') + (end < 99.9 ? `. It stops at ${end}%: measure 100% or use Extend to 100%.` : '.'), end < 99.9 ? 'bad' : 'good');
}

function syncImportControls() {
  const opts = Object.keys(AXES).map(a => `<option>${a}</option>`).join('');
  $('up').innerHTML = opts; $('up').value = ed.up;
  $('forward').innerHTML = Object.keys(AXES).filter(a => a[1] !== ed.up[1]).map(a => `<option>${a}</option>`).join(''); $('forward').value = ed.forward;
  $('units').value = ed.units; $('prop').value = ed.prop; $('iunit').value = ed.iunit;
  $('tint').checked = ed.tint; $('tint-color').value = ed.tintColor;
  const left = Object.entries(AXES).find(([, v]) => v.join() === cross(AXES[ed.up], AXES[ed.forward]).join())[0];
  $('axis-note').textContent = `Front is ${ed.forward}, up is ${ed.up}, so left is ${left}. The green arrow on the ground points forward.`;
}

function syncSections() {
  $('thrust-body').style.display = $('use-curve').checked ? '' : 'none';
  $('cam-body').style.display = $('use-cam').checked ? '' : 'none';
  $('cam-sum').textContent = $('use-cam').checked ? `${(+$('tilt-n').value).toFixed(1)}° uptilt` : 'Liftoff camera';
  $('mass-sum').textContent = `${(+$('mass').value || 0).toFixed(0)} g`;
}

// ---------------------------------------------------------------- drag

const dirToProfile = v => mul(basis(), v);
const cm2 = m2 => `${Math.round(m2 * 1e4)}`;
const sil = { key: null, areas: null, pending: null };
const silKey = () => parts ? `${parts.id}|${ed.units}|${ed.up}|${ed.forward}` : null;

// Silhouette areas of the CAD model from every table direction, measured once per model and import setting.
async function measure() {
  const key = silKey();
  if (!key || sil.pending === key || sil.key === key) return;
  sil.pending = key;
  $('drag-sum').textContent = 'Measuring…';
  const k = UNIT[ed.units], A = basis().map(r => r.map(v => v * k));
  const M = new THREE.Matrix4().set(A[0][0], A[0][1], A[0][2], 0, A[1][0], A[1][1], A[1][2], 0, A[2][0], A[2][1], A[2][2], 0, 0, 0, 0, 1);
  try {
    const areas = await silhouettes(renderer, parts, M, DIRS);
    if (silKey() === key) { sil.key = key; sil.areas = areas; }
  } catch (e) { toast(`Couldn't measure the model for drag: ${e.message}`, 'bad'); }
  finally { if (sil.pending === key) sil.pending = null; refreshDrag(); }
}

// The drag this profile would fly: at(t) gives Cd·A (m²) for a direction of travel t in profile axes.
function dragModel() {
  const d = ed.drag, A = sil.key && sil.key === silKey() ? sil.areas : null;
  if (d.mode === 'liftoff') return {};
  if (d.mode === 'cad') {
    if (!parts) return { problem: 'Load the CAD model so its silhouette can be measured, or use CFD runs.' };
    if (!A) return { problem: 'Measuring the model…', waiting: true };
    if (!(d.cd >= 0.1 && d.cd <= 3)) return { problem: 'Cd must be 0.1–3.' };
    const cda = A.map(a => a * d.cd);
    return { at: t => blend(DIRS, cda, t), cda };
  }
  const results = d.runs.map(r => runResult(r, d.rho, dirToProfile));
  if (!(d.rho >= 0.5 && d.rho <= 1.5)) return { problem: 'Air density must be 0.5–1.5 kg/m³.', results };
  const bad = results.findIndex(r => r.problem);
  if (bad >= 0) return { problem: `Run ${bad + 1} (${d.runs[bad].name || 'unnamed'}): ${results[bad].problem}.`, results };
  if (!results.length) return { problem: 'Add a CFD run, or import force.dat files.', results };
  const points = results.flatMap(r => {
    const out = [{ t: r.t, cda: r.cda }];
    if (d.mirror && Math.abs(r.t[0]) > 0.02) out.push({ t: [-r.t[0], r.t[1], r.t[2]], cda: r.cda, mirror: true });
    return out;
  });
  let at;
  if (parts && !A) return { problem: 'Measuring the model…', waiting: true, results, points };
  if (A) {
    // Shape from the silhouette, size from the runs: blend each run's effective Cd over the sphere.
    const area = t => blend(DIRS, A, t), cds = points.map(p => ({ t: p.t, v: p.cda / area(p.t) }));
    const prior = median(cds.map(c => c.v));
    at = t => shepard(cds, prior, t) * area(t);
  } else {
    const samples = points.map(p => ({ t: p.t, v: p.cda })), prior = median(samples.map(c => c.v));
    at = t => shepard(samples, prior, t);
  }
  return { at, cda: DIRS.map(at), results, points };
}

let stockSig = '', stockTable = null;
function stockDrag() {
  if (!last || !last.stockDragArea || !last.stockDragArea.length) return null;
  const sig = last.stockDragArea.join();
  if (!stockTable || stockTable.sig !== sig) {
    const f = last.stockDragDirections, dirs = [];
    for (let i = 0; i < f.length; i += 3) dirs.push([f[i], f[i + 1], f[i + 2]]);
    stockTable = { sig, at: t => blend(dirs, last.stockDragArea, t) };
  }
  return stockTable.at;
}

function refreshDrag() {
  const d = ed.drag;
  document.querySelectorAll('#drag-mode button').forEach(b => { b.classList.toggle('on', b.dataset.mode === d.mode); b.setAttribute('aria-checked', b.dataset.mode === d.mode); });
  $('drag-cad').style.display = d.mode === 'cad' ? '' : 'none';
  $('drag-cfd').style.display = d.mode === 'cfd' ? '' : 'none';
  if (d.mode !== 'liftoff' && parts && sil.key !== silKey()) measure();
  const m = dragModel(), stock = stockDrag();
  drawPolar($('drag-chart'), [m.at && { at: m.at, colour: '#fb7a1c' }, stock && { at: stock, colour: '#8b95a3', dash: true }].filter(Boolean), d.mode === 'cfd' ? m.points || [] : []);
  $('key-runs').style.display = d.mode === 'cfd' ? '' : 'none';
  const dirs = [[[0, -1, 0], 'forward'], [[0, 0, 1], 'climbing'], [[1, 0, 0], 'sideways']];
  const mine = m.at || (d.mode === 'liftoff' ? stock : null);
  $('drag-stats').innerHTML = dirs.map(([t, label]) => `<div class="stat"><b>${mine ? cm2(mine(t)) + ' cm²' : '—'}</b><span>${label}${stock && d.mode !== 'liftoff' ? ` · Liftoff ${cm2(stock(t))}` : ''}</span></div>`).join('');
  const cd = last && last.stockDragCd ? last.stockDragCd.toFixed(2) : null;
  $('drag-note').textContent = {
    liftoff: stock ? `Liftoff's own drag: the frame's Cd (${cd}) times the silhouette it measures of the stock drone, props included. Cd·A against the direction the drone is moving.` : 'Liftoff\'s own drag. Open Free Flight to see it for the current drone.',
    cad: 'Cd·A against the direction the drone is moving through the air.',
    cfd: parts ? 'Between runs, the drag follows the model\'s silhouette, scaled to match your runs.' : 'No model loaded, so the drag is blended between runs. Load the model to follow its shape between runs.',
  }[d.mode];
  const err = m.problem && !m.waiting;
  $('drag-err').hidden = !err; $('drag-err').textContent = err ? m.problem : '';
  $('drag-sum').textContent = d.mode === 'liftoff' ? 'Liftoff' : m.waiting ? 'Measuring…' : m.problem ? 'Needs fixing' : `${cm2(m.at([0, -1, 0]))} cm² forward`;
  $('drag-sum').className = err ? 'bad' : '';
  [...$('runs').children].forEach((el, i) => {
    const r = m.results ? m.results[i] : runResult(d.runs[i], d.rho, dirToProfile);
    el.classList.toggle('bad', !!r.problem);
    el.querySelector('.run-out').innerHTML = r.problem ? r.problem : `Cd·A <b>${cm2(r.cda)} cm²</b> · air ${describe(r.t)} · ${r.v.toFixed(1)} m/s` + (r.rest > 0.15 * r.cda ? ` · side/lift ${cm2(r.rest)} cm²` : '');
  });
}

function buildRuns() {
  $('runs').replaceChildren(...ed.drag.runs.map((run, i) => {
    const el = document.createElement('div'); el.className = 'run';
    const vec = (v, label) => `<div class="run-vec"><span>${label}</span>${[0, 1, 2].map(k => `<input type="number" step="any" data-v="${v}" data-k="${k}" aria-label="${v.toUpperCase()}${'xyz'[k]}" placeholder="${v.toUpperCase()}${'xyz'[k]}">`).join('')}</div>`;
    el.innerHTML = `<div class="run-head"><span class="who">${i + 1}</span><input class="run-name" aria-label="Run name"><button title="Remove run">×</button></div>${vec('u', 'U m/s')}${vec('f', 'F N')}<div class="run-out"></div>`;
    const name = el.querySelector('.run-name'); name.value = run.name || '';
    name.addEventListener('input', () => { run.name = name.value; dirty = true; refreshDrag(); });
    el.querySelectorAll('[data-v]').forEach(inp => {
      const v = run[inp.dataset.v][+inp.dataset.k];
      inp.value = v === null || v === undefined || Number.isNaN(v) ? '' : +(+v).toPrecision(8);
      inp.addEventListener('input', () => { const x = parseFloat(inp.value); run[inp.dataset.v][+inp.dataset.k] = Number.isFinite(x) ? x : null; dragChanged(); });
    });
    el.querySelector('.run-head button').addEventListener('click', () => { ed.drag.runs.splice(i, 1); buildRuns(); dragChanged(); });
    return el;
  }));
  refreshDrag();
}
function dragChanged() { dirty = true; refreshDrag(); drawMarkers(); }

async function importRuns(files) {
  const added = [], notes = [];
  for (const file of files) {
    try {
      const text = await file.text(), first = text.split(/\r?\n/).find(l => l.trim()) || '';
      if (/\.csv$/i.test(file.name) || /\bux\b/i.test(first)) {
        const runs = parseRunsCsv(text); added.push(...runs); notes.push(`${file.name}: ${runs.length} runs`);
      } else {
        const r = parseForces(text), taken = new Set([...ed.drag.runs, ...added].map(x => x.name));
        let name = file.name; for (let n = 2; taken.has(name); n++) name = `${file.name} (${n})`;
        added.push({ name, u: [null, null, null], f: r.f });
        notes.push(`${name}: ${r.rows} rows` + (r.spread > 0.05 ? `, still moving ${(r.spread * 100).toFixed(0)}% at the end (converged?)` : ''));
      }
    } catch (e) { notes.push(`${file.name}: ${e.message}`); }
  }
  ed.drag.runs.push(...added); ed.drag.mode = 'cfd';
  buildRuns(); dragChanged();
  const needU = added.some(r => r.u.some(v => v === null));
  toast(`Imported ${notes.join('; ')}.` + (needU ? ' Type each run\'s U (the freestream velocity) to finish.' : ''), added.length ? (needU ? '' : 'good') : 'bad');
}

// ---------------------------------------------------------------- validation

function profileMotors() { return cad.motors.map(p => mul(basis(), p).map(v => v * UNIT[ed.units])); }
function motorProblem() {
  const q = profileMotors(), c = scale(q.reduce(add), 0.25), r = q.map(p => sub(p, c));
  for (let i = 0; i < 4; i++) for (let j = i + 1; j < 4; j++) if (norm(sub(q[i], q[j])) < 0.02) return `${MOTORS[i]} and ${MOTORS[j]} are under 20 mm apart.`;
  const bad = [];
  if (!(r[0][0] > 0 && r[2][0] > 0)) bad.push('FL and RL should be on the left');
  if (!(r[1][0] < 0 && r[3][0] < 0)) bad.push('FR and RR should be on the right');
  if (!(q[0][1] < q[2][1] && q[1][1] < q[3][1])) bad.push('FL and FR should be in front of RL and RR');
  if (bad.length) return bad.join('; ') + '. Check the forward axis, or press Auto-label.';
  if (q.some(p => norm(p) > 1.5)) return 'Motors are over a metre out. Check the units.';
  return null;
}
function validate() {
  const issues = [];
  const m = motorProblem(); if (m) issues.push(m);
  const mass = +$('mass').value; if (!(mass >= 50 && mass <= 10000)) issues.push('Mass must be 50–10000 g.');
  if ($('use-curve').checked) { const c = curveProblem(); if (c) issues.push(c); }
  if (ed.drag.mode !== 'liftoff') { const dm = dragModel(); if (dm.problem) issues.push(dm.waiting ? 'Still measuring the model for drag. Try again in a moment.' : 'Drag: ' + dm.problem); }
  const I = profileInertia(), d = [I[0][0], I[1][1], I[2][2]];
  if (d.some(x => !(x > 0))) issues.push('Inertia diagonal (Ixx, Iyy, Izz) must be positive.');
  return issues;
}
function profileInertia() { const B = basis(), k = IUNIT[ed.iunit]; return mm(mm(B, cad.inertia), tr(B)).map(r => r.map(v => v * k)); }

function changed(redraw = true) {
  dirty = true;
  const m = motorProblem();
  $('motor-err').hidden = !m; $('motor-err').textContent = m || '';
  $('motor-sum').textContent = m ? 'Check order' : (() => { const q = profileMotors(); return `${Math.round(norm(sub(q[0], q[3])) * 1000)} mm diagonal`; })();
  $('motor-sum').className = m ? 'bad' : '';
  syncSections();
  if (redraw) { buildPoints(); drawMarkers(); } else drawMarkers();
  refreshDrag();
}

// ---------------------------------------------------------------- server

async function call(method, path, body, type = 'application/json') {
  const res = await fetch(path, { method, body, headers: method === 'GET' ? {} : { 'X-Geometry-Lab': '1', 'Content-Type': type } });
  const isJson = (res.headers.get('content-type') || '').includes('json');
  const data = isJson ? await res.json() : await res.arrayBuffer();
  if (!res.ok) throw new Error(isJson && data.message ? data.message : `${res.status} ${res.statusText}`);
  return data;
}

async function load() {
  const p = await call('GET', '/api/profile');
  profile = p;
  let saved = {}; try { saved = JSON.parse(p.editor || '{}'); } catch { }
  ed = freshEd(saved);
  $('name').value = p.name;
  $('mass').value = +(p.massKg * 1000).toFixed(3);
  cad.motors = [0, 1, 2, 3].map(i => fromProfile(p.motorsM.slice(3 * i, 3 * i + 3)));
  cad.cg = fromProfile(p.centerOfMassM);
  const B = basis(), k = IUNIT[ed.iunit], I = [0, 1, 2].map(i => p.inertiaKgM2.slice(3 * i, 3 * i + 3));
  cad.inertia = mm(mm(tr(B), I), B).map(r => r.map(v => v / k));
  rows = (p.thrustThrottle || []).map((t, i) => ({ pct: t * 100, g: p.thrustN[i] / G * 1000 }));
  $('use-curve').checked = rows.length > 0;
  $('use-cam').checked = !!p.cameraEnabled;
  const front = scale(add(cad.motors[0], cad.motors[1]), 0.5);
  cad.camera = p.cameraEnabled ? fromProfile(p.cameraM) : front;
  $('tilt').value = $('tilt-n').value = p.cameraUptiltDeg ?? 25;
  $('hide-stock').checked = p.hideStockFrame !== false;
  $('cd').value = ed.drag.cd; $('rho').value = ed.drag.rho; $('mirror').checked = ed.drag.mirror;
  parts = null; baked = null;
  if (p.bodyModel) {
    try { parts = importBody(await call('GET', `/api/model?name=${encodeURIComponent(p.bodyModel)}`)); parts.id = ++partsSeq; baked = bakeKey(); }
    catch (e) { toast(`Couldn't load the saved body model: ${e.message}`, 'bad'); }
  }
  syncImportControls(); showModel(); buildPoints(); buildTensor(); buildRows(); buildRuns(); drawChart(); changed(); frame('iso');
  dirty = false;
}

function buildProfile() {
  // Put the profile origin at the motor centre so motor order reads cleanly; points keep their CAD values.
  const q = profileMotors();
  ed.shift = scale(q.reduce(add), -0.25);
  const flat = a => a.flat().map(v => +v.toPrecision(9));
  return {
    ...profile, schema: 2, name: $('name').value.trim() || 'My drone', axes: 'left_backward_up',
    massKg: +$('mass').value / 1000,
    centerOfMassM: flat(toProfile(cad.cg)),
    motorsM: flat(cad.motors.map(toProfile)),
    inertiaKgM2: flat(profileInertia()),
    thrustThrottle: $('use-curve').checked ? rows.map(r => r.pct / 100) : [],
    thrustN: $('use-curve').checked ? rows.map(r => r.g / 1000 * G) : [],
    cameraEnabled: $('use-cam').checked, cameraM: flat(toProfile(cad.camera)), cameraUptiltDeg: +$('tilt-n').value,
    bodyModel: parts ? BODY_FILE : '', hideStockFrame: $('hide-stock').checked,
    ...(() => {
      const m = ed.drag.mode === 'liftoff' ? {} : dragModel();
      return m.cda ? { dragDirections: DIRS.flat().map(v => +v.toPrecision(7)), dragAreaM2: m.cda.map(v => +v.toPrecision(6)) } : { dragDirections: [], dragAreaM2: [] };
    })(),
    editor: JSON.stringify(ed),
  };
}

// Everything the body file bakes in: if any of it changes, the file is uploaded again.
const bakeKey = () => JSON.stringify([ed.shift, ed.units, ed.up, ed.forward, ed.tint, ed.tintColor]);
async function save() {
  const issues = validate();
  if (issues.length) throw new Error(issues[0]);
  const p = buildProfile();
  if (parts && (baked !== bakeKey() || profile.bodyModel !== BODY_FILE)) {
    await call('PUT', `/api/model?name=${BODY_FILE}`, exportBody(), 'application/octet-stream');
    baked = bakeKey();
  }
  await call('PUT', '/api/profile', JSON.stringify(p));
  profile = p;
  dirty = false;
}

let busy = false;
async function run(label, work) {
  if (busy) return; busy = true;
  document.querySelectorAll('.bar button').forEach(b => b.disabled = true);
  try { const message = await work(); toast(message || label, 'good'); }
  catch (e) { toast(e.message, 'bad'); }
  finally { busy = false; poll(); }
}

let last = null;
async function poll() {
  try {
    const s = await call('GET', '/api/state');
    last = s;
    const sig = (s.stockDragArea || []).join();
    if (sig !== stockSig) { stockSig = sig; refreshDrag(); }
    const link = $('link');
    link.className = 'link ' + (!s.compatible ? 'bad' : s.live ? 'live' : s.applied ? 'applied' : '');
    link.querySelector('span').textContent = !s.compatible ? 'Disabled on this Liftoff build' : !s.bound ? 'No drone. Open Free Flight' : s.live ? `LIVE${s.throttle ? ` · throttle ${(s.throttle * 100).toFixed(0)}%` : ''}` : s.applied ? 'Applied: arm to go live' : 'Stock drone';
    $('auto').checked = s.autoApply;
    $('report').textContent = s.report || s.status;
    $('game-sum').textContent = s.game;
    if (!busy) {
      $('apply').disabled = !s.compatible || !s.bound || !s.canEdit;
      $('apply').title = !s.bound ? 'Open single-player Free Flight first' : !s.canEdit ? 'Land and stop the drone first' : 'Save and fly this profile';
      $('restore').disabled = !s.bound || !s.canEdit; $('save').disabled = false;
    }
  } catch {
    $('link').className = 'link bad'; $('link').querySelector('span').textContent = 'Liftoff not running';
    if (!busy) { $('apply').disabled = $('restore').disabled = true; }
  }
}

let toastTimer;
function toast(text, kind = '') {
  const t = $('toast'); t.textContent = text; t.className = `toast ${kind}`; t.hidden = false;
  clearTimeout(toastTimer); toastTimer = setTimeout(() => { t.hidden = true; }, kind === 'bad' ? 7000 : 3500);
}

// ---------------------------------------------------------------- wiring

for (const id of ['file', 'file2']) $(id).addEventListener('change', e => { const f = e.target.files[0]; if (f) loadFile(f); e.target.value = ''; });
view.addEventListener('dragover', e => { e.preventDefault(); $('drop').classList.add('over'); });
view.addEventListener('dragleave', e => { if (e.target === view || !view.contains(e.relatedTarget)) $('drop').classList.remove('over'); });
view.addEventListener('drop', e => { e.preventDefault(); $('drop').classList.remove('over'); const f = e.dataTransfer.files[0]; if (f) loadFile(f); });
document.querySelectorAll('[data-view]').forEach(b => b.addEventListener('click', () => frame(b.dataset.view)));
$('clear-model').addEventListener('click', () => { parts = null; baked = null; ed.source = ''; showModel(); changed(); });

function importChanged() {
  if (ed.forward[1] === ed.up[1]) ed.forward = Object.keys(AXES).find(a => a[1] !== ed.up[1]);
  syncImportControls(); showModel(); buildTensor(); changed(); frame('iso');
}
$('units').addEventListener('change', e => { ed.units = e.target.value; importChanged(); });
$('up').addEventListener('change', e => { ed.up = e.target.value; importChanged(); });
$('forward').addEventListener('change', e => { ed.forward = e.target.value; importChanged(); });
$('prop').addEventListener('input', e => { const x = parseFloat(e.target.value); if (x > 0) { ed.prop = x; drawMarkers(); dirty = true; } });
$('tint').addEventListener('change', e => { ed.tint = e.target.checked; showModel(); dirty = true; });
$('tint-color').addEventListener('input', e => { ed.tintColor = e.target.value; if (ed.tint) showModel(); dirty = true; });
$('hide-stock').addEventListener('change', () => { dirty = true; });
$('iunit').addEventListener('change', e => {
  // Same physical inertia, new unit: rescale the numbers.
  const k = IUNIT[ed.iunit] / IUNIT[e.target.value];
  cad.inertia = cad.inertia.map(r => r.map(v => v * k)); ed.iunit = e.target.value; buildTensor(); dirty = true;
});
$('sort').addEventListener('click', () => {
  const q = cad.motors.map((p, i) => ({ p, q: profileMotors()[i] })).sort((a, b) => a.q[1] - b.q[1]);
  const front = q.slice(0, 2).sort((a, b) => b.q[0] - a.q[0]), rear = q.slice(2).sort((a, b) => b.q[0] - a.q[0]);
  cad.motors = [front[0].p, front[1].p, rear[0].p, rear[1].p]; changed(); toast('Relabelled as FL, FR, RL, RR from the forward axis.', 'good');
});
$('mass').addEventListener('input', () => { drawChart(); changed(false); });
$('use-curve').addEventListener('change', () => { syncSections(); drawChart(); dirty = true; });
$('use-cam').addEventListener('change', () => { if ($('use-cam').checked && !parts) toast('Type the lens position, or load the model to pick it.'); changed(false); });
$('tilt').addEventListener('input', e => { $('tilt-n').value = e.target.value; changed(false); });
$('tilt-n').addEventListener('input', e => { $('tilt').value = e.target.value; changed(false); });
$('csv').addEventListener('change', async e => { const f = e.target.files[0]; e.target.value = ''; if (!f) return; try { importCsv(await f.text(), f.name); } catch (err) { toast(`${f.name}: ${err.message}`, 'bad'); } });
$('add-row').addEventListener('click', () => { const l = rows[rows.length - 1]; rows.push({ pct: l ? Math.min(100, l.pct + 10) : 0, g: l ? l.g : 0 }); buildRows(); drawChart(); dirty = true; });
$('extend').addEventListener('click', () => {
  if (rows.length < 2) return;
  const a = rows[rows.length - 2], b = rows[rows.length - 1];
  if (b.pct >= 99.9) { toast('The curve already reaches 100%.'); return; }
  rows.push({ pct: 100, g: b.g + (b.g - a.g) / (b.pct - a.pct) * (100 - b.pct) }); buildRows(); drawChart(); dirty = true;
  toast('Extended in a straight line from the last two points. A real 100% measurement is better.');
});
$('name').addEventListener('input', () => { dirty = true; });
$('auto').addEventListener('change', e => run('', () => call('POST', `/api/autoapply?on=${e.target.checked ? 1 : 0}`).then(r => r.message)));
$('save').addEventListener('click', () => run('Saved', async () => { await save(); return `Saved ${$('name').value}. Apply it when the drone is stopped.`; }));
$('apply').addEventListener('click', () => run('Applied', async () => { await save(); const r = await call('POST', '/api/apply'); return r.message; }));
$('restore').addEventListener('click', () => run('Restored', async () => (await call('POST', '/api/restore')).message));
addEventListener('beforeunload', e => { if (dirty) e.preventDefault(); });
addEventListener('resize', () => { drawChart(); refreshDrag(); });
document.querySelectorAll('#drag-mode button').forEach(b => b.addEventListener('click', () => { ed.drag.mode = b.dataset.mode; dragChanged(); }));
$('cd').addEventListener('input', e => { ed.drag.cd = parseFloat(e.target.value); dragChanged(); });
$('rho').addEventListener('input', e => { ed.drag.rho = parseFloat(e.target.value); dragChanged(); });
$('mirror').addEventListener('change', e => { ed.drag.mirror = e.target.checked; dragChanged(); });
$('add-run').addEventListener('click', () => { ed.drag.runs.push({ name: `run ${ed.drag.runs.length + 1}`, u: [null, null, null], f: [null, null, null] }); buildRuns(); dragChanged(); });
$('run-files').addEventListener('change', e => { const files = [...e.target.files]; e.target.value = ''; if (files.length) importRuns(files); });

resize(); loop();
load().catch(e => toast(`Couldn't read the profile from Liftoff: ${e.message}`, 'bad'));
poll(); setInterval(poll, 1000);
