// Drag for the Geometry Lab editor: directions, silhouette areas, CFD runs and the polar plot.
//
// Everything here is in profile axes (x left, y back, z up) and SI units. A drag table maps a direction of
// travel through the air (unit vector) to a drag area Cd·A in m². Liftoff applies 1/2 ρ Cd·A v² against
// the drone's velocity through the air, looking the area up by direction exactly like blend() below.
import * as THREE from 'three';

const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const norm = a => Math.hypot(a[0], a[1], a[2]);
const unit = a => { const n = norm(a); return [a[0] / n, a[1] / n, a[2] / n]; };
export const RHO = 1.184; // what Liftoff uses

// 162 directions evenly round the sphere (a 4-frequency geodesic): the table the editor sends.
export const DIRS = (() => {
  const g = new THREE.IcosahedronGeometry(1, 3), p = g.attributes.position, seen = new Map();
  for (let i = 0; i < p.count; i++) {
    const v = unit([p.getX(i), p.getY(i), p.getZ(i)]);
    seen.set(v.map(x => x.toFixed(5)).join(), v);
  }
  g.dispose();
  return [...seen.values()];
})();

// Liftoff's own lookup (and the plugin's): inverse-distance blend of the three nearest directions.
export function blend(dirs, values, t) {
  t = unit(t);
  const near = dirs.map((d, i) => [Math.hypot(d[0] - t[0], d[1] - t[1], d[2] - t[2]), i]).sort((a, b) => a[0] - b[0]).slice(0, 3);
  if (near[0][0] < 1e-3) return values[near[0][1]];
  let w = 0, s = 0;
  for (const [d, i] of near) { w += 1 / d; s += values[i] / d; }
  return s / w;
}

// Smooth interpolation over the sphere from scattered samples [{t, v}], exact at each sample; far from all
// of them it settles on `prior`. Weights are 1/angle², with the prior counting as a sample 50° away.
export function shepard(samples, prior, t) {
  t = unit(t);
  let w = 1 / (50 * Math.PI / 180) ** 2, s = w * prior;
  for (const { t: d, v } of samples) {
    const a = Math.max(Math.acos(Math.max(-1, Math.min(1, dot(d, t)))), 0.005), k = 1 / (a * a);
    w += k; s += k * v;
  }
  return s / w;
}

export const median = a => { const s = [...a].sort((x, y) => x - y), n = s.length; return n ? (n % 2 ? s[n >> 1] : (s[n / 2 - 1] + s[n / 2]) / 2) : NaN; };

// Silhouette (projected) area of the model seen along each direction, in m². Renders the model flat white
// with an orthographic camera into a small target and counts covered pixels, like Liftoff does in game.
// `matrix` maps the parts' own coordinates to profile meters.
export async function silhouettes(renderer, parts, matrix, dirs, size = 256) {
  const scene = new THREE.Scene(), material = new THREE.MeshBasicMaterial({ color: 0xffffff, side: THREE.DoubleSide });
  for (const p of parts) { const m = new THREE.Mesh(p.geometry, material); m.matrixAutoUpdate = false; m.matrix.copy(matrix); scene.add(m); }
  scene.updateMatrixWorld(true);
  const sphere = new THREE.Box3().setFromObject(scene).getBoundingSphere(new THREE.Sphere());
  const r = sphere.radius * 1.02;
  const camera = new THREE.OrthographicCamera(-r, r, r, -r, r * 0.1, r * 4);
  const target = new THREE.WebGLRenderTarget(size, size), pixels = new Uint8Array(size * size * 4);
  const before = renderer.getRenderTarget(), clear = renderer.getClearColor(new THREE.Color()), alpha = renderer.getClearAlpha();
  const out = [];
  try {
    for (let i = 0; i < dirs.length; i++) {
      const d = new THREE.Vector3(...dirs[i]);
      camera.up.set(...(Math.abs(d.z) > 0.9 ? [0, 1, 0] : [0, 0, 1]));
      camera.position.copy(sphere.center).addScaledVector(d, r * 2);
      camera.lookAt(sphere.center); camera.updateMatrixWorld(); camera.updateProjectionMatrix();
      renderer.setRenderTarget(target); renderer.setClearColor(0x000000, 0); renderer.clear(); renderer.render(scene, camera);
      renderer.readRenderTargetPixels(target, 0, 0, size, size, pixels);
      // Hand the renderer back between renders: the editor's own view keeps drawing while this runs.
      renderer.setRenderTarget(before); renderer.setClearColor(clear, alpha);
      let n = 0; for (let k = 3; k < pixels.length; k += 4) if (pixels[k]) n++;
      out.push(n * (2 * r / size) ** 2);
      if (i % 24 === 23) await new Promise(r => setTimeout(r, 0));
    }
  } finally {
    renderer.setRenderTarget(before); renderer.setClearColor(clear, alpha);
    target.dispose(); material.dispose();
  }
  return out;
}

// One CFD run → its direction of travel (profile axes) and drag area. `toProfile` turns a CAD-axis
// direction into profile axes. U is the freestream velocity; the drone travels the opposite way.
export function runResult(run, rho, toProfile) {
  const num = x => (x === null || x === undefined || x === '' ? NaN : Number(x));
  const u = run.u.map(num), f = run.f.map(num);
  if (![...u, ...f].every(Number.isFinite)) return { problem: 'needs U and F' };
  const v = norm(u);
  if (v < 0.5) return { problem: 'U is zero' };
  const drag = dot(f, u) / v, q = 0.5 * rho * v * v, cda = drag / q;
  if (!(cda > 0)) return { problem: 'force points into the wind; check the axes or signs' };
  if (cda > 1) return { problem: `Cd·A of ${cda.toFixed(2)} m² is too big; check the units` };
  const t = unit(toProfile(u.map(x => -x)));
  // What's left of the force after drag: lift and side force, kept for reference.
  const rest = norm(f.map((x, i) => x - drag * u[i] / v)) / q;
  return { t, cda, rest, v };
}

// Where the air comes from, in words, for a direction of travel t (profile axes).
export function describe(t) {
  const up = Math.asin(Math.max(-1, Math.min(1, t[2]))) * 180 / Math.PI;
  if (up > 87) return 'from straight above';
  if (up < -87) return 'from straight below';
  const az = Math.atan2(t[0], -t[1]) * 180 / Math.PI; // 0 front, +90 left
  const a = Math.round(Math.abs(az));
  const side = a <= 2 ? 'the front' : a >= 178 ? 'behind' : a === 90 ? `the ${az > 0 ? 'left' : 'right'}` : a < 90 ? `front, ${a}° ${az > 0 ? 'left' : 'right'}` : `behind, ${180 - a}° ${az > 0 ? 'left' : 'right'}`;
  const e = Math.round(Math.abs(up));
  return `from ${side}` + (e <= 2 ? ', level' : `, ${e}° ${up > 0 ? 'above' : 'below'}`);
}

// OpenFOAM forces output (postProcessing/<name>/<time>/force.dat or forces.dat). Returns the total force
// averaged over the last fifth of the run, and how much it still moves there.
export function parseForces(text) {
  const lines = text.split(/\r?\n/);
  let names = null;
  const data = [];
  for (const raw of lines) {
    const line = raw.trim();
    if (!line) continue;
    if (line.startsWith('#')) {
      const cols = line.slice(1).trim().split(/\s+/);
      if (cols.some(c => /_(x|y|z)$/i.test(c))) names = cols.map(c => c.toLowerCase());
      continue;
    }
    const nums = line.replace(/[()]/g, ' ').trim().split(/\s+/).map(Number);
    if (nums.every(Number.isFinite)) data.push({ nums, nested: line.includes('(') });
  }
  if (!data.length) throw new Error('no force rows found');
  let total;
  const col = n => names ? names.indexOf(n) : -1;
  if (col('total_x') >= 0) total = r => ['total_x', 'total_y', 'total_z'].map(n => r[col(n)]);
  else if (col('pressure_x') >= 0 && col('viscous_x') >= 0) total = r => [0, 1, 2].map(k => r[col('pressure_' + 'xyz'[k])] + r[col('viscous_' + 'xyz'[k])] + (col('porous_x') >= 0 ? r[col('porous_' + 'xyz'[k])] : 0));
  else if (data[0].nested && data[0].nums.length >= 7) total = r => [0, 1, 2].map(k => r[1 + k] + r[4 + k] + (r.length >= 10 ? r[7 + k] : 0));
  else if (data[0].nums.length >= 4) total = r => [r[1], r[2], r[3]];
  else throw new Error('unrecognised force file layout');
  const tail = data.slice(Math.floor(data.length * 0.8)).map(d => total(d.nums));
  const mean = [0, 1, 2].map(k => tail.reduce((s, f) => s + f[k], 0) / tail.length);
  const size = Math.max(norm(mean), 1e-9);
  const spread = Math.max(...tail.map(f => norm(f.map((x, k) => x - mean[k])))) / size;
  return { f: mean, spread, rows: data.length };
}

// A summary CSV, one row per run: ux, uy, uz, fx, fy, fz (any order, any case), optional name.
export function parseRunsCsv(text) {
  const lines = text.trim().split(/\r?\n/).filter(l => l.trim() && !l.trim().startsWith('#'));
  const heads = lines[0].split(/[,;\t]/).map(h => h.trim().toLowerCase().replace(/[^a-z_]/g, ''));
  const find = (...names) => heads.findIndex(h => names.includes(h));
  const cols = [['ux', 'u_x'], ['uy', 'u_y'], ['uz', 'u_z'], ['fx', 'f_x', 'total_x'], ['fy', 'f_y', 'total_y'], ['fz', 'f_z', 'total_z']].map(n => find(...n));
  if (cols.some(c => c < 0)) throw new Error('needs ux, uy, uz, fx, fy, fz columns');
  const name = find('name', 'run', 'case');
  return lines.slice(1).map(l => l.split(/[,;\t]/)).map((c, i) => ({
    name: name >= 0 ? c[name].trim() : `run ${i + 1}`,
    u: cols.slice(0, 3).map(k => parseFloat(c[k])), f: cols.slice(3).map(k => parseFloat(c[k])),
  })).filter(r => [...r.u, ...r.f].every(Number.isFinite));
}

// Two polar views of drag area against direction of travel: from the side (forward to the right, up at
// the top) and from above (forward to the right, left at the top). curves: [{at(t) → m², colour, dash}].
export function drawPolar(canvas, curves, points) {
  const ctx = canvas.getContext('2d'), dpr = devicePixelRatio || 1;
  const W = canvas.clientWidth, H = canvas.clientHeight; canvas.width = W * dpr; canvas.height = H * dpr; ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, W, H);
  const views = [
    { title: 'Side', dir: a => [0, -Math.cos(a), Math.sin(a)], labels: ['fwd', 'up', 'back', 'down'], onPlane: t => Math.abs(t[0]) < 0.35, angle: t => Math.atan2(t[2], -t[1]) },
    { title: 'Top', dir: a => [Math.sin(a), -Math.cos(a), 0], labels: ['fwd', 'left', 'back', 'right'], onPlane: t => Math.abs(t[2]) < 0.35, angle: t => Math.atan2(t[0], -t[1]) },
  ];
  const samples = curves.map(c => views.map(v => Array.from({ length: 121 }, (_, i) => c.at(v.dir(i / 120 * 2 * Math.PI)))));
  const top = Math.max(1e-4, ...samples.flat(2).filter(Number.isFinite), ...points.map(p => p.cda)) * 1.1;
  const R = Math.min(W / 4, H / 2) - 18;
  if (!(R > 10)) return; // not laid out (section closed or page hidden)
  ctx.font = '10px ui-sans-serif, system-ui';
  views.forEach((v, k) => {
    const cx = W / 4 + k * W / 2, cy = H / 2 + 4, r = x => x / top * R;
    ctx.strokeStyle = '#1f252d'; ctx.lineWidth = 1;
    const raw = top / 3, mag = Math.pow(10, Math.floor(Math.log10(raw))), step = [1, 2, 5, 10].find(s => s * mag >= raw) * mag;
    for (let g = step; g < top; g += step) { ctx.beginPath(); ctx.arc(cx, cy, r(g), 0, 7); ctx.stroke(); }
    ctx.beginPath(); ctx.moveTo(cx - R, cy); ctx.lineTo(cx + R, cy); ctx.moveTo(cx, cy - R); ctx.lineTo(cx, cy + R); ctx.stroke();
    ctx.fillStyle = '#5b6573';
    ctx.textAlign = 'left'; ctx.fillText(`${Math.round(step * 1e4)} cm²`, cx + r(step) * 0.72 + 2, cy - r(step) * 0.72 - 2);
    ctx.fillStyle = '#8b95a3';
    ctx.textAlign = 'right'; ctx.fillText(v.labels[0], cx + R + 2, cy - 4);
    ctx.textAlign = 'center'; ctx.fillText(v.labels[1], cx, cy - R - 4); ctx.fillText(v.labels[3], cx, cy + R + 11);
    ctx.textAlign = 'left'; ctx.fillText(v.labels[2], cx - R - 2, cy - 4);
    ctx.fillStyle = '#e6e9ee'; ctx.font = '600 11px ui-sans-serif, system-ui'; ctx.fillText(v.title, cx - R - 4, 12); ctx.font = '10px ui-sans-serif, system-ui';
    curves.forEach((c, j) => {
      const s = samples[j][k]; if (!s.every(Number.isFinite)) return;
      ctx.strokeStyle = c.colour; ctx.lineWidth = c.dash ? 1.2 : 2; ctx.setLineDash(c.dash ? [4, 3] : []);
      ctx.beginPath();
      s.forEach((x, i) => { const a = i / 120 * 2 * Math.PI, px = cx + r(x) * Math.cos(a), py = cy - r(x) * Math.sin(a); i ? ctx.lineTo(px, py) : ctx.moveTo(px, py); });
      ctx.closePath(); ctx.stroke(); ctx.setLineDash([]);
    });
    for (const p of points) {
      if (!v.onPlane(p.t)) continue;
      const a = v.angle(p.t);
      ctx.fillStyle = p.mirror ? '#64748b' : '#ffffff';
      ctx.beginPath(); ctx.arc(cx + r(p.cda) * Math.cos(a), cy - r(p.cda) * Math.sin(a), 3, 0, 7); ctx.fill();
    }
  });
}
