import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';
import {RoomEnvironment} from 'three/addons/environments/RoomEnvironment.js';
import {toCreasedNormals} from 'three/addons/utils/BufferGeometryUtils.js';

const BUILD = 'v3';
const $ = id => document.getElementById(id);
const el = $('viewer'), Q = new URLSearchParams(location.search);
const scene = new THREE.Scene();
const cam = new THREE.PerspectiveCamera(28, el.clientWidth / el.clientHeight, 5, 8000);
const r = new THREE.WebGLRenderer({antialias: true, alpha: true, preserveDrawingBuffer: true});
r.setPixelRatio(Math.min(devicePixelRatio, 2)); r.setSize(el.clientWidth, el.clientHeight);
r.shadowMap.enabled = true; r.shadowMap.type = THREE.PCFSoftShadowMap;
r.toneMapping = THREE.ACESFilmicToneMapping; r.toneMappingExposure = 1.05; r.outputColorSpace = THREE.SRGBColorSpace;
el.prepend(r.domElement);
const pm = new THREE.PMREMGenerator(r); scene.environment = pm.fromScene(new RoomEnvironment(), 0.04).texture; scene.environmentIntensity = 0.6;
const ctl = new OrbitControls(cam, r.domElement); ctl.enableDamping = true; ctl.minDistance = 150; ctl.maxDistance = 2500;

// robot frame: x forward, y left, z up (mm)  ->  three: (x, z, -y)
const TGT = new THREE.Vector3(-5, 62, 55);
const VIEWS = {front: [760, 130, 55], '34': [520, 300, -470], side: [-5, 120, -760], back: [-760, 160, 55], top: [-4, 900, 56], under: [300, -420, -260]};
function setView(n) {
  const v = VIEWS[n] || VIEWS['34']; cam.position.set(...v); ctl.target.copy(TGT); ctl.update();
  document.querySelectorAll('[data-view]').forEach(b => b.classList.toggle('on', b.dataset.view === n));
}

scene.add(new THREE.HemisphereLight(0xfff6ea, 0x9a8f80, 0.9));
const key = new THREE.DirectionalLight(0xffffff, 2.2); key.position.set(380, 760, -420); key.castShadow = true;
key.shadow.mapSize.set(2048, 2048); Object.assign(key.shadow.camera, {left: -360, right: 360, top: 360, bottom: -360, near: 100, far: 2200});
key.shadow.bias = -0.0004; key.shadow.normalBias = 1.0; scene.add(key);
const rim = new THREE.DirectionalLight(0xffe0bf, 1.1); rim.position.set(-600, 380, 300); scene.add(rim);
const floor = new THREE.Mesh(new THREE.CircleGeometry(1400, 96), new THREE.ShadowMaterial({opacity: 0.16}));
floor.rotation.x = -Math.PI / 2; floor.receiveShadow = true; scene.add(floor);
const root = new THREE.Group(); root.rotation.x = -Math.PI / 2; scene.add(root);

const GROUPS = {
  shell: {name: 'Shell, face & tail', color: '#e9e1d3'}, structure: {name: 'Chassis', color: '#4a4f58'},
  legs: {name: 'Legs', color: '#c9bfae'}, actuators: {name: 'Servos & horns', color: '#2d3138'},
  power: {name: 'Power & charging', color: '#3b6fb6'}, electronics: {name: 'Main board', color: '#1f6f43'},
  sensors: {name: 'Sensors', color: '#7a2fa0'}, expression: {name: 'Face & voice', color: '#ff7a59'}, dock: {name: 'Charging bed', color: '#9c9285'}
};
const items = {}, links = {}, labels = [];
const shellMats = [];
let RI, xray = Q.get('xray') === '1', showLabels = Q.get('labels') !== '0', explodeT = +(Q.get('explode') || 0), demo = Q.get('walk') === '1', sel = null;
const visible = Object.fromEntries(Object.keys(GROUPS).map(g => [g, true]));
(Q.get('hide') || '').split(',').filter(Boolean).forEach(g => visible[g] = false);

// ---- the face: two rounded eyes drawn on the 1.47" LCD, with blinks and glances
const eyeC = document.createElement('canvas'); eyeC.width = 320; eyeC.height = 172;
const eyeTex = new THREE.CanvasTexture(eyeC); eyeTex.colorSpace = THREE.SRGBColorSpace;
function drawEyes(t) {
  const g = eyeC.getContext('2d'); g.fillStyle = '#040608'; g.fillRect(0, 0, 320, 172);
  const bl = (t % 4.2) > 4.05 ? 0.12 : 1, gx = 14 * Math.sin(t * 0.6) * (Math.sin(t * 0.23) > 0.3 ? 1 : 0.2);
  for (const cx of [100, 220]) {
    const h = 92 * bl, w = 62;
    const grd = g.createLinearGradient(0, 86 - h / 2, 0, 86 + h / 2); grd.addColorStop(0, '#b8fff0'); grd.addColorStop(1, '#3fd9b8');
    g.fillStyle = grd; g.shadowColor = '#6ff5d8'; g.shadowBlur = 18;
    g.beginPath(); g.roundRect(cx - w / 2 + gx, 86 - h / 2, w, h, Math.min(28, h / 2)); g.fill();
    g.shadowBlur = 0;
    if (bl > 0.5) { g.fillStyle = 'rgba(255,255,255,.85)'; g.beginPath(); g.ellipse(cx - 12 + gx, 62, 9, 11, -0.3, 0, Math.PI * 2); g.fill(); }
  }
  g.fillStyle = 'rgba(255,140,120,.35)'; for (const cx of [62, 258]) { g.beginPath(); g.ellipse(cx, 140, 22, 9, 0, 0, Math.PI * 2); g.fill(); }
  eyeTex.needsUpdate = true;
}
drawEyes(0);

function makeMat(key, it) {
  const k = key.split(':')[1];
  const c = new THREE.Color(it.color);
  if (key.startsWith('part:')) {
    if (k === 'visor') { const m = new THREE.MeshPhysicalMaterial({color: 0x161b22, roughness: 0.08, metalness: 0, clearcoat: 1, transparent: true, opacity: 0.55, depthWrite: false}); m.userData.base = 0.55; shellMats.push(m); return m; }
    if (k === 'tail' || k.startsWith('foot')) return new THREE.MeshPhysicalMaterial({color: c, roughness: 0.6, sheen: 0.5, sheenColor: 0xffc0a0});
    if (k === 'top_shell') { const m = new THREE.MeshPhysicalMaterial({color: c, roughness: 0.35, clearcoat: 0.5, clearcoatRoughness: 0.3}); shellMats.push(m); return m; }
    if (k === 'chassis') { const m = new THREE.MeshPhysicalMaterial({color: c, roughness: 0.55, clearcoat: 0.15}); shellMats.push(m); return m; }
    if (k.startsWith('thigh')) return new THREE.MeshPhysicalMaterial({color: c, roughness: 0.4, clearcoat: 0.4});
    return new THREE.MeshStandardMaterial({color: c, roughness: 0.55});
  }
  if (k.startsWith('tailpad') || k.startsWith('bed_pin') || k.startsWith('horn')) return new THREE.MeshStandardMaterial({color: c, metalness: 0.85, roughness: 0.3});
  if (k === 'battery') return new THREE.MeshStandardMaterial({color: c, roughness: 0.4, metalness: 0.2});
  return new THREE.MeshStandardMaterial({color: c, roughness: 0.5, metalness: 0.05});
}

Promise.all([
  fetch('assets/robot_info.json?v=' + BUILD).then(x => x.json()),
  fetch('assets/bom.json?v=' + BUILD).then(x => x.json()),
  new Promise((ok, bad) => new GLTFLoader().load('assets/nibbo.gltf?v=' + BUILD, ok, undefined, bad))
]).then(([ri, bom, gltf]) => {
  RI = ri; const g = gltf.scene; root.add(g);
  const fix = s => s.replace(/^(link|part|elec)(?!:)/, '$1:');
  const meshes = [];
  g.traverse(o => { const n = fix(o.name); if (n.startsWith('link:') && !o.isMesh) links[n.slice(5)] = o; if (o.isMesh) meshes.push(o); });
  for (const o of meshes) {
    let n = fix(o.name); if (!ri.items[n]) n = fix(o.parent.name);
    const it = ri.items[n]; if (!it) continue;
    o.geometry = toCreasedNormals(o.geometry.index ? o.geometry.toNonIndexed() : o.geometry, THREE.MathUtils.degToRad(32));
    o.material = makeMat(n, it); o.castShadow = !n.includes('visor'); o.receiveShadow = true;
    const node = ri.items[fix(o.name)] ? o : o.parent;
    if (!items[n]) items[n] = Object.assign({key: n, nodes: [], p0: []}, it);
    if (!items[n].nodes.includes(node)) { items[n].nodes.push(node); items[n].p0.push(node.position.clone()); }
    o.userData.key = n;
  }
  for (const l of Object.values(links)) { l.userData.q0 = l.quaternion.clone(); }
  // eyes on the LCD (LCD centre 59.5, 0, 103; tilted back 10 deg)
  const lcd = items['elec:lcd'];
  if (lcd) {
    const eye = new THREE.Mesh(new THREE.PlaneGeometry(32.4, 17.4), new THREE.MeshBasicMaterial({map: eyeTex, toneMapped: false}));
    const B = new THREE.Matrix4().makeBasis(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 1), new THREE.Vector3(1, 0, 0));
    const R = new THREE.Matrix4().makeRotationY(THREE.MathUtils.degToRad(-10));
    const M = R.clone().multiply(B); eye.quaternion.setFromRotationMatrix(M);
    const p = new THREE.Vector3(1.4, 0, 0).applyMatrix4(R).add(new THREE.Vector3(59.5, 0, 103));
    const host = lcd.nodes[0]; eye.position.copy(p).sub(host.position); host.add(eye); eye.userData.key = 'elec:lcd';
  }
  buildUI(); buildBOM(bom); applyAll(); setView(Q.get('view') || '34');
  document.body.classList.add('loaded');
}).catch(e => { $('loading').textContent = 'Could not load the model: ' + e; console.error(e); });

function buildUI() {
  const counts = {};
  for (const it of Object.values(items)) counts[it.group] = (counts[it.group] || 0) + 1;
  $('layers').innerHTML = Object.entries(GROUPS).map(([g, G]) =>
    `<label class="layer"><input type="checkbox" data-g="${g}" ${visible[g] ? 'checked' : ''}><span class="sw" style="background:${G.color}"></span>${G.name}<span class="n">${counts[g] || 0}</span></label>`).join('');
  $('layers').querySelectorAll('input').forEach(i => i.onchange = () => { visible[i.dataset.g] = i.checked; applyAll(); });
  const rows = {};
  for (const it of Object.values(items)) { const k = it.group + '|' + it.title; (rows[k] = rows[k] || {it, keys: [], n: 0}).keys.push(it.key); rows[k].n++; }
  let html = '', ncomp = 0;
  for (const g of Object.keys(GROUPS)) {
    const rs = Object.values(rows).filter(r => r.it.group === g); if (!rs.length) continue;
    html += `<div class="grp"><span class="sw" style="background:${GROUPS[g].color}"></span>${GROUPS[g].name}</div>`;
    for (const rw of rs) {
      ncomp++;
      const short = (rw.it.desc || '').split(/(?<=\.)\s/)[0];
      html += `<div class="c" data-keys="${rw.keys.join(',')}" data-g="${g}"><b>${rw.it.title}${rw.n > 1 ? ` &times;${rw.n}` : ''}</b><span>${short}</span></div>`;
    }
  }
  $('clist').innerHTML = html; $('ncomp').textContent = '(' + ncomp + ' types)';
  $('clist').querySelectorAll('.c').forEach(d => {
    d.onclick = () => select(d.dataset.keys.split(','));
    d.onmouseenter = () => highlight(d.dataset.keys.split(','), true); d.onmouseleave = () => { if (!sel || sel[0] !== d.dataset.keys.split(',')[0]) highlight(d.dataset.keys.split(','), false); };
  });
  const L = $('labels');
  for (const lb of RI.labels) {
    const it = items[lb.key]; if (!it) continue;
    const a = new THREE.Object3D(); a.position.fromArray(lb.local); (links[lb.node.slice(5)] || root).add(a);
    const d = document.createElement('div'); d.className = 'lbl'; d.innerHTML = `<i></i><span>${it.title}</span>`;
    d.onclick = () => select([lb.key]); L.appendChild(d);
    const ext = ['shell', 'legs'].includes(it.group) || ['part:bed'].includes(lb.key);
    labels.push({a, d, it, ext, key: lb.key});
  }
  const s = RI.summary;
  $('stats').innerHTML = `<div><b>${s.length_mm} mm</b>long</div><div><b>${s.height_mm} mm</b>tall</div><div><b>${(s.total_mass_g / 1000).toFixed(2)} kg</b>est. mass</div>` +
    `<div><b>${s.servos}</b>servos (1 type)</div><div><b>${s.molded_parts}</b>molded pieces</div><div><b>${s.unique_tools}</b>robot tools</div>`;
  document.querySelectorAll('.tabs button').forEach(b => b.onclick = () => {
    document.querySelectorAll('.tabs button').forEach(x => x.classList.toggle('on', x === b));
    document.querySelectorAll('.tab').forEach(t => t.classList.toggle('on', t.id === 'tab-' + b.dataset.tab));
  });
  if (Q.get('tab')) document.querySelector(`.tabs button[data-tab="${Q.get('tab')}"]`)?.click();
}
function buildBOM(b) {
  $('bom-v').textContent = '$' + b.volume_total.toFixed(2); $('bom-p').textContent = '$' + b.proto_total.toFixed(0);
  $('bom-table').innerHTML = b.volume.map(x => `<tr><td>${x.item.split(',')[0]}${x.qty > 1 ? ' &times;' + x.qty : ''}${x.src.startsWith('est') ? ' <i>est</i>' : ''}</td><td class="n">$${(x.qty * x.unit).toFixed(2)}</td></tr>`).join('') +
    `<tr class="t"><td>Total per robot</td><td class="n">$${b.volume_total.toFixed(2)}</td></tr>`;
}

function highlight(keys, on) {
  for (const k of keys) { const it = items[k]; if (!it) continue;
    for (const n of it.nodes) n.traverse(o => { if (o.isMesh && o.material.emissive && o.userData.key) { o.material.emissive.set(on ? 0xff6a3a : 0x000000); o.material.emissiveIntensity = on ? 0.45 : 1; } }); }
}
function select(keys) {
  if (sel) highlight(sel, false);
  sel = keys; highlight(keys, true);
  const it = items[keys[0]];
  $('info').classList.remove('hidden'); $('info-t').textContent = it.title + (keys.length > 1 ? ` (x${keys.length})` : '');
  $('info-k').textContent = (GROUPS[it.group] || {}).name + (it.kind === 'molded' ? ' · molded part' : ' · bought part') + (it.mass_g ? ` · ${it.mass_g} g` : '');
  $('info-d').textContent = it.desc + (it.material ? ' Material: ' + it.material + '.' : '');
  document.querySelectorAll('#clist .c').forEach(c => c.classList.toggle('sel', c.dataset.keys.split(',').includes(keys[0])));
  labels.forEach(l => l.d.classList.toggle('sel', keys.includes(l.key)));
}

function applyAll() {
  for (const it of Object.values(items)) {
    const vis = visible[it.group];
    it.nodes.forEach((n, i) => { n.visible = vis; n.position.copy(it.p0[i]).add(new THREE.Vector3(...it.explode).multiplyScalar(explodeT)); });
  }
  for (const m of shellMats) { const base = m.userData.base ?? 1; m.transparent = xray || base < 1; m.opacity = xray ? Math.min(0.12, base) : base; m.depthWrite = !xray && base === 1; m.needsUpdate = true; }
  $('xray').classList.toggle('on', xray); $('lbls').classList.toggle('on', showLabels); $('demo').classList.toggle('on', demo);
  $('explode').value = explodeT;
  document.querySelectorAll('#clist .c').forEach(c => c.classList.toggle('off', !visible[c.dataset.g]));
}

document.querySelectorAll('[data-view]').forEach(b => b.onclick = () => setView(b.dataset.view));
$('xray').onclick = () => { xray = !xray; applyAll(); };
$('lbls').onclick = () => { showLabels = !showLabels; applyAll(); };
$('demo').onclick = () => { demo = !demo; if (!demo) { for (const l of Object.values(links)) l.quaternion.copy(l.userData.q0); if (links.base) links.base.position.z = 0; } applyAll(); };
$('explode').oninput = e => { explodeT = +e.target.value; applyAll(); };

const ray = new THREE.Raycaster(), mp = new THREE.Vector2(); let down = null;
r.domElement.addEventListener('pointerdown', e => down = [e.clientX, e.clientY]);
r.domElement.addEventListener('pointerup', e => {
  if (!down || Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 4) return;
  const b = r.domElement.getBoundingClientRect(); mp.set((e.clientX - b.left) / b.width * 2 - 1, -(e.clientY - b.top) / b.height * 2 + 1);
  ray.setFromCamera(mp, cam);
  const hits = ray.intersectObject(root, true).filter(h => h.object.visible && h.object.userData.key && isShown(h.object) && !(xray && shellMats.includes(h.object.material)));
  if (hits.length) select([hits[0].object.userData.key]); else { if (sel) highlight(sel, false); sel = null; $('info').classList.add('hidden'); }
});
function isShown(o) { while (o) { if (!o.visible) return false; o = o.parent; } return true; }

// ---- trot demo: diagonal pairs, hip swing + knee lift (pushrod rides with the thigh in this preview)
const q = new THREE.Quaternion(), Y = new THREE.Vector3(0, 1, 0);
const PH = {fl: 0, rr: 0, fr: Math.PI, rl: Math.PI};
function stepDemo(t) {
  if (!demo) return;
  const w = t * 2 * Math.PI * 1.4;
  for (const [leg, ph] of Object.entries(PH)) {
    const th = links['thigh_' + leg], sh = links['shin_' + leg]; if (!th || !sh) continue;
    const p = w + ph, swing = Math.sin(p) > 0;
    q.setFromAxisAngle(Y, 0.28 * Math.cos(p)); th.quaternion.copy(th.userData.q0).multiply(q);
    q.setFromAxisAngle(Y, swing ? -0.55 * Math.sin(p) : 0); sh.quaternion.copy(sh.userData.q0).multiply(q);
  }
}
const v = new THREE.Vector3();
function placeLabels() {
  const w = el.clientWidth, h = el.clientHeight;
  for (const L of labels) {
    const vis = showLabels && visible[L.it.group] && (L.ext || xray || !visible.shell || explodeT > 0.35);
    if (!vis) { L.d.style.display = 'none'; continue; }
    L.a.getWorldPosition(v); v.project(cam);
    if (v.z > 1) { L.d.style.display = 'none'; continue; }
    L.d.style.display = ''; L.d.style.transform = `translate(${(v.x * .5 + .5) * w}px,${(-v.y * .5 + .5) * h}px)`;
  }
}
new ResizeObserver(() => { cam.aspect = el.clientWidth / el.clientHeight; cam.updateProjectionMatrix(); r.setSize(el.clientWidth, el.clientHeight); }).observe(el);
let lastEye = 0;
(function loop(t) { requestAnimationFrame(loop); const s = (t || 0) / 1000; stepDemo(s); if (s - lastEye > 0.04) { drawEyes(s); lastEye = s; } ctl.update(); placeLabels(); r.render(scene, cam); })();
