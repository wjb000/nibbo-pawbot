#!/usr/bin/env python3
"""Build everything from cad/nibbo.py + cad/elec.py:
  cad/step/*.step       molded parts (one per tool, assembly position) + cad/step/nibbo_assembly.step
  cad/stl/*.stl         prototype print files, ASCII, oriented for printing
  cad/reports/*.json    mass properties, interference check, part list
  site/assets/nibbo.gltf + robot_info.json   viewer model (link hierarchy, labels, layers, explode offsets)
Run:  pip install cadquery trimesh manifold3d fast-simplification  &&  python cad/export.py"""
import os, sys, json, math, time
import numpy as np, trimesh, cadquery as cq
import manifold3d as mf
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import nibbo as N, elec
from textmesh import write_ascii_stl, glb_to_embedded_gltf

DENS = {'PC/ABS': 1.15e-3, 'PA6-GF30': 1.36e-3, 'TPU 85A': 1.21e-3, 'PC (smoked)': 1.20e-3, 'ABS': 1.05e-3}
STL_FACES = int(os.environ.get('STL_FACES', 4500))
LEGS = list(N.LEGS)


def d(*p):
    q = os.path.join(ROOT, *p); os.makedirs(q, exist_ok=True); return q


def tess(shape, tol=0.15):
    v, f = shape.tessellate(tol, 0.2)
    m = trimesh.Trimesh(np.array([[p.x, p.y, p.z] for p in v]), np.array(f), process=True)
    m.merge_vertices(); trimesh.repair.fix_normals(m)
    return m


def wp_mesh(wp, tol=0.15):
    vals = wp.vals()
    return trimesh.util.concatenate([tess(s, tol) for s in vals]) if len(vals) > 1 else tess(vals[0], tol)


def to_mf(m):
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(m.vertices, np.float32), tri_verts=np.asarray(m.faces, np.uint32)))


def inter_vol(a, b):
    try:
        return (to_mf(a) ^ to_mf(b)).volume()
    except Exception:
        return float('nan')


def slim_step(path):
    """Trim STEP floats to 6 decimals (1 nm): same geometry, ~40 % smaller text files."""
    import re
    s = open(path).read()
    open(path, 'w').write(re.sub(r'(-?\d+\.\d{6})\d+', r'\1', s))


def decimate(m, faces):
    if len(m.faces) <= faces:
        return m
    try:
        return m.simplify_quadric_decimation(face_count=faces)
    except Exception:
        return m


PART_INFO = {
    'top_shell': ('Top shell', 'One molded shell: back, brow and the two leaf ears. Mic port; copper-foil touch pad inside. 4 screws to the chassis.', 'shell', 'PC/ABS', '#f1ece3'),
    'chassis': ('Chassis', 'The structural tub: four servo towers, battery cradle, PCB standoffs, speaker grille, cliff windows, tail socket. Everything mounts to this one part.', 'structure', 'PC/ABS', '#4a4f58'),
    'visor': ('Face visor', 'Smoked polycarbonate window over the eyes, camera and ToF. IR-transparent ink patch over the ToF.', 'shell', 'PC (smoked)', '#1a1f26'),
    'tail': ('Tail (charging plug)', 'Soft TPU tail overmolded on a 2-wire harness. Its tip holds the charging pads: Nibbo backs into its bed and plugs in with its tail.', 'shell', 'TPU 85A', '#ff7a59'),
    'thigh': ('Thigh', 'Same part on all four legs (symmetric). Bolts to the hip horn; the knee pushrod runs along its back edge. Glass-filled nylon.', 'legs', 'PA6-GF30', '#e6dfd3'),
    'shin': ('Shin', 'Same part on all four legs. Tapered blade, crank above the knee for the pushrod, TPU foot ball overmolded (2-shot).', 'legs', 'PA6-GF30', '#d6cec1'),
    'foot': ('Foot ball (TPU overmold)', 'Grippy, quiet 15 mm TPU ball molded onto the shin: no slipping on wood floors, no clicking.', 'legs', 'TPU 85A', '#ff7a59'),
    'pushrod': ('Knee pushrod', '1:1 link from the knee servo horn (in the body) to the shin crank, so the knee servo lives in the body instead of the leg.', 'legs', 'PA6-GF30', '#7b828d'),
    'bed': ('Charging bed', 'Oval bed Nibbo walks into and lies down in. Raised back holds the spring contacts and IR beacon; USB-C at the back.', 'dock', 'ABS', '#d8d1c4'),
}


def main():
    t0 = time.time()
    # ----------------------------------------------------------------- links and joints
    origin = {'base': np.zeros(3), 'bed': np.array([N.BED_AT[0], N.BED_AT[1], 0.0])}
    J = {}
    for leg, (fb, s) in N.LEGS.items():
        hx = fb * N.HIP_X; kx, kz = N.knee_pt(leg)
        origin['thigh_' + leg] = np.array([hx, s * 49.5, N.HIP_Z]); origin['shin_' + leg] = np.array([kx, s * 49.5, kz])
        J['hip_' + leg] = ('base', 'thigh_' + leg, (0, 1, 0), (-70, 70))
        J['knee_' + leg] = ('thigh_' + leg, 'shin_' + leg, (0, 1, 0), (-60, 60))
    # ----------------------------------------------------------------- molded parts
    N.hollow()
    parts = []
    def P(name, solid, link, inst=''):
        parts.append(dict(name=name, inst=inst or name, solid=solid, link=link))
    P('top_shell', N.top_shell(), 'base'); P('chassis', N.chassis(), 'base'); P('visor', N.visor(), 'base'); P('tail', N.tail(), 'base')
    for leg in LEGS:
        P('thigh', N.thigh(leg), 'thigh_' + leg, 'thigh_' + leg)
        P('shin', N.shin(leg), 'shin_' + leg, 'shin_' + leg)
        P('foot', N.foot(leg), 'shin_' + leg, 'foot_' + leg)
        P('pushrod', N.pushrod(leg), 'thigh_' + leg, 'pushrod_' + leg)
    bed = N.bed().translate((N.BED_AT[0], N.BED_AT[1], 0))
    P('bed', bed, 'bed')
    for p in parts:
        p['mesh'] = wp_mesh(p['solid'])
        p['vol'] = sum(v.Volume() for v in p['solid'].vals())
        nm, desc, grp, mat, col = PART_INFO[p['name']]
        p['mass'] = p['vol'] * DENS[mat]
    print('parts', round(time.time() - t0, 1), 's')
    E = elec.build()
    bed_shift = np.array([N.BED_AT[0], N.BED_AT[1], 0.0])
    for e in E:
        if e['link'] == 'bed':
            e['solid'] = e['solid'].translate(tuple(bed_shift))
        e['mesh'] = wp_mesh(e['solid'], 0.2)
    # ----------------------------------------------------------------- STEP (one per tool) + assembly
    sd = d('cad', 'step'); seen = set()
    for p in parts:
        if p['name'] in seen:
            continue
        seen.add(p['name'])
        cq.exporters.export(p['solid'], os.path.join(sd, p['name'] + '.step')); slim_step(os.path.join(sd, p['name'] + '.step'))
    asm = cq.Assembly(name='nibbo')
    for p in parts:
        nm, desc, grp, mat, col = PART_INFO[p['name']]
        asm.add(p['solid'], name=p['inst'], color=cq.Color(col))
    for e in E:
        asm.add(e['solid'], name=e['key'], color=cq.Color(e['color']))
    asm.save(os.path.join(sd, 'nibbo_assembly.step')); slim_step(os.path.join(sd, 'nibbo_assembly.step'))
    # ----------------------------------------------------------------- STL for prototype printing (ASCII, print-oriented)
    st = d('cad', 'stl'); seen = set(); print_rep = {}
    for p in parts:
        if p['name'] in seen or p['name'] == 'foot':
            continue
        seen.add(p['name'])
        m = p['mesh'].copy()
        if p['name'] in ('thigh', 'shin', 'pushrod'):          # lay flat on the side face
            m.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]))
        if p['name'] == 'top_shell':                           # open side down
            pass
        if p['name'] == 'visor':                               # stand on its rim edge
            m.apply_transform(trimesh.transformations.rotation_matrix(-math.pi / 2, [0, 1, 0]))
        m.apply_translation(-m.bounds[0] * [1, 1, 1] + [0, 0, 0])
        m = decimate(m, int(STL_FACES * (1.4 if p['name'] in ('top_shell', 'chassis', 'bed') else 0.8)))
        write_ascii_stl(m, os.path.join(st, p['name'] + '.stl'), p['name'])
        print_rep[p['name']] = dict(faces=len(m.faces), size_mm=np.round(m.extents, 1).tolist())
    # ----------------------------------------------------------------- reports
    rd = d('cad', 'reports')
    robot_parts = [p for p in parts if p['link'] != 'bed']
    robot_e = [e for e in E if e['link'] != 'bed']
    m_parts = sum(p['mass'] for p in robot_parts); m_e = sum(e['mass'] for e in robot_e)
    fasteners = 6.0; wires = 8.0
    total = m_parts + m_e + fasteners + wires
    com = (sum(p['mass'] * p['mesh'].center_mass for p in robot_parts) + sum(e['mass'] * e['mesh'].center_mass for e in robot_e)) / (m_parts + m_e)
    allm = trimesh.util.concatenate([p['mesh'] for p in robot_parts] + [e['mesh'] for e in robot_e])
    ext = allm.extents
    mass = dict(total_g=round(total, 1), molded_g=round(m_parts, 1), bought_g=round(m_e, 1), fasteners_wires_g=fasteners + wires,
                com_mm=np.round(com, 1).tolist(), length_mm=round(ext[0]), width_mm=round(ext[1]), height_mm=round(ext[2]),
                parts={p['inst']: round(p['mass'], 1) for p in parts})
    json.dump(mass, open(os.path.join(rd, 'mass_properties.json'), 'w'), indent=1)
    # interference: every pair at the nominal pose (expected contacts listed)
    objs = [(p['inst'], p['mesh']) for p in parts] + [(e['key'], e['mesh']) for e in E]
    hits = []
    for i in range(len(objs)):
        for j in range(i + 1, len(objs)):
            a, b = objs[i][1], objs[j][1]
            lo = np.maximum(a.bounds[0], b.bounds[0]); hi = np.minimum(a.bounds[1], b.bounds[1])
            if np.any(hi <= lo):
                continue
            v = inter_vol(a, b)
            if v > 1.0:
                hits.append((objs[i][0], objs[j][0], round(v, 1)))
    json.dump(dict(threshold_mm3=1.0, overlaps=hits), open(os.path.join(rd, 'interference.json'), 'w'), indent=1)
    print('interference overlaps > 1 mm3:', len(hits))
    for h in hits:
        print('  ', h)
    # ----------------------------------------------------------------- viewer glTF + info
    write_viewer(parts, E, origin, J, dict(height_mm=mass['height_mm'], length_mm=mass['length_mm'], width_mm=mass['width_mm'],
                                          total_mass_g=mass['total_g'], molded_parts=len([p for p in robot_parts if p['name'] != 'foot']), unique_tools=len({p['name'] for p in robot_parts} - {'foot'}),
                                          servos=8, com_mm=mass['com_mm']))
    json.dump(dict(print=print_rep, mass=mass), open(os.path.join(rd, 'summary.json'), 'w'), indent=1)
    print('done', round(time.time() - t0, 1), 's', mass['total_g'], 'g', ext)


EXPLODE = {'top_shell': (0, 0, 70), 'visor': (45, 0, 25), 'tail': (-45, 0, 10), 'chassis': (0, 0, 0), 'bed': (0, -40, 0)}
GROUP_E = {'servo': 'actuators', 'horn': 'actuators', 'battery': 'power', 'charger': 'power', 'buck': 'power', 'tailpad': 'power',
           'pcb': 'electronics', 'esp32': 'electronics', 'imu': 'electronics', 'amp': 'electronics',
           'camera': 'sensors', 'tof': 'sensors', 'cliff': 'sensors', 'mic': 'sensors', 'touch': 'sensors',
           'lcd': 'expression', 'speaker': 'expression', 'bed': 'dock'}
E_EXPLODE = {'battery': (0, 0, -45), 'pcb': (0, 0, 32), 'esp32': (0, 0, 40), 'imu': (0, 0, 40), 'amp': (0, 0, 40), 'charger': (0, 0, 40), 'buck': (0, 0, 40), 'mic': (0, 0, 40),
             'lcd': (30, 0, 12), 'camera': (28, 0, 0), 'tof': (28, 0, 0), 'cliff': (12, 0, -30), 'speaker': (0, 0, -40), 'touch': (0, 0, 85),
             'tailpad': (-45, 0, 10), 'bed': (0, -40, 0)}


def write_viewer(parts, E, origin, J, summary):
    out = d('site', 'assets')
    sc = trimesh.Scene()
    parent = {c: p for jn, (p, c, a, lim) in J.items()}
    order = ['base', 'bed'] + [J[j][1] for j in J]
    for l in order:
        T = np.eye(4)
        T[:3, 3] = origin[l] - (origin[parent[l]] if l in parent else 0)
        sc.graph.update(frame_from=('link:' + parent[l]) if l in parent else sc.graph.base_frame, frame_to='link:' + l, matrix=T)
    items, labels = {}, []

    def put(name, mesh, link, faces):
        m = mesh.copy(); m.apply_translation(-origin[link])
        m = decimate(m, faces); m.vertices = np.round(m.vertices, 2)
        sc.add_geometry(m, node_name=name, geom_name=name, parent_node_name='link:' + link)

    budget = {'top_shell': 9000, 'chassis': 9000, 'visor': 1800, 'tail': 1500, 'bed': 3000, 'thigh': 1100, 'shin': 700, 'foot': 300, 'pushrod': 250}
    LBL = {'top_shell': (-20, 30, 112), 'chassis': (-30, -38, 66), 'visor': (66, -14, 108), 'tail': (-101, 0, 80), 'bed': (N.BED_AT[0] + 60, N.BED_AT[1] - 70, 16)}
    for p in parts:
        key = 'part:' + p['inst']
        put(key, p['mesh'], p['link'], budget[p['name']])
        t, desc, grp, mat, col = PART_INFO[p['name']]
        leg = p['inst'].split('_')[-1] if p['inst'] != p['name'] else ''
        ex = EXPLODE.get(p['name'])
        if ex is None:
            fb, s = N.LEGS[leg]; ex = (fb * 10, s * (35 if p['name'] == 'thigh' else 50 if p['name'] in ('shin', 'foot') else 60), 0)
        items[key] = dict(title=t, desc=desc, group=grp, color=col, explode=list(ex), kind='molded', material=mat, mass_g=round(p['mass'], 1))
        if p['name'] in LBL:
            labels.append(dict(key=key, node='link:' + p['link'], local=(np.array(LBL[p['name']], float) - origin[p['link']]).tolist()))
        elif leg == 'fl':
            labels.append(dict(key=key, node='link:' + p['link'], local=(p['mesh'].bounds.mean(0) - origin[p['link']]).tolist()))
    seen = set()
    for e in E:
        key = 'elec:' + e['key']
        put(key, e['mesh'], e['link'], 900 if e['key'].startswith('servo') else 500)
        k0 = e['key'].split('_')[0]
        grp = GROUP_E.get(k0, 'electronics')
        ex = E_EXPLODE.get(k0)
        if ex is None:
            leg = e['key'].split('_')[-1]
            fb, s = N.LEGS.get(leg, (0, 0)); ex = (0, s * 22, 0)
        items[key] = dict(title=e['name'], desc=e['spec'], group=grp, color=e['color'], explode=list(ex), kind='bought', mass_g=e['mass'])
        if e.get('label') and e['name'] not in seen:
            seen.add(e['name'])
            labels.append(dict(key=key, node='link:' + e['link'], local=(e['mesh'].bounds.mean(0) - origin[e['link']]).tolist()))
    glb = os.path.join(out, 'nibbo.glb'); sc.export(glb)
    glb_to_embedded_gltf(glb, os.path.join(out, 'nibbo.gltf')); os.remove(glb)
    info = dict(summary=summary, joints={jn: dict(node='link:' + c, axis=list(a), limits_deg=list(lim)) for jn, (p, c, a, lim) in J.items()},
                items=items, labels=labels)
    json.dump(info, open(os.path.join(out, 'robot_info.json'), 'w'), indent=1)
    print('wrote nibbo.gltf', os.path.getsize(os.path.join(out, 'nibbo.gltf')) // 1024, 'kB')


if __name__ == '__main__':
    main()
