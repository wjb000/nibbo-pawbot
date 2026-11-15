"""Nibbo: parametric CadQuery model of every molded part.
Frame: x forward, y left, z up, millimetres, floor at z = 0, robot standing in its nominal pose.
Change a number here and re-run `python cad/export.py`.

Molded parts on the robot (7 tools, 16 pieces): top_shell, chassis, visor, tail, thigh x4, shin x4 (TPU foot 2-shot), pushrod x4.
Plus the bed (charging dock): 1 tool."""
import math
import cadquery as cq

# ---------------------------------------------------------------- leg geometry (see docs/TORQUE.md)
L1, L2 = 45.0, 50.0          # hip->knee, knee->foot-centre
FOOT_R = 7.5                 # TPU foot ball (overmolded on the shin)
HIP_X, HIP_Z = 44.0, 88.0    # hip axes at x = +/-44, z = 88 (all four legs)
KNEE_DZ = 15.0               # knee servo shaft sits 15 mm above the hip shaft (servos stacked in the body)
HORN, CRANK = 12.0, 12.0     # knee servo horn and shin crank: 1:1 knee drive through the pushrod
SERVO_FACE_Y = 34.0          # servo case top face (|y|); spline + horn reach 40.5..42.5
THIGH_Y = (42.5, 56.5)       # thigh capsule spans this |y| band (pushrod runs inside it)
SHIN_Y = (45.5, 53.5)
FOOT_Y = 49.5
STAND_D = HIP_Z - FOOT_R     # hip to foot-centre distance in the nominal stand

# ---------------------------------------------------------------- body (one lofted "pebble", split at SPLIT_Z)
WALL = 2.0
SPLIT_Z = 90.0
# (x, half-width, z_bottom, z_top) -> rounded-rectangle sections lofted into a smooth pebble
SECTIONS = [(-70.0, 29.0, 70.0, 110.0), (-52.0, 38.0, 61.0, 118.0), (0.0, 38.5, 58.5, 116.0),
            (42.0, 38.0, 59.5, 119.0), (66.0, 32.0, 67.0, 120.0)]
VISOR = dict(x0=50.0, w=56.0, z0=82.0, z1=117.0, r=12.0)      # smoked PC face window region (y width, z range)
BATT_Y = (-10.0, 10.0); BATT_Z = 71.5                           # 2 x 18650 lying along the body, side by side
PCB = dict(x0=-19.5, x1=19.5, y=28.0, z=84.0, t=1.6)
LEGS = {'fl': (+1, +1), 'fr': (+1, -1), 'rl': (-1, +1), 'rr': (-1, -1)}   # (front/back, left/right)
BED_AT = (-40.0, -230.0)     # viewer only: bed drawn beside the robot

def ik(d=STAND_D):
    """Nominal stand: foot straight under the hip. Returns (hip angle from straight-down, rad, + = knee to the rear),
    knee point offset (dx, dz) from the hip, and the shin direction."""
    a = math.acos((L1 ** 2 + d ** 2 - L2 ** 2) / (2 * L1 * d))
    kx, kz = -L1 * math.sin(a), -L1 * math.cos(a)
    return a, (kx, kz)

A_HIP, (KDX, KDZ) = ik()
def knee_pt(leg):
    fb, s = LEGS[leg]; return (fb * HIP_X + KDX, HIP_Z + KDZ)
def foot_pt(leg):
    fb, s = LEGS[leg]; return (fb * HIP_X, FOOT_R)


def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    return cq.Workplane('XY').box(x1 - x0, y1 - y0, z1 - z0).translate(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def cyl_y(x, z, r, y0, y1):
    y0, y1 = sorted((y0, y1))
    return cq.Workplane('XZ', origin=(0, y1, 0)).center(x, z).circle(r).extrude(y1 - y0)


def _section(x, hw, z0, z1, t=0.0):
    hw -= t; z0 += t; z1 -= t
    w, h = 2 * hw, z1 - z0
    r = min(hw, h / 2) * 0.62
    return cq.Workplane('YZ', origin=(x, 0, (z0 + z1) / 2)).sketch().rect(w, h).vertices().fillet(r).finalize()


def pebble(t=0.0):
    secs = list(SECTIONS)
    secs[0] = (secs[0][0] + t, *secs[0][1:]); secs[-1] = (secs[-1][0] - t, *secs[-1][1:])
    ws = []
    for (x, hw, z0, z1) in secs:
        hw2, z02, z12 = hw - t, z0 + t, z1 - t
        ws.append(_rrect_wire(x, hw2, z02, z12, min(hw2, (z12 - z02) / 2) * 0.52))
    s = cq.Solid.makeLoft(ws, ruled=False)
    body = cq.Workplane('XY').add(s)
    rr = 13.0 - t
    try:
        body = body.faces('<X or >X').edges().fillet(rr)
    except Exception as e:
        print('end fillet failed', e)
    return body


def _rrect_wire(x, hw, z0, z1, r):
    zc = (z0 + z1) / 2; hh = (z1 - z0) / 2
    w = (cq.Workplane('YZ', origin=(x, 0, zc)).moveTo(-hw + r, -hh).lineTo(hw - r, -hh)
         .radiusArc((hw, -hh + r), -r).lineTo(hw, hh - r).radiusArc((hw - r, hh), -r)
         .lineTo(-hw + r, hh).radiusArc((-hw, hh - r), -r).lineTo(-hw, -hh + r).radiusArc((-hw + r, -hh), -r).close())
    return w.wires().val()


def visor_region(pad=0.0):
    v = VISOR
    return (cq.Workplane('YZ', origin=(v['x0'] - pad, 0, (v['z0'] + v['z1']) / 2))
            .sketch().rect(v['w'] + 2 * pad, v['z1'] - v['z0'] + 2 * pad).vertices().fillet(v['r'] + pad).finalize().extrude(40))


def leg_slots():
    """Side-wall slots for the hip shaft + knee horn standoff of each leg."""
    out = None
    for leg, (fb, s) in LEGS.items():
        x = fb * HIP_X
        inb = -fb
        sl = cyl_y(x, HIP_Z, 7.5, s * 30, s * 48).union(cyl_y(x, HIP_Z + KNEE_DZ, 6.6, s * 30, s * 48)).union(
            box(x - 6.6, x + 6.6, s * 30, s * 48, HIP_Z, HIP_Z + KNEE_DZ))
        for zz in (HIP_Z, HIP_Z + KNEE_DZ):
            sl = sl.union(box(x, x + inb * 10.5, s * 33.5, s * 48, zz - 4.2, zz + 4.2))
        out = sl if out is None else out.union(sl)
    return out


_cache = {}
def hollow():
    if 'h' not in _cache:
        _cache['outer'] = pebble(0.0); _cache['inner'] = pebble(WALL)
        _cache['h'] = _cache['outer'].cut(_cache['inner']).cut(leg_slots())
    return _cache['h']


def ear(sy):
    """Leaf-shaped ear fin, molded into the top shell."""
    e = (cq.Workplane('XZ').ellipse(8.0, 12.5).extrude(1.8, both=True).edges().fillet(0.8))
    e = e.rotate((0, 0, 0), (0, 0, 1), sy * -12).rotate((0, 0, 0), (1, 0, 0), sy * -24).rotate((0, 0, 0), (0, 1, 0), -18)
    return e.translate((22.0, sy * 21.0, 121.5))


def top_shell():
    h = hollow()
    t = h.intersect(box(-90, 90, -60, 60, SPLIT_Z, 140)).cut(visor_region())
    for sy in (1, -1):
        t = t.union(ear(sy).cut(_cache['inner']))
    # mic port + speaker-free top; touch area is inside (copper foil)
    t = t.cut(cyl_z(-6, 0, 0.6, 100, 130))
    # 4 screw bosses down into the chassis
    for (x, y) in ((-36, 0), (8, 32), (8, -32), (36, 0)):
        t = t.union(cyl_z(x, y, 3.2, SPLIT_Z, 116).cut(cyl_z(x, y, 1.1, SPLIT_Z - 1, 113)).intersect(_cache['inner']))
    return t


def cyl_x(y, z, r, x0, x1):
    return cq.Workplane('YZ', origin=(x0, 0, 0)).center(y, z).circle(r).extrude(x1 - x0)


def cyl_z(x, y, r, z0, z1):
    return cq.Workplane('XY', origin=(0, 0, z0)).center(x, y).circle(r).extrude(z1 - z0)


def servo_env(x_shaft, z_shaft, s, inboard):
    """MG92B-class envelope (case + tabs) for clearance pockets. Shaft along y, case extends `inboard` (+1/-1 in x)."""
    a, b = (x_shaft - 6.0, x_shaft + 17.0) if inboard > 0 else (x_shaft - 17.0, x_shaft + 6.0)
    case = box(a, b, s * (SERVO_FACE_Y - 22.5), s * SERVO_FACE_Y, z_shaft - 6.1, z_shaft + 6.1)
    ta, tb = a - 4.75, b + 4.75
    tabs = box(ta, tb, s * (SERVO_FACE_Y - 6.5), s * (SERVO_FACE_Y - 4.0), z_shaft - 6.1, z_shaft + 6.1)
    return case.union(tabs), (a, b, ta, tb)


def chassis():
    h = hollow()
    c = h.intersect(box(-90, 90, -60, 60, 40, SPLIT_Z)).cut(visor_region())
    inner = _cache['inner']
    # servo towers: one frame per leg holding the hip + knee servo stack
    for leg, (fb, s) in LEGS.items():
        x = fb * HIP_X; inb = -fb               # case extends toward the body centre
        e1, (a, b, ta, tb) = servo_env(x, HIP_Z, s, inb)
        e2, _ = servo_env(x, HIP_Z + KNEE_DZ, s, inb)
        lo, hi = min(ta, tb) - 2.0, max(ta, tb) + 2.0
        frame = box(lo, hi, s * (SERVO_FACE_Y - 9.0), s * (SERVO_FACE_Y - 2.5), HIP_Z - 9.0, HIP_Z + KNEE_DZ + 7.0)
        frame = frame.cut(box(min(a, b) - 0.2, max(a, b) + 0.2, s * 20, s * 40, HIP_Z - 6.3, HIP_Z + KNEE_DZ + 6.3))
        post = box(lo, hi, s * (SERVO_FACE_Y - 9.0), s * (SERVO_FACE_Y - 2.5), 50, HIP_Z - 9.0).intersect(inner)
        c = c.union(frame.union(post).cut(e1).cut(e2).intersect(pebble(WALL + 0.8)))
    # battery cradle (two half-pipes) + retaining strap lugs
    for by in BATT_Y:
        for x0 in (-22.0, 14.0):
            cr = cyl_x(by, BATT_Z, 11.3, x0, x0 + 8).cut(cyl_x(BATT_Y[0], BATT_Z, 9.5, -40, 40)).cut(cyl_x(BATT_Y[1], BATT_Z, 9.5, -40, 40)).intersect(box(-40, 40, by - 12, by + 12, 55, BATT_Z - 2))
            c = c.union(cr.intersect(inner))
    # PCB standoffs
    for (x, y) in ((-16, -25), (-16, 25), (16, -25), (16, 25)):
        c = c.union(cyl_z(x, y, 2.6, 55, PCB['z'] - 0.2).cut(cyl_z(x, y, 1.0, 60, PCB['z'] + 1)).intersect(inner))
    # screw bosses for the top shell
    for (x, y) in ((-36, 0), (8, 32), (8, -32), (36, 0)):
        c = c.union(cyl_z(x, y, 3.2, 55, SPLIT_Z).cut(cyl_z(x, y, 1.1, 55, SPLIT_Z + 1)).intersect(inner))
    # belly openings: speaker grille, cliff-sensor windows, tail socket, factory test-pad port
    for i in range(-2, 3):
        for j in range(-2, 3):
            if i * i + j * j <= 5:
                c = c.cut(cyl_z(-51 + 3.2 * i, 3.2 * j, 0.9, 50, 66))
    for sy in (1, -1):
        c = c.cut(box(50, 56, sy * 15, sy * 21, 50, 64))
    c = c.cut(cq.Workplane('YZ', origin=(-80, 0, 80)).circle(5.3).extrude(14))
    return c


def visor():
    v = _cache['outer'].cut(_cache['inner']).intersect(visor_region(-0.3))
    return v


def _thigh_profile(leg):
    fb, s = LEGS[leg]
    hx, hz = fb * HIP_X, HIP_Z
    kx, kz = knee_pt(leg)
    return hx, hz, kx, kz


def thigh(leg='fl'):
    """Thigh capsule: two rounded ends (hip r11, knee r8) joined by tangent sides, 14 mm thick, hollow channel for the
    pushrod, clevis at the knee for the shin. Identical part on all four legs (flip for the right side)."""
    fb, s = LEGS[leg]
    hx, hz, kx, kz = _thigh_profile(leg)
    y0, y1 = s * THIGH_Y[0], s * THIGH_Y[1]
    ends = cyl_y(hx, hz, 10.0, y0, y1).union(cyl_y(kx, kz, 8.5, y0, y1))
    # hull approximated: lofted web between the ends
    L = math.hypot(kx - hx, kz - hz); ang = math.degrees(math.atan2(kz - hz, kx - hx))
    web = (cq.Workplane('XZ', origin=(0, max(y0, y1), 0)).center(hx, hz)
           .polyline([(0, 10.0), (L, 8.5), (L, -8.5), (0, -10.0)]).close().extrude(abs(y1 - y0)))
    web = web.rotate((hx, 0, hz), (hx, 1, hz), -ang)
    t = ends.union(web)
    try:
        t = t.faces('>Y or <Y').edges().fillet(3.0)
    except Exception:
        pass
    # pushrod + crank clearance (the pushrod runs inside the thigh, entering through a slot on its top edge)
    t = t.cut(pushrod(leg, fat=1.2)).cut(cyl_y(*horn_tip(leg)[1], 5.2, s * 46.0, s * 53.0))
    t = t.cut(cyl_y(kx, kz, 8.6, s * SHIN_Y[0] - s * 0.3, s * SHIN_Y[1] + s * 0.3))
    ux, uz = (foot_pt(leg)[0] - kx), (foot_pt(leg)[1] - kz); Ls = math.hypot(ux, uz); ang2 = math.degrees(math.atan2(uz, ux))
    t = t.cut(cq.Workplane('XZ', origin=(0, s * (SHIN_Y[1] + 0.3) if s > 0 else -SHIN_Y[0] + 0.3, 0)).center(kx, kz)
              .polyline([(0, 4.6), (-CRANK - 5, 4.6), (-CRANK - 5, -4.6), (0, -4.6)]).close().extrude(SHIN_Y[1] - SHIN_Y[0] + 0.6)
              .rotate((kx, 0, kz), (kx, 1, kz), -ang2))
    t = t.cut(cq.Workplane('XZ', origin=(0, s * (SHIN_Y[1] + 0.3) if s > 0 else -SHIN_Y[0] + 0.3, 0)).center(kx, kz)
              .polyline([(0, 8.7), (16, 8.7), (16, -8.7), (0, -8.7)]).close().extrude(SHIN_Y[1] - SHIN_Y[0] + 0.6)
              .rotate((kx, 0, kz), (kx, 1, kz), -ang2))
    t = t.cut(cyl_y(hx, hz, 1.6, y0 - s, y1 + s)).cut(cyl_y(kx, kz, 1.6, y0 - s, y1 + s))
    return t


def shin(leg='fl'):
    """Shin: tapered blade from the knee to the foot ball, crank above the knee for the pushrod, TPU foot overmolded."""
    fb, s = LEGS[leg]
    kx, kz = knee_pt(leg); fx, fz = foot_pt(leg)
    y0, y1 = s * SHIN_Y[0], s * SHIN_Y[1]
    L = math.hypot(fx - kx, fz - kz); ang = math.degrees(math.atan2(fz - kz, fx - kx))
    blade = (cq.Workplane('XZ', origin=(0, max(y0, y1), 0)).center(kx, kz)
             .polyline([(0, 7.5), (L, 4.0), (L, -4.0), (0, -7.5)]).close().extrude(abs(y1 - y0))
             .rotate((kx, 0, kz), (kx, 1, kz), -ang))
    knee = cyl_y(kx, kz, 7.5, y0, y1)
    # crank: points opposite the shin, 12 mm, ending in a pin boss
    ux, uz = (fx - kx) / L, (fz - kz) / L
    cx, cz = kx - CRANK * ux, kz - CRANK * uz
    crank = cyl_y(cx, cz, 3.5, y0, y1)
    web = (cq.Workplane('XZ', origin=(0, max(y0, y1), 0)).center(kx, kz)
           .polyline([(0, 3.5), (-CRANK, 3.5), (-CRANK, -3.5), (0, -3.5)]).close().extrude(abs(y1 - y0))
           .rotate((kx, 0, kz), (kx, 1, kz), -ang))
    sh = blade.union(knee).union(crank).union(web)
    try:
        sh = sh.faces('>Y or <Y').edges().fillet(1.5)
    except Exception:
        pass
    sh = sh.cut(cyl_y(kx, kz, 1.6, y0 - s, y1 + s)).cut(cyl_y(cx, cz, 1.1, y0 - s, y1 + s)).cut(pushrod(leg, fat=0.4).intersect(cyl_y(cx, cz, 9.0, -80, 80)))
    return sh


def foot(leg='fl'):
    fx, fz = foot_pt(leg); s = LEGS[leg][1]
    return cq.Workplane('XY').sphere(FOOT_R).translate((fx, s * FOOT_Y, fz)).cut(shin(leg))


def horn_tip(leg):
    """Knee servo horn tip in the nominal stand (horn points toward the crank tip direction)."""
    fb, s = LEGS[leg]
    kx, kz = knee_pt(leg); fx, fz = foot_pt(leg)
    L = math.hypot(fx - kx, fz - kz); ux, uz = (fx - kx) / L, (fz - kz) / L
    cx, cz = kx - CRANK * ux, kz - CRANK * uz
    # parallelogram-ish: horn tip = knee servo axis + (crank tip - knee)
    return (fb * HIP_X + (cx - kx), HIP_Z + KNEE_DZ + (cz - kz)), (cx, cz)


def pushrod(leg='fl', fat=0.0):
    fb, s = LEGS[leg]
    (hx, hz), (cx, cz) = horn_tip(leg)
    L = math.hypot(cx - hx, cz - hz); ang = math.degrees(math.atan2(cz - hz, cx - hx))
    y0, y1 = s * (47.5 - fat), s * (51.5 + fat)
    w = 2.5 + fat
    r = (cq.Workplane('XZ', origin=(0, max(y0, y1), 0)).center(hx, hz)
         .polyline([(0, w), (L, w), (L, -w), (0, -w)]).close().extrude(abs(y1 - y0))
         .rotate((hx, 0, hz), (hx, 1, hz), -ang))
    r = r.union(cyl_y(hx, hz, 3.5 + fat, y0, y1)).union(cyl_y(cx, cz, 3.5 + fat, y0, y1))
    if fat:
        return r
    return r.cut(cyl_y(hx, hz, 1.1, y0 - s, y1 + s)).cut(cyl_y(cx, cz, 1.1, y0 - s, y1 + s))


TAIL_PTS = [(-69.0, 0.0, 80.0), (-82.0, 0.0, 88.0), (-97.0, 0.0, 84.0), (-104.0, 0.0, 68.0), (-105.0, 0.0, 54.0)]
def tail():
    """Soft TPU tail overmolded on a 2-wire harness. The tip carries the two charging contacts: Nibbo backs into
    its bed and 'plugs in' with its tail. A 14 mm plug snaps into the chassis socket."""
    path = cq.Workplane('XZ').spline([(p[0], p[2]) for p in TAIL_PTS])
    p0 = TAIL_PTS[0]
    prof = cq.Workplane(cq.Plane(origin=p0, xDir=(0, 1, 0), normal=(-13, 0, 8))).circle(5.0)
    t = prof.sweep(path, transition='round')
    tip = cq.Workplane('XY').sphere(6.5).translate(TAIL_PTS[-1])
    t = t.union(tip)
    if 'outer' in _cache:
        t = t.cut(_cache['outer'])
    plug = cq.Workplane('YZ', origin=(-80, 0, 80)).circle(5.0).extrude(13)
    tip = TAIL_PTS[-1]
    for sy in (1, -1):      # pockets for the two insert-molded gold contact pads
        t = t.cut(cq.Workplane('XY').sphere(2.0).translate((tip[0] - 5.0, sy * 2.6, tip[2] - 2.2)))
    return t.union(plug)


BED = dict(rx=122.0, ry=80.0, h=16.0, back_h=46.0, pin_z=30.0)
def bed():
    """Charging bed: an oval cushion-dish Nibbo walks into and lies down in. Raised back holds two spring contacts at
    tail-tip height + IR beacon so it can find the bed. USB-C in at the back."""
    bx, by = 0.0, 0.0
    dish = cq.Workplane('XY').ellipse(BED['rx'], BED['ry']).extrude(BED['h']).edges('>Z').fillet(7.0)
    dish = dish.cut(cq.Workplane('XY', origin=(0, 0, 4)).ellipse(BED['rx'] - 9, BED['ry'] - 9).extrude(30).edges('<Z').fillet(6.0))
    back = (cq.Workplane('XY').ellipse(BED['rx'], BED['ry']).extrude(BED['back_h'])
            .cut(cq.Workplane('XY', origin=(0, 0, -1)).ellipse(BED['rx'] - 9, BED['ry'] - 9).extrude(80))
            .intersect(box(-200, -55, -200, 200, 0, 200)))
    try:
        back = back.edges('>Z').fillet(4.0)
    except Exception:
        pass
    b = dish.union(back)
    bx = -BED['rx'] + 4.5
    b = b.cut(box(-BED['rx'] - 5, -BED['rx'] + 12, -6, 6, 6, 14))          # USB-C notch
    b = b.cut(box(bx - 1.4, bx + 1.4, -15, 15, 16, 41))                   # board pocket (closed by a snap cover)
    for sy in (1, -1):
        b = b.cut(cq.Workplane('YZ', origin=(bx, sy * 2.6, BED['pin_z'])).circle(1.9).extrude(8))
    b = b.cut(cq.Workplane('YZ', origin=(bx, 0, 22)).circle(2.8).extrude(8))
    return b
