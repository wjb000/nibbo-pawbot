"""Bought parts placed in the assembly, modelled from datasheet dimensions (kind='modelled').
Each entry: key, name, CadQuery solid, mass (g), link, spec text, bom key, colour."""
import math
import cadquery as cq
from nibbo import *


def servo(x_shaft, z_shaft, s, inboard, horn='disk', horn_y=None):
    """MG92B-class micro servo: 23 x 12.2 x 22.5 mm case, 32.5 mm tab span, 20T spline. Shaft along +s*y."""
    a, b = (x_shaft - 6.0, x_shaft + 17.0) if inboard > 0 else (x_shaft - 17.0, x_shaft + 6.0)
    F = SERVO_FACE_Y
    case = box(a, b, s * (F - 22.5), s * F, z_shaft - 6.1, z_shaft + 6.1).edges('|Y').fillet(1.0)
    tabs = box(a - 4.75, b + 4.75, s * (F - 6.5), s * (F - 4.0), z_shaft - 6.1, z_shaft + 6.1)
    for tx in (a - 2.4, b + 2.4):
        tabs = tabs.cut(cyl_y(tx, z_shaft, 1.0, s * (F - 7), s * (F - 3)))
    boss = cyl_y(x_shaft, z_shaft, 5.8, s * F, s * (F + 3.0)).union(cyl_y(x_shaft + inboard * 6, z_shaft, 3.5, s * F, s * (F + 2.0)))
    spline = cyl_y(x_shaft, z_shaft, 2.4, s * (F + 3.0), s * (F + 6.5))
    body = case.union(tabs).union(boss).union(spline)
    return body


def horn_disk(x, z, s):
    return cyl_y(x, z, 9.0, s * 40.5, s * 42.5)


def horn_arm(leg):
    fb, s = LEGS[leg]
    kx0, kz0 = fb * HIP_X, HIP_Z + KNEE_DZ
    (hx, hz), _ = horn_tip(leg)
    L = math.hypot(hx - kx0, hz - kz0); ang = math.degrees(math.atan2(hz - kz0, hx - kx0))
    stand = cyl_y(kx0, kz0, 3.5, s * 40.5, s * 44.0)
    arm = (cq.Workplane('XZ', origin=(0, max(s * 44.0, s * 47.0), 0)).center(kx0, kz0)
           .polyline([(0, 3.5), (L, 2.6), (L, -2.6), (0, -3.5)]).close().extrude(3.0)
           .rotate((kx0, 0, kz0), (kx0, 1, kz0), -ang))
    return stand.union(arm).union(cyl_y(hx, hz, 2.8, s * 44.0, s * 47.0))


def build():
    E = []

    def add(key, name, solid, mass, link, spec, bom=None, label=True, color='#555b66', kind='modelled'):
        E.append(dict(key=key, name=name, solid=solid, mass=mass, link=link, kind=kind, spec=spec, bom=bom, label=label, color=color))

    servo_spec = ('MG92B-class metal-gear micro servo, 3.5 kg-cm (0.34 N-m) stall at 6 V, 0.08 s/60 deg, 13.8 g. '
                  'Production version ordered with a 4th wire on the potentiometer wiper so the ESP32 reads joint angle. '
                  'Peak need is 0.15 N-m (self-righting), trot needs about 0.1 N-m: see docs/TORQUE.md.')
    first = True
    for leg, (fb, s) in LEGS.items():
        x = fb * HIP_X
        add(f'servo_hip_{leg}', 'Hip servo (MG92B-class)', servo(x, HIP_Z, s, -fb), 13.8, 'base', servo_spec, 'servo', label=first, color='#2d3138')
        add(f'servo_knee_{leg}', 'Knee servo (MG92B-class)', servo(x, HIP_Z + KNEE_DZ, s, -fb), 13.8, 'base',
            'Same servo as the hips (one part number for all 8). Drives the shin through a 1:1 horn/pushrod/crank, so the leg stays slim and the servo mass sits in the body.',
            'servo', label=first, color='#2d3138')
        add(f'horn_hip_{leg}', 'Hip horn (round, 20T)', horn_disk(x, HIP_Z, s), 1.0, f'thigh_{leg}', 'Aluminium round horn that comes with the servo; 4 x M2 screws into the thigh.', None, label=False, color='#b9bec7')
        add(f'horn_knee_{leg}', 'Knee horn + standoff', horn_arm(leg), 1.2, 'base', '12 mm single-arm horn on a 3.5 mm standoff, M2 shoulder pin to the pushrod.', None, label=False, color='#b9bec7')
        first = False
    # battery: 2 x 18650 lying across the body, 2S
    cells = None
    for by in BATT_Y:
        c = cyl_x(by, BATT_Z, 9.25, -32.5, 32.5)
        cells = c if cells is None else cells.union(c)
    add('battery', '2S Li-ion pack, 2 x 18650 (7.2 V, 2500 mAh)', cells, 92.0, 'base',
        '18 Wh with protection IC on the main board. Lying low along the belly between the four servo towers keeps the centre of mass low and central. Estimated 60-90 min of active play, ~8 h of watching/idling.',
        'battery', color='#3b6fb6')
    p = PCB
    add('pcb', 'Main board (4-layer, 42 x 56 mm)', box(p['x0'], p['x1'], -p['y'], p['y'], p['z'], p['z'] + p['t']), 7.0, 'base',
        'One board does everything: ESP32-S3 module, 8 servo outputs with angle feedback, 2S charger (IP2326), protection, 6 V / 5 A servo buck, 3.3 V rail, I2S amp, IMU, mic, touch inputs. No USB port: charges through the tail, updates over Wi-Fi, factory-programmed on test pads.',
        'pcb', color='#1f6f43')
    zt = p['z'] + p['t']
    add('esp32', 'ESP32-S3-WROOM-1-N16R8', box(-5.0, 20.5, -9.0, 9.0, zt, zt + 3.1), 2.0, 'base',
        'Dual-core 240 MHz, 16 MB flash, 8 MB PSRAM, Wi-Fi + BLE (pre-certified module). Runs the gait, the face, wake-word and person/pet detection (ESP-DL), and streams the camera to the phone app.',
        'esp32', color='#c9ccd2')
    add('imu', 'IMU (ICM-42670-P)', box(-14, -11.5, 8, 11, zt, zt + 0.9), 0.1, 'base', '6-axis IMU: balance, fall detection, "pick-up" and petting gestures.', 'imu', color='#202226')
    add('amp', 'I2S amp (MAX98357A)', box(-14, -11, -12, -9, zt, zt + 0.9), 0.1, 'base', '3 W class-D amp for the voice/chirp speaker.', 'amp', label=False, color='#202226')
    add('charger', '2S charger IC (IP2326)', box(-18, -14, -4, 0, zt, zt + 0.8), 0.1, 'base', 'Boost-charges the 2S pack from 5 V at the bed contacts, with balancing.', 'charger', label=False, color='#202226')
    add('buck', '6 V / 5 A servo buck + inductor', box(-18, -12, 14, 20, zt, zt + 4.0), 1.5, 'base', 'Synchronous buck from the 2S pack to 6 V for the servos (full MG92B torque).', 'buck', color='#3a3d44')
    add('mic', 'MEMS microphone (I2S)', box(-10.5, -7.5, -1.5, 1.5, zt, zt + 1.0), 0.1, 'base', 'Hears its name, claps and the wake word; port hole in the top shell above.', 'mic', color='#9aa0a8')
    # face: 1.47" LCD, landscape, tilted back 10 deg behind the visor
    lcd = box(-1.25, 1.25, -19.25, 19.25, -11.0, 11.0).rotate((0, 0, 0), (0, 1, 0), -10).translate((59.5, 0, 103.0))
    add('lcd', '1.47" IPS face display (172 x 320, ST7789)', lcd, 6.0, 'base',
        'Landscape behind the smoked visor: draws the two eyes, blinks, moods, and icons (timer, weather, battery). The whole personality lives here.',
        'lcd', color='#0b0d10')
    cam = box(-2.5, 2.5, -5, 5, -5, 5).union(cq.Workplane('YZ', origin=(2.5, 0, 0)).circle(4.0).extrude(4.5)).translate((54.5, 9.0, 88.0))
    add('camera', 'Camera, OV2640 160 deg wide (DVP)', cam, 1.5, 'base',
        'Sees people, pets, the bed marker and obstacles. Wide lens so the home check-in view covers a room from floor height.', 'camera', color='#15171a')
    tof = box(-1.5, 1.5, -3.5, 3.5, -2.5, 2.5).translate((58.5, -9.0, 88.0))
    add('tof', 'ToF distance sensor (VL53L1X)', tof, 0.5, 'base', '4 m laser ranging through an IR window in the visor: obstacle stop + "come here" approach distance.', 'tof', color='#30343b')
    for sy, s_ in ((1, 'l'), (-1, 'r')):
        add(f'cliff_{s_}', 'Cliff sensor (IR reflective)', box(51, 55, sy * 16, sy * 20, 61.0, 64.0), 0.2, 'base',
            'Downward IR pair behind each front window: stops Nibbo at table and stair edges.', 'cliff', label=(sy > 0), color='#5d2f86')
    spk = cyl_z(-51, 0, 10.0, 66.5, 71.0)
    add('speaker', 'Speaker 20 mm, 8 ohm 1 W', spk, 3.0, 'base', 'Faces down through the belly grille; the floor reflects the sound out. Chirps, purrs and speaks reminders.', 'speaker', color='#2a2c31')
    foil = box(-28, 8, -16, 16, 111.0, 111.3)
    add('touch', 'Touch pad (copper foil on shell inside)', foil, 0.5, 'base', 'ESP32 capacitive touch on the back: pet it and it leans in, purrs and its eyes go soft.', 'touch', color='#c7843d')
    tip = TAIL_PTS[-1]
    for sy in (1, -1):
        add(f'tailpad_{"l" if sy > 0 else "r"}', 'Tail charging contacts (gold pads)', cq.Workplane('XY').sphere(2.0).translate((tip[0] - 5.0, sy * 2.6, tip[2] - 2.2)), 0.2, 'base',
            'Two gold-plated pads in the tail tip. Nibbo backs into its bed and the tail lands on the bed\'s spring contacts.', 'tailpads', label=(sy > 0), color='#d9a441')
    # bed electronics (link 'bed')
    bx = -BED['rx'] + 4.5
    add('bed_pcb', 'Bed board: USB-C in + IR beacon', box(bx - 0.8, bx + 0.8, -14, 14, 17, 40), 4.0, 'bed',
        'USB-C 5 V in, two spring contacts at tail-tip height, and a modulated IR LED beacon Nibbo homes on when its battery is low.', 'bedpcb', color='#1f6f43')
    for sy in (1, -1):
        add(f'bed_pin_{"l" if sy > 0 else "r"}', 'Bed spring contacts', cq.Workplane('YZ', origin=(bx + 0.8, sy * 2.6, BED['pin_z'])).circle(1.5).extrude(6.5), 0.3, 'bed',
            'Spring-loaded pogo contacts (5 mm travel) that meet the tail pads.', 'bedpins', label=(sy > 0), color='#d9a441')
    add('bed_ir', 'IR beacon LED', cq.Workplane('YZ', origin=(bx + 0.8, 0, 22)).circle(2.5).extrude(4.5), 0.2, 'bed', '940 nm LED, coded so Nibbo can tell its bed from a TV remote.', None, label=False, color='#3a1d4d')
    return E
