# Production notes (DFM + assembly)

## Molded parts

| Part | Qty / robot | Material | Process | Notes |
|---|---|---|---|---|
| Top shell | 1 | PC/ABS, gloss white | Injection, 1-cavity | Ears molded in. 1° draft. Mic port. Touch foil is applied inside. 4 screw bosses. |
| Chassis | 1 | PC/ABS, graphite | Injection, 1-cavity, 2 side actions (tail socket, cliff windows) | Has four servo towers, battery cradle, PCB standoffs, speaker grille and tail socket. Every bought part mounts here. |
| Visor | 1 | PC, smoked | Injection | IR-transparent ink patch over the ToF. Gloss SPI-A2 finish. |
| Tail | 1 | TPU 85A | Overmold on the 2-wire harness + contact pads | Doubles as the charging connector. |
| Thigh | 4 | PA6-GF30 | 4-cavity family tool | Symmetric, so the same part fits all 4 legs. |
| Shin + foot | 4 | PA6-GF30 + TPU 85A | 2-shot | Foot ball overmolded. Symmetric. |
| Pushrod | 4 | PA6-GF30 | Shares the thigh family tool | |
| Bed | 1 | ABS | Injection | Cored out in production (the CAD shows a solid floor). Has a snap cover over the board pocket. |

That is 16 pieces from **7 robot tools** plus 1 bed tool. Tooling is estimated at $55–80k for P20 steel at about 100k-shot life, with a T1 lead time of about 6–8 weeks (estimate; quote needed).

Shell rules:

- 2.0 mm walls in the CAD (2.0 for the FDM prototype, 1.8 mm for molding).
- Ribs at 60 % of wall thickness.
- Boss OD at 2.2× the hole.
- One straight-pull parting line at the z = 90 mm split plane.
- No undercuts on the top shell.

## Electronics

- **Main board:** one 4-layer board, 42 × 56 mm, all SMT on one side. The ESP32-S3-WROOM-1 is a pre-certified module, so only intentional-radiator verification and unintentional emissions testing are needed. FCC/CE testing is estimated at $15–25k.
- **Ports:** none. Charging goes through the tail. Firmware updates go over Wi-Fi, with A/B partitions. Factory flash and test run on bed-of-nails pads.
- **Harness:** 8 servo leads (4-wire). Display and camera use FPCs. The battery uses a JST-PH. The tail plug has 2 wires.
- **Battery:** 2S 18650 pack with the protection IC on the main board. The pack needs UN38.3 and IEC 62133. Buy a certified pack from the cell vendor.

## Assembly sequence (target about 12 min including test)

1. **Servo towers.** Press 8 servos into the chassis towers (the tabs drop into slots) and add 16 × M2×6 self-tappers. Hip servos go below and knee servos above.
2. **Belly.** Drop the 2S pack into the cradle. Snap in the speaker, the 2 cliff sensors and the tail plug (it clicks into the socket).
3. **Board.** Fit the main board onto 4 standoffs with 2 screws and 2 snaps. Plug in the 8 servo leads, battery, display FPC and camera FPC.
4. **Face.** Seat the LCD, camera and ToF in the visor frame, then clip the visor into the chassis.
5. **Close.** Fit the top shell (touch foil pre-applied, mic gasket in place) with 4 × M2×8 screws from below.
6. **Legs.** Put the robot on the zeroing jig so the servos are commanded to 90°. Fit the hip horn, then the thigh (4 screws). Fit the knee horn, the pushrod (2 shoulder pins) and the shin (knee pin).
7. **End-of-line test (automated):** IMU calibration, stand, walk 1 m on the test mat, camera frame check, audio loopback, touch check, dock-and-charge check on a bed, and a Wi-Fi RSSI check.
8. **Pack.** The box holds the robot, bed, USB-C cable and quick-start card.

## Prototype (FDM), build one now

- **Print** `cad/stl/*.stl`:
  - top shell, chassis, visor, thighs, shins and pushrods in PETG;
  - tail and feet in TPU 95A (the feet are separate balls glued onto the shin for the prototype; `cad/step/foot.step`);
  - visor in clear or smoked PETG.
- **Orientation:** the STLs are already oriented. Thigh, shin and pushrod lie flat. Print the chassis upright with tree supports under the belly curve.
- **Settings:** 0.2 mm layers, 3 perimeters, 15 % gyroid. Total is about 300 g PETG and 40 g TPU, roughly 14 h on a Bambu-class printer (estimate).
- **Electronics:** for the prototype electronics, see the Prototype BOM in `BOM.md`. A XIAO ESP32S3 Sense stands in for the custom board.
