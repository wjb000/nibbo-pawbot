# Actuator sizing: why MG92B-class servos

Every joint uses the same servo: an MG92B-class metal-gear micro servo, rated 3.5 kg·cm (0.343 N·m) stall at 6 V and 3.1 kg·cm (0.304 N·m) at 5 V, with 0.08 s/60° no-load speed and a mass of 13.8 g ([Adafruit datasheet listing](https://www.adafruit.com/product/2307)). The board runs the servos from a 6 V buck so they get their full rated torque.

## Inputs (from the CAD, `cad/reports/mass_properties.json`)

| | |
|---|---|
| Robot mass | 0.49 kg from the CAD. The calculation uses **0.50 kg**, so W = 4.91 N |
| Thigh L1 / shin L2 | 45 mm / 50 mm |
| Hip height, nominal stand | 88 mm, with the foot ball centre 80.5 mm below the hip |
| Knee horizontal offset in stand | 26 mm, so the shin's moment arm about the knee is about 26 mm |
| Knee drive | Body-mounted servo, 12 mm horn, pushrod, 12 mm shin crank, about 1:1 |

The knee servo lives in the body and drives the shin through a pushrod. This keeps the legs light (31 g per leg including the foot) and lets all 8 servos share one mounting design. Because the linkage is close to a parallelogram, the approximations are: hip torque ≈ F·(x_foot − x_hip) and knee torque ≈ F·(x_foot − x_knee).

## Load cases

| Case | Feet loaded | Foot force F | Lever | Joint torque | % of 0.343 N·m stall |
|---|---|---|---|---|---|
| Standing still | 4 | 1.23 N | knee 26 mm | **0.032 N·m** | 9 % |
| Trot (diagonal pairs), 1.8× dynamic peak | 2 | 4.42 N | knee 26 mm | **0.115 N·m** | 34 % |
| Trot, hip with ±15 mm foot excursion | 2 | 4.42 N | hip 15 mm | 0.066 N·m | 19 % |
| Stand up from lying, 1.2× accel | 4 | 1.47 N | knee 40 mm | 0.059 N·m | 17 % |
| Paw wave / "high five" on 3 legs | front support leg ~45 % W | 2.21 N | knee 30 mm | 0.066 N·m | 19 % |
| Sit (rear legs carry 70 %) | 2 rear | 1.72 N | knee 40 mm | 0.069 N·m | 20 % |
| Self-righting, 2 legs push together | 2 | 2.45 N each | 43 mm | 0.105 N·m | 31 % |
| Self-righting, worst case 1 leg at full reach | 1 | 2.45 N | 85 mm | **0.21 N·m** | 61 % (short burst) |

**Design rule:** continuous loads stay at or under about 35 % of stall (for heat and speed) and short peaks under about 65 %. Every case passes, and trot is the tightest continuous case at 34 %.

**Speed check:** at a 1.4 Hz trot, the hip swings ±16° in about 0.18 s, which needs about 3.1 rad/s. The servo does 13 rad/s with no load and roughly 8 rad/s at 34 % load. That leaves 2.5× margin.

## Why not the cheaper MG90S?

An MG90S-class servo (2.2 kg·cm, about 0.216 N·m) costs about $0.76 at 10k ([Alibaba listing](https://www.alibaba.co.th/product-detail/OEM-Factory-MG90S-Steering-Gear-9g_1601709710911.html)). On the knees it would run the trot at 53 % of stall continuously. That means heat, sag and gear wear, plus a gait that feels sluggish and "old". It does pass on the hips (31 %).

**Cost-down path after testing:** put MG90S-class servos on the 4 hips. That saves about $8.50 per robot, at the cost of a second part number.

## Power

- Walking draws about 8 × 0.25 A at 6 V ≈ 12 W, plus about 1.5 W for electronics.
- The pack is 2S 18650, 18 Wh. At about 85 % efficiency that gives about **1.1 h** of continuous walking.
- A realistic pet day (mostly sitting, watching and reacting) gives about **6–8 h**, then it walks itself to bed.

## Angle feedback

The production servo is ordered with a 4th wire on the potentiometer wiper. A standard factory option is assumed here and needs to be confirmed in the quote. The ESP32-S3 ADC reads the real joint angle, which gives:

- fall and pick-up detection,
- "go limp" when you move its legs by hand,
- data for sim-to-real system ID.
