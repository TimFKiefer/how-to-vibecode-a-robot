# Takt: a six-legged walking robot

Source location: https://github.com/TimFKiefer/how-to-vibecode-a-robot

> **The parts list and the model differ.** The parts list ([`BOM.md`](BOM.md)) moved to the 12 V Feetech STS3215
> (ST-3215-C018), a 3S 11.1 V 1000 mAh pack with an XT60 plug and the Waveshare Bus Servo Adapter (A). The model,
> its keep-out boxes and the load percentages below are still for the 7.4 V STS3215 run at 6 V and a 2S pack. They
> need a CAD change.

Nothing has been printed or tested on a real servo yet. Every number below comes from the CAD script
[`cad/takt.py`](../cad/takt.py), its [`cad/report.json`](../cad/report.json) and a height study whose 120 mm results
are in [`cad/blade120_report.json`](../cad/blade120_report.json) (FreeCAD runs of 2026-11-09). All of them are
**UNVERIFIED: model only**. All loads are static, so they cover gravity only.

**The legs end in a 120 mm blade.** A first design had a 150 mm blade; a study of leg variants and body heights
favoured the shorter one. Where the text below compares with "the 150 mm design", it means that first design.

## What it is
- **Body:** a hex puck with a flat front. The lower part is a hex prism with rounded vertical edges. On top sits a
  chamfered hex lid with a honeycomb speaker grille (217 holes, 3.2 mm, on a 5.8 mm pitch). The front face has a
  smoked window with two white pill eyes on an OLED. Below the window are a camera hole and a status LED.
- **Legs:** six, one at each vertex, all identical. Each leg has three STS3215 servos.
  - **Coxa:** the servo sits in the body and turns a U bracket.
  - **Femur:** a straight link from horn to horn.
  - **Knee:** the knee servo sits inside a frame at the top of the tibia, with a window that shows the red label.
    Below the frame hangs a 120 mm sickle blade (the 150 mm design's curve, scaled) with a TPU claw tip.
- **Belly height is a software setting.** The body rises or sinks on feet planted at one radius. Three named
  heights are below; "Normal" is the default stand.
- **Standing silhouette at Normal:** the femurs rise at 12.5°. The knee axes sit 21 mm below the lid top, and the
  knee frames reach 20 mm above it. In the 150 mm design they stood 12 and 53 mm above it.

| Main dimension | Value (UNVERIFIED) |
|---|---|
| Body | hex, 154 mm across corners, 133 mm across flats, 79 mm tall (base 61 + lid 18) |
| Stand at Normal (envelope) | 412 x 377 mm footprint, 161 mm tall; belly 62 mm above the floor |
| Feet in the stand | on a 284 mm circle (35 mm outside the femur axes); walking stance on a 314 mm circle (50 mm outside) |
| Leg | hip radius 74, coxa 33.2, femur 66, knee to tip 120 mm |
| Mass | **2.05 kg**: printed 659 g, 18 servos 1080 g, electronics 160 g, wiring and screws 148 g |

## Three heights
From the height study, with this exact leg. Each height has its own walking radius, lift and outward
swing (`NAMED_HEIGHTS` in `takt.py`). Step h is the tallest box (40 mm deep) one foot can step onto from the
walking stance. All numbers UNVERIFIED.

| Name | Belly | Knee top vs lid | Walking feet | Lift | Worst walk frame | Push to tip (stand tripod / walk) | Step h |
|---|---|---|---|---|---|---|---|
| **Spinne** (the robot's `low_stand`) | 30 mm | +56 mm | 40 mm out | 25 mm + 45 mm out | 62 % | 0.90 / 0.67 g | 21 mm |
| **Normal** (the robot's `stand` and gait) | 62 mm | +20 mm | 50 mm out | 50 mm straight | 77 % | 0.72 / 0.58 g | 50 mm |
| **Hoch** (data only, no pose of its own) | 95 mm | −13 mm | 50 mm out | 50 mm straight | 73 % | 0.58 / 0.47 g | 69 mm |

- **Switching heights:** raise or lower on six planted feet. That is checked every 5 mm from 30 to 100 mm, worst
  six-foot load 28 %. Then move the feet to the height's walking radius, one tripod at a time on a short arc
  (checked at every height).
- **Not scripted:** takt.py builds the three heights, but the moves between them are not scripted here.

## Servo in the model: Feetech STS3215, 7.4 V version
Source: Feetech product specification STS3215, edition A/0, 2020-04-10 (the machine-translated copy hosted by Core
Electronics: `core-electronics.com.au/attachments/uploads/sts3215-smart-servo-datasheet-translated.pdf`). The
model follows its section 9 drawing and its section 10 horn drawing.
- Case 45.23 x 24.73 x 35 mm, 55 +-1 g. 25T spline, 5.9 mm. Rear dummy shaft 6 mm, 4.1 mm proud. Two aluminium
  horns: 19.95 mm, 4 x M3 tapped on a 14 mm circle. Two 5264 3-pin sockets on the back face, beside the tail.
- 6 V: stall 16.5 kg.cm, rated 4 kg.cm. 7.4 V: stall 19.5 kg.cm, rated 5 kg.cm. 360° magnetic encoder with no
  mechanical stop. The output switches off after more than 80 % of stall for 2 s.
- **Voltage:** the spec gives a 4-7.4 V input, and the protection trips above 7.4 V. A full 2S pack is 8.4 V, so
  a **regulator is required**. The servos run at 6 V, so **all % below are against the 6 V
  figures**; the 7.4 V percentages are also in `cad/report.json`.
- The parts list buys the 12 V ST-3215-C018 instead (see the note at the top). Check the SKU when buying: the
  variants, such as C044 and C046, have different gearing and torque.

## Electronics (keep-out boxes and mounts in the model)
| Part | Size (mm) | Where | Source of size |
|---|---|---|---|
| Raspberry Pi Zero 2 W | 65 x 30 | hangs from 4 lid bosses | standard Zero outline |
| Feetech FE-URT-1 (or -2, same outline) bus board | 55.8 x 36.6 x 11.5, holes 48.8 x 28.8 | printed deck over the battery | retailer listings |
| 2S LiPo, Gens ace 1300 mAh 25C | 69 x 35 x 17, 69-86 g | floor; drops out through the belly hatch | retailer listings |
| Pololu D24V150F6, 6 V 15 A (servo power) | 43.2 x 31.8 x 11, 19 g, input 7.2-40 V | under the lid, below the grille | pololu.com/product/2882 |
| 5 V regulator for the Pi, D24V22F5 class | 17.8 x 17.8 | taped inside the back wall | not checked |
| Pi Camera Module 3 | 25 x 24, about 11.5 deep | 4 standoffs behind the 8 mm camera hole | retailer listing + RPi brief |
| 1.3" SH1106 128x64 I2C OLED (the eyes) | board 35.4 x 33.5, active area 29.42 x 14.7 | slides into rails behind the 2 mm smoked acrylic | vendor listing |
| Speaker, Adafruit 1890 | 28 x 4.5, 6 g | ring under the grille | retailer listing |
| MAX98357A amplifier, Adafruit 3006 | 19.4 x 17.8 x 3 | under the lid, taped | Adafruit |
| SPH0645 I2S mic, Adafruit 3421 | 16.7 x 12.7 x 1.8 | under the grille, taped | Adafruit |

The D24V150F6 regulates while the pack is above about 7.2 V. Below that, the servo voltage sags with the pack.
That is still inside the 4 V lower limit.

## Parts list
**Printed** ([`cad/parts/`](../cad/parts/), already turned to their print orientation; STL and STEP). PETG unless
noted, at a 75 % density factor. Masses and bed sizes are model numbers (UNVERIFIED).

| Part | Count | Bed (mm) | g each | Supports |
|---|---:|---|---:|---|
| base | 1 | 146 x 146 x 61 | 139 | yes: the six coxa servo socket covers (26 mm bridges) and the slots for the coxa plates |
| lid | 1 | 146 x 146 x 21 | 44 | top down; walls lean 36°, none needed |
| deck | 1 | 70 x 68 x 2 | 7 | none |
| hatch | 1 | 80 x 54 x 2 | 8 | none |
| coxa (U bracket on S1 + cradle for S2) | 6 | 61 x 41 x 45 | 18 | under the top plate |
| femur (link) | 6 | 90 x 45 x 24 | 18 | small, on the rib edge |
| knee (tibia frame around S3) | 6 | 43 x 41 x 72 | 18 | a little, inside the frame |
| blade (sickle, 120 mm) | 6 | 74 x 72 x 16 | 21 | none, flat |
| foot (claw tip), **TPU** | 6 | 19 x 16 x 21 | 1.8 | none |
| foot_sneaker (peg for a bought mini sneaker), instead of foot | 6 | 27 x 27 x 16 | 3 | small |
| fit_gauge, fit_collar | 1 each | print first | | none |

The overhang areas the script measured are in `cad/report.json` under `print` (base 54 cm², the rest below 8 cm²).
They count bridges too. Filament: about **650 g PETG (212 m of 1.75 mm) plus 11 g TPU**, before supports.

**Hardware:** 18 x STS3215 plus 2 spare. 144 x M3x6 for the horns, 8 per joint. 72 x M2 self-tapping screws
(PA2.0) for the case holes: M2x6 at S1, M2x8 at S2 and S3. 12 x M3x12 countersunk screws and 12 x M3 heat-set
inserts for the blades. 6 x M3x16 with nyloc nuts for the feet. 4 x M3x20 and 4 inserts for the lid. 4 x M3x6
countersunk and 4 inserts for the hatch. M2.5 screws and inserts for the deck and the Pi. One 34 x 28 x 2 mm
smoked acrylic window. Rubber or TPU feet are already in the design, because bare PETG tips slip.

**Assembly order per leg:** first the coxa bracket onto S1's horns, sliding in from outside. Then S2 into the coxa
cradle from above. Then the femur onto S2's horns, sliding along the femur. Then bolt the blade to the knee frame,
put S3 into the knee frame from the open inboard side, and slide the femur onto S3's horns. Every U bracket slides
on sideways because the horns go onto the servo first.

## Checks (all in `cad/report.json`)
| Check | Result |
|---|---|
| Parts valid, one solid each, watertight STL | **pass**, all 12 printed parts (and the servo, window, sneaker foot) |
| Static fit (screwed-together parts overlap?) | **pass**, 0 overlaps |
| Collisions, > 0.5 mm³, 8 poses + 48 gait frames | **pass**, 0 hits |
| Feet on the floor | **pass**: standing feet within 0.1 mm (sneakers 0.03 mm), nothing below the floor |
| Joint ranges, each swept with the others neutral | **pass**: coxa ±40° from its case, femur -20…100°, knee 70…155°; every pose and gait frame inside them |
| What sets those ranges (sweep) | coxa plates hit the hull past 40°; femur hits the lid at 110°; knee 65° femur meets the frame; knee 160° blade hits the coxa and S2 |
| Gait IK, 24 frames each, walk 50 mm per cycle and turn 16° per cycle | **pass**, every frame reachable and inside the ranges |
| Every printed part fits the 180 mm bed | **pass**; the largest is the base at 146 x 146 x 61 |

**Servo loads** (6 V: rated 4.0, stall 16.5 kg.cm; UNVERIFIED). The worst femur and worst knee over all legs,
including the legs in the air:

| Case | Femur kg.cm (rated / stall) | Knee kg.cm (rated / stall) |
|---|---|---|
| Stand (Normal), six feet | 0.44 (11 % / 3 %) | 1.04 (26 % / 6 %) |
| Stand (Normal), one tripod | 1.64 (41 % / 10 %) | 2.06 (52 % / 12 %) |
| Walking, worst of 24 frames | 2.91 (73 % / 18 %) | 3.08 (77 % / 19 %) |
| Turning, worst of 24 frames | 2.77 (69 % / 17 %) | 0.99 (25 % / 6 %) |
| Wave (five feet) | 1.35 (34 %) | 1.43 (36 %) |
| Look up, one tripod | 1.74 (44 %) | 2.30 (57 %) |
| Getting up from sit, six feet | 3.18 (80 % / 19 %) | 2.34 (58 % / 14 %) |
| Spinne (low stand), one tripod | 1.87 (47 % / 11 %) | 0.63 (16 % / 4 %) |

**Balance** (UNVERIFIED). Margin is the distance from the centre of mass to the nearest support edge. Push to tip
is that margin divided by the centre-of-mass height, in g.

| Case | Margin | Push to tip |
|---|---|---|
| Stand, six feet | 123 mm | 1.24 g |
| Stand, one tripod | 71 mm | 0.72 g |
| Walking, worst frame | 57 mm | 0.58 g |
| Turning, worst frame | 78 mm | 0.73 g |
| Wave | 80 mm | 0.79 g |
| Spinne, one tripod | 71 mm | 0.90 g |
| Sneakers, one tripod | 71 mm | 0.56 g |
| Sit | belly plate carries it; centre of mass 26 mm inside the plate edge | |

## What the numbers mean
- **Torque is fine for standing, walking and turning on flat ground:** the worst joint is the knee at 77 % of the
  rated 6 V torque while walking. That is higher than the 150 mm design's 67 %: the knee now stands further out
  than the foot. Real stepping adds acceleration and friction, and none of that is modelled. The margin to the
  80 % stall cut-out (13.2 kg.cm) is about 4x the static peaks.
- **The low stand is now a real stance.** Spinne at 30 mm loads a tripod to 47 % and is the steadiest stance.
  The 150 mm design's 35 mm low stand reached 118 %. Getting up from sit is a short push at 80 % of rated on six
  legs: fine as a burst, not something to hold.
- **The foot lifts straight at Normal.** The knee stands at 117° of the 155° it may fold to, so the gait lifts
  each foot 50 mm without an outward swing. The box test gives 50 mm at Normal, against 21 mm for the 150 mm
  design, and 69 mm at Hoch. At Spinne the knee is folded to 143°, so the foot swings 45 mm out while it lifts.
- **The walk uses wider feet than the stand** (50 mm instead of 35 mm outside the femur axis): from the stand
  spots the knee carries most of the load at the ends of the stride. The height study checked the foot shuffle
  between the two, but takt.py does not script it.
- **Look up is 10° nose up, not 12°.** At 12° the front femurs reach −24°, past the −20° range end.

## Design decisions
- **Hex orientation:** flat front for the face, legs at the six vertices, all 60° apart. Inside the body, the four
  corner coxa servos are turned 15° away from the face. Without that, their tails would fill the space behind the
  window. The legs still stand straight out from the vertices.
- **Knee servo in the tibia frame.** Its tail leans 20° outboard. This gives the knee its range, and
  it keeps the frame clear of the femur when the knee is folded.
- **Blade length:** 120 mm, not 150. The leg variant and height studies show it reaches every
  step height of the 150 mm blade about 30 mm lower, with 26 to 51 % more push to tip and the same loads.
- **Face:** the window shows only the eyes. The 8 mm camera hole sits below it, not behind the smoked window.
- **Blade slot:** the slot goes through the side of the blade, not its front face, because the blade prints flat
  on that side and a long horizontal slot would need support.
- **Sneaker variant:** the same blade with a straight peg foot instead of the claw. The script refits the peg to
  the 120 mm tip (peg at −27.3°, sole contact at 136.9 / −38.5 mm in the tibia frame). The body stands 32 mm
  higher (94.0 mm) because the sneaker makes the leg longer. Only that stand is checked. The sneaker in the render
  is a generic red and white stand-in without logos.

## UNVERIFIED (measure at the fit test before printing the robot)
Print `fit_gauge` and `fit_collar` with two servos first. Fix the parameters at the top of `takt.py`, then
run the script again.
1. `S_NOSE` 10.2 mm (shaft axis to the nose end). Read from the drawing raster.
2. `S_STEP` -5.5 mm, where the 29 mm nose block meets the 32 mm tail block. The tail block split (-1.5 / +30.5).
   The lip and the bump that make the 35 mm overall height (`S_LIP`, `S_BUMP`). All raster readings.
3. `HORN_FACE` 33.5 and `IDLER_FACE` -3.7: where the two horns sit. The drawing does not show this. The gauge
   tests it: one servo with both horns must slide snugly between its plates.
4. `HOLE_TOP` / `HOLE_BOT`: the PA2.0 case holes (raster). The brackets only use the tail holes, -28.8 on the
   output side and -32.7 on the back, at ±10.4. The collar tests them.
5. `PLUG` depth 7 mm, for the plug plus the cable bend. Several plates are narrow near the plugs because of it.
6. That the idler horn turns freely on the rear dummy shaft.
7. Servo mass with horns, 60 g (55 g datasheet plus an estimate for the horns).
8. OLED module depth (5 mm) and where its active area sits. 1.3" boards differ: Waveshare's 40.5 x 37.5 mm
   version does **not** fit.
9. Camera Module 3 depth (11.5 mm), lens position on the board, and the 21 x 12.5 mm hole pattern.
10. Pololu D24V150F6 hole positions (`REG_HOLES`).
11. Masses not taken from a listing: FE-URT-1 15 g, Pi Zero 2 W 11 g, OLED 8 g, 5 V regulator 3 g, a mini
    sneaker 10 g. Battery 86 g; listings say 69-86 g.
12. Heat-set insert hole sizes (`M3_INSERT`, `M25_INSERT`) depend on the brand. `FIT` 0.25 mm per side.
13. The height numbers (Spinne / Normal / Hoch, step heights, transitions) are FreeCAD statics from
    the height study, like everything else here.

## Not designed yet
- **Cables:** the loops are drawn but not routed or collision-checked, and there is no strain relief. The servo
  leads are 15 cm. Each leg chains S3 to S2 to S1 to the body.
- **Bus hub:** no 6 V servo distribution board is chosen or placed.
- **Power switch:** the slot in the back fits a small switch only. Servo current (several amps, up to 15 A at the
  regulator) needs a heavier switch or unplugging the battery.
- **Charging:** the battery comes out through the belly hatch. There is no charge port.
- **Fasteners for the amplifier, mic and 5 V regulator:** tape for now.
- **Thermal:** the regulator under the lid is not checked for heat.
- **Strength:** there is no strength analysis. Thin spots to watch: the 16 mm neck of each coxa's lower plate,
  the 12 mm idler-side femur plate near the knee, and the blade around its slot.
- **Firmware:** the gait controller, calibration, height switching (raise or lower on planted feet, then the foot
  shuffle), and the moves between poses (stand to walking stance, stand to sit).
- **Runtime:** a rough estimate only, from datasheet Kt (8 kg.cm/A). About 13-20 min walking and about 50 min
  standing on the 1300 mAh pack. It comes from the 150 mm design and was not redone. Not measured.
- **Wiring space:** inside the base it is tight around the camera and the OLED.

## Renders ([`cad/renders/`](../cad/renders/))
- `stand_front34.png`: stand from the front, three-quarter view.
- `face.png`: close-up of the face. `side.png`: the silhouette. `top.png`: the grille.
- `sit.png`, `low_stand.png` (Spinne, 30 mm), `look_up.png`, `look_around.png`, `wave.png`, `wave_2.png`.
- Gait: `walk_00/03/06/09.png` (frames 0, 3, 6, 9 of 24, an eighth of a cycle apart; combined in
  `gait_walk_sheet.png`) and `turn_03.png`.
- `sneaker.png`: the sneaker variant.

The renders are flat-shaded z-buffer images, not photoreal. Labels are red stickers. The cables are drawn loops.

## Files
- [`cad/takt.py`](../cad/takt.py): the model, poses, gait, checks and export. Run it from the repository root with
  `nice -n 15 freecadcmd cad/takt.py`; `STAGE=parts` stops after the parts. It writes `robot2.log`,
  `report.json`, `parts/` and `meshes/` next to itself (or into `TAKT_OUT`).
- [`cad/render.py`](../cad/render.py): renders from `cad/meshes/` with numpy and Pillow (FreeCAD's python). It writes
  `cad/renders/*.png`.
- [`cad/viewer/`](../cad/viewer/): the data for the 3D view, its FreeCAD export (`export_viewer.py`) and its
  self-test (`selftest.mjs`).
