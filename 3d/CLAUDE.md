# CLAUDE.md

Parametric CAD for a 12-DOF printed quadruped built around 12 × Feetech/Waveshare ST3215
bus servos - the dachshund: a 220 mm body on 75 / 55 mm legs. Pure code-CAD — no GUI model, no feature history outside the scripts.
`README.md` is the human-facing spec; read it before changing geometry.

## Environment

Always use the project venv — CadQuery, gmsh, sfepy and VTK are installed there only:

```bash
.venv/bin/python mini_dog.py     # rebuild → out/step, out/stl, out/mini_dog_assembly.step, out/bom.json
.venv/bin/python render.py       # → out/view_{iso,front,side}.png
.venv/bin/python fea.py --all    # strength, every part, every load case
.venv/bin/python export_sim.py --check   # → out/sim/{mini_dog.urdf,mini_dog.xml,meshes/}
```

Run everything from `3d/`; scripts use paths relative to the CWD (`ref/…`) or to their own
location (`out/…`). A `mini_dog.py` run takes a couple of minutes — the ROM scan is swept
boolean interference against real solids. Do not add `--fast` paths that fake it.

## Layout

| path | role |
|---|---|
| `mini_dog.py` | **the model.** All geometry, all parameters, the ROM scan, the clearance probes, the exporters. |
| `fea.py` | linear static FEA (gmsh + sfepy). Imports `mini_dog`; derives load cases and clamp regions from its joint frames. |
| `export_sim.py` | URDF + MJCF export. Imports `mini_dog`; every length, mass and limit is read from it. |
| `camera.py` | the IMX415 **as a sensor**: the site and `<camera>` both sim exporters emit. No CAD in it. |
| `lidar.py` | the Unitree L2's scan model. The robot carries no LiDAR and no exporter imports this; it reads `LIDAR_*` constants `mini_dog.py` no longer has. |
| `terrain.py` | procedural heightfield + the ramp/wall/log course, imported by both sim exporters. |
| `render.py` | offscreen VTK renders of `out/stl/*.stl`. |
| `bench_rig.py`, `torque_rig.py` | printed bench fixtures. One-way consumers of `mini_dog` (sleeve, clamp, hub pattern, densities); **not robot parts** — no `PARTS` entry, no mass, no FEA or sim consumer, so steps 4–6 below do not apply. Keep the arrow one-way. |
| `tools/` | one-off measurement/diagnostic scripts (`tools/README.md`). Not part of the build. |
| `ref/` | vendor downloads and transcribed measurements of bought parts. Read-only inputs. |
| `out/` | **generated — never edit by hand, never treat as source.** |
| `../ros2/smalldog_description/scripts/generate_model.py` | the *second* exporter of this CAD. Not optional — step 6. |
| `mini_dog_codex_handoff.md` | the original brief and its hard constraints. |

## How to change the model

Every dimension lives in the constant block at the top of `mini_dog.py`. Change it there
and re-run — never patch a part function with a literal, never edit `out/`. Part functions
return a CadQuery `Workplane` in robot coordinates; `PARTS` maps name → (workplane, qty,
note) and drives export and BOM; `PRINT_ORIENT` maps name → (axis, angle), applied to the
STL only.

## Invariants

**Source and measurement**

- **Units are mm** everywhere: geometry, FEA (N, MPa), exports (which scale to SI).
- **The servo interface is measured, not assumed** (`S_*`/`HUB_*`, from the vendor STEP via
  `tools/measure*.py`). If one looks wrong, re-measure. **A real part beats the STEP**:
  `HUB_BOLT_D` is M3 on the hub that arrived, ⌀2.5 in the STEP. Mark such rows in the
  README's source column.
- **Masses, densities, the MuJoCo joint feel (`MJ_*`), the foot contact (`MJ_FOOT_*`), the
  servo's stall torque (the 2 s peak, 3.2 — the held 2.3 is a runtime budget) and no-load
  speed, the stand pose (`STAND_*`) and the IMU site all live
  once, in `mini_dog.py` sections 3–4.** `fea.py`, `export_sim.py` and the ROS 2 generator
  read them and keep no copies. Every one of these was duplicated once and diverged (servo
  mass 55 vs 60 g; rounded `J_EFF`; IMU sites 25 mm apart; joint limits narrower by 52° in
  one exporter; the stand pose −22°/46° here against −24°/60° in the ROS 2 generator). `rl/checks/check_model.py` is what catches it.
- `MJ_DAMPING` / `MJ_FRICTIONLOSS` / `MJ_ARMATURE` / `MJ_KP` are measured on one real
  ST3215 (provenance beside each constant). `MJ_DAMPING` and `SERVO_STALL_NM` are in one
  torque calibration — the scale's — because the joint ceiling divides one by the other;
  the ladder's 1.37 is the same measurement in the ladder's (× 0.596/0.400). `MJ_ARMATURE` = 0.0165 is deliberately not
  `rl/actuator.py`'s `Params.J_m` default of 0.008 — that is a flagged vendor prior and
  `rl/params/st3215.json` carries the fit.
- `MJ_FOOT_CONDIM` = 4, not 6: condim 6 triples the terrain spread (measured; it is the
  dimension, not its coefficient). Torsion is derived from the real TPU patch.
  `MJ_FOOT_PRIORITY` = 1 makes the foot's numbers win over the floor's — MuJoCo otherwise
  takes the elementwise max of the pair. Check a contact model by reading
  `d.contact[i].friction`, never the XML.
- The Waveshare `ROBOTIC DOG.step` supplied the shin's **shape** and nothing else — it is
  a single-shear-horn aluminium design. At `L_SHIN` = 55 that shape is down to fork,
  shoulder and ankle boss.

**Legs**

- **`L_THIGH` = 75 is a floor, not a choice.** The knee servo sits inline in the thigh,
  connector towards the hip, and `servo_envelope()` keeps 6 mm of plug and 14 mm of lead
  clear behind it. At 75 that keep-out exactly clears the hip bracket's pitch sleeve; at
  72 it cuts 488 mm³ out of the sleeve's end wall and thrust lug, at 66 1633. Nothing
  flags it: the cut hip bracket is still one valid solid. A shorter thigh is a different
  knee architecture.
- **The shin is solid** (`SHIN_CAV = None`, `SHIN_TIE_U = ()`): the fork spine owns
  u ≤ 31 whatever the length, the ankle boss is at u = `L_SHIN − FOOT_D/2` = 42, and the
  foot bolt's bore runs up to u = 32.6 through all of the 11 mm between.
- **The low stance is the stand pose**, `STAND_PITCH` / `STAND_KNEE` = −30° / 73°: foot
  under the hip, base at 148 mm. Both exporters read it from here.

**Structure**

- **The ST3215 case has no threaded side holes.** The sleeve+fork architecture exists
  because of that. Nothing loads the servo through a printed thread or a single-shear horn.
- **No metal parts, no machining, no external bearings.** FDM only plus the stock aluminium
  hubs (handoff doc).
- **Nothing in a torque path threads into plastic.** Every load-bearing screw not going
  into the stock aluminium lands in a nut in a `nut_slot()`; the slot's `ang` must point at
  a face still reachable at that moment of the assembly order — it is what fixes the order
  in `README.md`. Hub screws thread into the tapped aluminium plates (no room for a nut).
- **The cradle → tray screws are the one exception: heat-set inserts** (`INS_*`), from the
  flange side through the register spigot. A nut has nowhere to go round that axis — the
  side channel it had was a sealed cavity, a pocket open to the servo bore hollows the
  boss and halves `cradle_front`'s SF, and the servo side has 3.7 mm under the strap.
  `cradle_insert_clear()` checks the bore, the lip it bottoms on and the screw's reach.
- **The camera mount stands on the cradle and is bolted to the deck**: two tabs
  (`cam_tab()`) on the deck's top, each on its own M3 × 12 flat head from under the deck
  (`CAM_BOLT`, `cam_bolt_check()`), nut on the tab. Not into the cradle: its old M3s cut
  the cradle's 4 mm flange rib in two.
- **Off the torque path a screw may thread into plastic** (lids, covers): hole
  `M3_TAP`/`M25_TAP` (⌀2.6/⌀2.1, radii like the `_CLR` pair, both **UNVERIFIED** — print a
  coupon first), ≥ 1 D of wall, 2 D engaged, one-assembly thread. Unsure whether a load
  path runs through it → nut.
- **Do not slit the sleeve.** Its −x end wall is the tube's only crossing of y = 0.
  Measured: `thigh_A` inter-layer SF at stall 1.1 → 0.6, deflection 2.65 → 12.2 mm. The
  clamp that survives is the thrust clamp (`THRUST_*`).
- **Nothing goes into 23 < r < 34 of a joint axis over the sleeve's length** — the distal
  fork's spine sweeps it. **Screws count**: an M3 × 10 cap head reaches r = 24.2 and bound
  the first leg; the fastener is headless (`THRUST_HEAD_*`). Hub screw heads were the same
  defect: 1.65 mm ISO 7380 heads in 0.6 mm of air. `thrust_bolts()` and `fork_screws()`
  are in every `rom_scan`; `thrust_clear()` / `head_clear()` print margins every run.
- **The servo envelope must not be cut from the link that bolts to that servo's hubs** —
  `servo_envelope(hub=False)` / `env_all(no_hub=...)`. Sweeping the hub discs out is how
  all three forks once lost their bolt circles.
- **The hip-roll cradles are bolted parts** (`cradle_front` / `cradle_rear`, one per end
  because the halves' rails overlap on the centreline): four M3 through `CRADLE_REG_D`
  spigots into pockets in the tray wall. The register is stiffness, not decoration —
  0.1 mm of slop at the flange is 0.5 mm at the ground. Nut bosses run to `CRADLE_BOSS_X`,
  legal only because past the flange the fork's arm disc is the only thing there.
  `STRAP_Y` is a hard ROM limit: full-width straps read `hip_roll +0 .. +0`.
- **`cradle_front` is the weakest printed part** (inter-layer SF **1.54** at `land3g` on
  `--orient`; the crude `SF xy / z` column says 0.9 because it assumes the worst stress
  orientation — read `--orient` for this part). It is structural: the leg's load funnels
  back to the flange through a section capped at `STRAP_Y`, and the fork sweeps the space
  that would widen it. Filling the neck solid, closing it with a web and thickening the
  straps were each tried and each made the peak worse. It prints flange-down (its worst
  direction) on purpose: the only orientation with the servo bores vertical. Its stall case
  is a real couple (`couple=` in `part_specs()`) — a lever force about an axis concentric
  with the load patch divides by zero.
- **The camera slot at the nose is three measured walls, all of them other parts**: floor
  `CAM_LEDGE` = `CRADLE_Z1` (on `cradle_front`), back the chassis front face
  (`CRADLE_X`), front the roll fork's rear arm at `ROLL_X + FORK_Y0`, a disc sweeping
  r ≤ 34. Only the lens passes it. ~1 mm front and back: the fix for a clash is in the
  camera block, not in a part function. The slot follows the body - `CAM_*` are derived
  from `CRADLE_X` and `ROLL_X`, and `ROLL_X − BODY_L/2` must stay 27 (`CRADLE_T`).
- **The battery is a module** (`cell_holder` ×2 combs → welded brick → heatshrink →
  `battery_case` + `battery_lid`, BMS inside), dropped into the tray as one payload. Not
  structure — its lid screws are the one place a screw threads into plastic. The
  combs are clipped flush with the outer cells' tangent planes (no material outboard of any
  cell); `CELL_GAP` is the one number that costs height; the printed web between bores is
  `CELL_GAP − CH_FIT`. `batt_clear()` (lid to deck: 0.60 mm, a foam strip, never
  negative), `holder_clear()` and `module_clear()` exist because payloads are invisible to
  `interference()`.
- **The IMU is inside the Orange Pi's case**, taped face up to its bottom plate at (0, +15),
  in the strip beside the M.2 module; `imu_clear()` measures it against the M.2 band,
  the case's corner spacers and the PCB — all payload geometry. The
  mount is `IMU_*` in
  section 3 via `imu_xyz()`, and both exporters emit the `imu` site from it.
  `rl/checks/imu_placement.py` is the argument for the position — run it whenever the
  mount or the gait moves (it runs on the mac). Moving the site is a re-baseline for `rl/`
  (retrain, not fine-tune).
- **Each end wall has two cable windows, the rear wall the XT30 as well**: the bus window
  inside the cradle flange's frame (the front one was missing — the front frame was a
  sealed box with the roll servos' ports inside it), the low window on the floor
  (`PANEL_LOW`, the 6.64 mm band below the flange; on the centreline, because at |y| > 12
  the passive fork arm sweeps z = −20 at 90° of roll), and the XT30 above the cradle at
  z = +20 (the 9.64 mm band between `CRADLE_Z1` and `BODY_Z1`, so its z is not round).
  Inside the frame, everything but the window is behind a roll servo's case.
  `panel_clear()` probes all five outward every run — the connector against the body
  *and* `servo_hardware()`, because a servo is not a part and two connectors once shipped
  behind one.
- **Servo leads cross a joint on the driven arm's outer face, through the axis**, held by
  two tie slots in `fork()` (`FORK_TIE_*`) on the link side of the axis where the arm is
  the spine web. The shin's fork has none (`ties=False`): nothing crosses the knee.
  `README.md`, "Cables".
- **The power node is two WAGO 221-415 in the rear strip** (`NODE_*`, `node_clear()`),
  and the strip holds exactly that: 17.6 mm deep, 14.8 behind the seats, against a
  lever nut's 18.6 along the wire. The bus adapter and the fuse holder still have no
  keep-out (`POWER.md`, "What it costs the CAD").
- **The buck rides on the Pi's case top** (`BUCK_*`, placed from `OPI_X`), nothing over
  it. Its 18 g came out of `ELECTRONICS_KG`'s allowance. Set on the bench and varnished
  before it goes on — nothing on it is adjusted in place (`ref/power/README.md`).
- **The masses are placed, not left where they fell**: Orange Pi at `OPI_X` = −33 (the IMU
  inside it at the body origin), battery module at `BATT_X` = +15, base link CoM at
  x = +0.3 mm. `generate_model.py` prints it; move one, read the other.

**Checks, and why each exists.** The recurring lesson: `isValid()`, `interference()` (static
body parts only) and `rom_scan` (moving parts only) are each blind to a class of defect.

- `build()` fails a part on `isValid()`, **`Volume() > 0`** and **solid count**
  (`PART_SOLIDS`, default 1; only `servo_gauge` is legitimately 2). A boolean on a
  degenerate contact comes back **inverted** with `isValid()` True (the chassis once
  shipped at −257 mm³ and everything downstream passed, 280 g light); an orphaned piece
  passes both. **Cut a through-path after everything that adds material near it.** Raising
  a `PART_SOLIDS` entry is a design claim — two of its three rows were permitting defects.
- **Fasteners get three separate questions: does the hole reach, does the head clear, can
  a driver get to it.** `foot_bolt_check()` (the foot bolt once opened inside the part and
  was specified 12 mm too short), `thrust_clear()`, `head_clear()`, `fork_access()` (a
  key-sized cylinder on each fork screw's real line, run **until it stands in open air** — a
  probe that ends inside a bore passes on a screw nobody can turn), `cradle_clear()`,
  `cradle_head_clear()`. `FOOT_BOLT_L` follows from `FOOT_CB_Z`, `FOOT_NUT_Z`, `M3_NUT_H`;
  move one, move the BOM line.
- **An access problem that keeps needing cleverer holes means the part boundary is
  wrong.** Two answers to the hip fork screws (four coaxial bores, then two 20° channels
  through the corner, eight holes) were deleted when the cradle became a bolted part and
  the screws ended up in open air. `README.md`, "Reaching the fork screws".
- **Payload gaps get their own probe**: `imu_clear()`, `batt_clear()`, `holder_clear()`,
  `module_clear()`, `panel_clear()`, `camera_clear()`, and in `main()` the Pi case
  (`opi_box()`) against `camera_mount` and its rear standoffs against the deck's end. Every `!! …` line the build prints is a failure like `!! INTERFERENCE`.
- **OCC booleans on the shin lofts fail silently**: a tool crossing a lofted spline end cap
  returns the input unchanged, `isValid()` True, and every later boolean fails the same
  way. Hence the profile interpolation is monotone, and a cavity (if `SHIN_CAV` ever
  returns) stops its slab 0.5 mm short. After any change to `shin_beam`, verify with
  `tools/section_check.py`, not `isValid()`.
- **`rom_scan_all()` never sweeps the foot, or the shin against the belly.** With a 55 mm
  shin the foot folds to within 23 mm of the thigh at 110° of knee; that figure is by
  hand, not by scan.

## Verifying a change

Any edit to `mini_dog.py` — a constant as much as a part function — is unfinished until all
six steps have run in the same pass, so the printed, analysed and simulated robots never
disagree. A "cosmetic" parameter still moves the moment arms `fea.py` derives.

1. `.venv/bin/python mini_dog.py` — every part valid; ROM table (hip pitch collapsing to
   ±15° means the pitch axis lost its 30 mm drop below the roll axis); every clearance
   line green, no `!!`.
2. Mass and print bbox: parts must fit a 256 mm bed.
3. `.venv/bin/python render.py` — look at the three PNGs.
4. `.venv/bin/python fea.py --all` (`--selftest` first if `fea.py` changed; `--orient` if a
   `PRINT_ORIENT` entry changed). Run it **before** the change too. Judge on the
   **inter-layer** SF (second of the `SF xy / z` pair). A drop is a regression: fix it or
   report before/after explicitly.
5. `.venv/bin/python export_sim.py --check` — rebuilds `out/sim/`, stands both files up in
   MuJoCo, then on the terrain scene; renders the camera;
   cross-checks total mass against `fea.robot_mass()`. `!! it fell over`, `!! the two
   files disagree` or a jumped link mass is a model regression.
6. Regenerate the ROS 2 description and run its regressions — the second exporter lives
   outside this tree and nothing here imports it:

   ```bash
   cd ../ros2 && ../3d/.venv/bin/python smalldog_description/scripts/generate_model.py
   ../3d/.venv/bin/python tools/standalone_sim.py --headless
   ../3d/.venv/bin/python tools/standalone_sim.py --headless --terrain
   ../3d/.venv/bin/python tools/standalone_sim.py --course
   ```

   No `colcon build` needed (`--symlink-install`). Seeds: `generate_model.py
   --terrain-seed N`, then the two terrain runs; regenerate without the flag afterwards.
7. `rom_scan(..., step=2)` before committing to real joint limits; `export_sim.py
   --rom-step 2` re-scans.

### Current baseline (2.318 kg; the no-GPU WSL2 box, MuJoCo 3.12.0)

Inter-layer SF per load case. `stall` scales with `SERVO_STALL_NM` only, the three ground
columns with `fea.robot_mass()`. Read `--orient` for `thigh_A` and `cradle_front`.

| | stand4 | stand2 | land3g | stall | `--orient` |
|---|---|---|---|---|---|
| `hip_bracket_A` | 47.6 | 23.8 | 7.9 | 2.1 | 7.78 |
| `thigh_A` | 15.4 | 7.7 | 2.6 | 1.1 | 2.81 |
| `shin_A` | 58.0 | 29.0 | 9.7 | 4.8 | 10.35 |
| `cradle_front` | 5.3 | 2.6 | 0.9 | 2.9 | **1.54** |
| `chassis_bottom` | 20.9 | 10.5 | 3.5 | — | 6.73 |

(`--orient` is the `<- current` row. `chassis_bottom` has no stall case and runs at a 3 mm
mesh: `part_specs()` says why. `thigh_A`'s ground columns are lower than the upright
robot's 17.6 / 8.8 / 2.9 because the crouch turns the ground reaction across the thigh.)

**The `stall` column is the live one, not a bound.** Under the servo firmware a joint
could not hold full duty; in MODE 2 (`robot/runtime/mode2.py`) it can, and a foot that
grips (the siped sole) hands the servo's whole torque to the bracket where a sliding dome
handed it μ·N. Both front `hip_bracket`s of the original robot broke that way, and the
part is unchanged. Read `hip_bracket_A` 2.1 and `thigh_A` 1.1 against ~3.3 N·m cold at a
fresh pack's 12.6 V (`SERVO_STALL_NM` is the 12 V figure), and re-run `fea.py --all` with
that torque before the first print of either. The runtime's `duty_cap` (600 ≈ 2.3 N·m) is
the stop-gap, not a fix for the part.

`export_sim.py --check`: `4 feet down, upright +1.00`, base z 148 mm, camera axis
(+0.99 0 +0.10), ROM ±90 / ±90 / ±110 (±90 is the scan window, not a stop). Probes:
`imu clear +1.20`, `batt clear +0.60`, `clamp clear +1.43`, `head clear +0.65`,
`cradle bolts +42.15` (to the pack), `cradle inserts` to x 111.5, `fork access: all six
arms`, `panel clear` (bus and low windows at both ends, XT30 clear of body and servos),
`node clear` (two 221-415 at x −104.4..−96.1), `camera bolts` (M3 × 12 flat, 2.3 mm of
deck above the seats, 2.6 mm past the nut), `opi bolts` (M2.5 × 11 flat, 2.5 mm into the
case's spacers), `opi clear` (rear standoff foot 48.7 mm inside the deck's end),
`camera view: out of frame` for all three.

Step 6, gait defaults (swing 12 mm, body 133 mm): **flat trot 557.1 mm**; **terrain seeds
7…12: 548 / 530 / 464 / 387 / 444 / 543 mm, 0/6 down**; **course seeds 7/8/9: 4/7 at
2468 mm, 0/7 at 1046 mm, 0/7 at 949 mm, all upright** — on 8 and 9 the 12 mm swing stalls
at the first log (14 mm) instead of falling over it. `ros2/smalldog_walker` tests: 11
passed.

### How to read step 6

- **The flat trot is deterministic to 0.1 mm and is the control**: unchanged means the
  model did not move. A change that touches no value (re-expressing a literal) must
  reproduce it exactly.
- **Compare against this machine only.** The same model reads a few mm differently on the
  mac, and MuJoCo 4.x puts it on its back on the terrain.
- **Read the terrain arm over seeds 7…12 or not at all**; one seed spans 387…548 here.
- **The course is a report, not a pass/fail**, read over seeds 7/8/9.
- **Mass sensitivity is not yet measured on this body.** The short-bodied robot's trot was
  chaotic in the gram (11 g anywhere: 778 → 597 mm; 7 g of dead mass put a terrain seed on
  its back), so run a +Δm control — the same mass parked in `ELECTRONICS_KG` on the
  unchanged geometry — before reading a distance change as the geometry's doing.
- **Do not recover distance with `MJ_DAMPING` or the rate limit.** Both were made honest
  against the measured servo (`(forcerange − frictionloss)/damping` = 3.28 rad/s).
- When `terrain.py` changes, re-baseline by running the unchanged model on the new course
  first. Seed sweeps need one heightfield file per seed (MuJoCo caches by file name).

## Notes

- FEA meshes in `out/fea/` are cached on the STEP content hash; stale entries are safe to
  delete.
- `README.md` marks several dimensions **verify** (where the Orange Pi sits in its case,
  connector bodies, camera lens); keep the marker until measured.
- `tools/section_check.py` gives exact section moments along the shin in seconds — use it
  to converge a profile, then still run `fea.py`.
