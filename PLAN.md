# PLAN.md — what to build next, and why

This fork is the dachshund: a 220 mm body on 75 / 55 mm legs, camera and IMU, no LiDAR and
no GPS (`plan/README.md` has the comparison with the original). **It exists in CAD and in
simulation only.** Nothing is bought or printed, and no policy is trained.

Steps 1–5 below are the original robot's bench work. They are about the ST3215 servo, not
about a body, so their results carry over; where one needs the assembled robot it says so.

## Where things stand

| tree | state |
|---|---|
| `3d/` | dachshund geometry done; the ladder in `3d/CLAUDE.md` is green on the current tree |
| `ros2/` | generated from the CAD; the re-tuned trot's flat, terrain and course regressions are baselined. `smalldog_nav` and the SLAM tools need a LiDAR this robot does not carry |
| `rl/` | environment adapted, `check_model.py` 0 FAIL, **no policy for this body** |
| `robot/` | written and measured on the original robot; `calib.json`, the contact baselines and the launch `FIT` points are that robot's and have to be re-taken |

## Done — where the result lives

| step | result | write-up |
|---|---|---|
| 1 servo identification | friction 0.19 N·m + 0.28 per N·m carried; stiffness 40.9 N·m/rad at 12 V; total speed-proportional 1.37 N·m·s/rad (`b_v` is not separable from back-EMF on this bench — use the total) | `robot/README.md`, "Two things the fit cannot find" |
| 2 `mu_load` in `actuator.py` | `tau_c` 0.184, `J_m` 0.0165, `mu_load` 0.286 | `robot/README.md`, "The load-dependent friction term" |
| 2b static friction | `simulate()` is Karnopp; the training path gets stick from MuJoCo `frictionloss` = `tau_c`; the refit that followed was **not** adopted (mixed result) | `rl/CLAUDE.md`, "Re-baselines" |
| 2c stall torque | **4.50 N·m at 12 V** on a scale — an extrapolation the full-duty ladder has since bent to **~3.2 peak, ~2.3 sustained** (3d below) | `robot/README.md`, "The stall torque" |
| 3 fitted constants into the CAD | `MJ_KP` 40.9, `MJ_FRICTIONLOSS` 0.184, `MJ_DAMPING` 1.37, `MJ_ARMATURE` 0.0165 | `3d/mini_dog.py` section 4; `3d/CLAUDE.md` baseline |
| 3b gait rate limit | limits against the achievable 3.15 rad/s ceiling, not the no-load speed | `ros2/README.md`, "Gait" |
| 3c no-load speed | **3.86 rad/s**, a firmware profile cap: open loop the motor does 5.0 (`--pwm`, reading A) | `robot/README.md`, "The no-load speed" |

---

## 3c, closed: the plateau is the position loop's profile (A)

`noload_speed.py --pwm` ran on the second unit: open loop the speed is linear in duty
through 1000 (5.0 rad/s at 12.1 V, k_e 2.39), while position mode on the same unit stops
at 3.88 with the register flat at 2500 counts/s. What `rl/actuator.py` still lacks is a
rate cap on the *goal* — a state per joint, not a duty cap (**done 2026-09-14**, as a speed
*and* acceleration profile, `actuator.profile_goal`; `rl/CLAUDE.md`, "Re-baselines"). The rig then ran to full duty
(3d): the loop does apply it, and the motor is what bends.

## 3d, decided: 3.2 in the model, damping in the same units, the held 2.3 is a budget

`SERVO_STALL_NM` = **3.2** — the 2 s cold peak the rig measured — for FEA (stall SF ×1.4)
and both sims' `forcerange`. `MJ_DAMPING` 1.37 → **0.92**: the ladder's total was in its own
calibration (k_t 0.596 N·m/V, before the gearbox's load loss) and the stall in the scale's
(0.400 after it); the joint ceiling divides one by the other, so both carry the scale's
now, and the ceiling is 3.15 → **3.28 rad/s** — speed kept, stall honest. `rl/` zeroes
`MJ_DAMPING` and its law's own stall after `mu_load` is ~3.2 already. **Open:** the held 2.3 N·m and the
2 s timer are a per-joint duty budget for `robot/runtime/safety.py`, not written; and a 3 kg
scale at 12 V would confirm the 3.2 peak directly.

## 4. Randomise over the pack

**Done:** `mu_load` is drawn per joint per episode over the same `[0.40, 2.20]` as `tau_c`;
`check_model.py` asserts every per-unit servo parameter comes back batched. Voltage is
measured rather than guessed.

**Done:** four units through `holdbi` and `speed` at 12 V (`robot/README.md`, "Unit to
unit"). Electrical side within 3 %; friction floor 0.64–1.01× and load slope 0.88–1.33× of
the nominal. `tau_c`, `mu_load` and `kp` in `rl/params/domain_rand.json` are `measured`
ranges now — 2.2× narrower than the guess on tau_c. A policy trained before this saw a
wider world than exists; nothing needs retraining for it.

## 5. Settle the traction question before touching a sole

Two bench runs, no code, and they need a robot on a floor (step 8). The shin angles and
foot duties quoted below are the original robot's; re-read them off the dachshund first.
The sim has already been asked and cannot answer: four alternative
soles were built and measured (`ros2/README.md`, "The foot's contact patch") and none
bought anything, because MuJoCo contact is rigid and Coulomb friction ignores area. On
hardware the foot is a true R = 13 hemisphere with a 2.5 mm Hertz patch at 1.3 MPa, and the
sim ships μ = 1.2 where TPU on a bench is 0.3–0.5. Whether the robot still *slips* after
the gait fit (`robot/README.md`) is an inference, not a measurement.

1. **Dust the runway** (talc or flour over the ~40 cm). A planted foot leaves a dot, a
   sliding one a streak; streak length is slip per step. Read it per foot — the plate sits
   forward, so front duty is 0.68 against the rear's 0.40.
2. **A/B the surface**: same gait, same start mark, same seconds, bare bench against a
   rubber mat. Distance grows → the deficit is slip and sole work pays. Unchanged → traction
   is not where the speed goes. Shorten the run rather than let one arm hit the end.
3. **Only if 1–2 say yes:** cross-leg consistency from the encoders — two planted diagonal
   feet each imply a body velocity through FK, and their disagreement is slip. Use measured
   positions, not commanded (tracking error peaks at 35°).

What each answer costs: the friction coefficient is `MJ_FOOT_FRICTION` in `mini_dog.py`
section 4 plus the ladder — cheap. Geometry is a full ladder and a BOM line, and two things
to know first: the shin is 34.6° off vertical under load (46.7° worst), so a flat normal to
the shin lands on its edge — rake it or use a large-radius crown; and the terrain
heightfield cannot compare feet at `CELL_MM` = 12 (a ⌀17.7 face is smaller than one cell),
so `--rough` is the arm that can. Choose a shape with coupons on a tilt plate, not with feet.

## 6. Train

Nothing blocks it but a machine: MJX needs a CUDA GPU (`WSL.md`, `GPU.md`). From scratch —
the action is an offset from the stand pose, which moved, so the old policy was removed
rather than fine-tuned. `python train_ppo.py`, then `eval.py --seconds 5` against the IK
trot (556.2 mm flat, 525.6 mm on the heightfield), then `export_onnx.py` into `rl/policy/`
and commit it.

Look at these after the first run, not before: the command ranges and `ACTION_SCALE` in
`rl/env/walk.py` (sized for an 82 mm shin), and `rl/tools/bc_trot.py`'s gait literals.

What to expect it to beat: the hand-tuned trot swings the foot 12 mm and stalls at a 14 mm
log on two of three course seeds.

**This is the gate.** A policy that walks in `replay.py` and beats the trot in `eval.py` is
what justifies buying parts.

---

## 7. Buy and print

Parts list in `plan/README.md`. Before the full set: print `servo_gauge` and a
`M3_TAP` / `M25_TAP` coupon (`3d/CLAUDE.md`, both radii are unverified), then one leg.
Measure the knee lead's bend behind the plug on that leg — 14 mm of escape is what holds
the thigh at 75 mm, and a real lead that needs less would let it shorten.

## 8. Bring-up

`plan/README.md` section 7. Three things are this robot's and not inherited:

1. **Re-capture `robot/runtime/calib.json`.** Its own soft-limit block wins over
   `robot_params.json`.
2. **Weigh it, front feet and rear feet.** The CAD puts the centre of mass at the hip
   centre; the original measured 25–40 mm ahead of its CAD. The result goes into
   `stance_x` (`robot.launch.py`) and `com_offset_m_abs` (`rl/params/domain_rand.json`),
   and the policy is retrained on the measured band.
3. **Check the bus on the long leads.** The front trunk runs 47 mm further than on the
   original, which already showed checksum errors on two rear-leg leads under torque
   (`robot/README.md`).

Step 5's traction runs belong here too: they need the robot on a floor.

## 9. If it should see

The camera is in the head, looking out of the forehead in portrait, and nothing uses it
yet. A LiDAR would need a mount designed
for the long deck — the old bracket's positions were tied to the 126 mm body.

---

**Throughout:** every model change goes through the ladder in `3d/CLAUDE.md` in one pass,
and every bench number goes into the repo *and* `ST3215_STS3215_measured_parameters.md`.
