# CLAUDE.md

A 12-DOF printed quadruped on 12 × Feetech ST3215 servos - the dachshund fork of
`alarin/smalldog`: 220 mm body, 75 / 55 mm legs, camera and IMU, no LiDAR, no GPS. It exists
in CAD and simulation only (`PLAN.md`). `README.md` is the human-facing overview and the
machine map; this file only routes.

## The trees

| tree | what it is | read first |
|---|---|---|
| `3d/` | **the source of truth** — code-CAD, FEA, the URDF/MJCF exporters | `3d/CLAUDE.md` |
| `ros2/` | ROS 2 workspace + MuJoCo. `smalldog_description` is **generated** from `3d/`, never authored | `ros2/README.md` |
| `rl/` | the RL walking policy — the third consumer of the same CAD | `rl/CLAUDE.md` |
| `robot/` | the hardware side: bus driver, bench, the 50 Hz runtime | `robot/README.md` |

`3d/CLAUDE.md`'s rule about what is source and what is output holds across the whole
repository: anything under `ros2/smalldog_description/` or `3d/out/` is output — fix it in
`3d/mini_dog.py` and regenerate.

## Machines

Four machines, no shared filesystem, no scp: **the git repository is the only thing that
crosses**, so a result that has to reach another machine gets committed.

| machine | does | runbook |
|---|---|---|
| a mac | CAD, FEA, both sim exporters, the ROS 2 workspace | `3d/CLAUDE.md` |
| Windows/WSL2, RTX 3070 (8 GB) | RL training, the pure-Python sim regressions | `WSL.md` |
| Windows/WSL2, RTX 5070 Ti (16 GB) | RL training — the box-terrain run 8 GB never allowed; also the gaming PC | `GPU.md` |
| Orange Pi 5 Pro | `robot/runtime` on the robot, bare or under ROS 2 Jazzy (`ros2/smalldog_hardware`) | `robot/README.md`, `ros2/README.md` "On the robot" |
| Windows/WSL2, no CUDA GPU | CAD, FEA, both sim exporters, the pure-Python sim regressions, `rl/checks/check_model.py`. **No RL training** | below |

Work out which one you are on before running anything; `WSL.md` has the check, and
`nvidia-smi` says which of the two training boxes this is. The first four are the upstream
author's; the dachshund geometry was done on the fifth, and its numbers are that box's.

### The no-GPU WSL2 box

The checkout is on the Windows side (`C:/git/robo-dackel`, `/mnt/c/git/robo-dackel` from
WSL) and the tools run inside WSL Ubuntu from a venv outside the tree:

```bash
python3 -m venv ~/venvs/robo-dackel-3d
~/venvs/robo-dackel-3d/bin/python -m pip install cadquery sfepy gmsh scipy pillow meshio vtk numpy pytest "mujoco==3.12.0"
```

- **`mujoco==3.12.0`, the version `rl/uv.lock` pins.** 4.0.1 loads the same model and
  puts the robot on its back on the terrain scene.
- **gmsh needs `libGLU.so.1` and `libOpenGL.so.0`.** Without sudo: `apt-get download
  libglu1-mesa libopengl0 libglvnd0`, `dpkg -x` each into `~/venvs/robo-dackel-3d/syslibs`,
  and export `LD_LIBRARY_PATH=~/venvs/robo-dackel-3d/syslibs/usr/lib/x86_64-linux-gnu`.
- **Wherever a doc says `.venv/bin/python`, use that venv's python**, from the same
  directory the doc names.
- **7.6 GB of RAM.** `fea.py`'s tray case runs at a 3 mm mesh for that reason; `ulimit -v`
  segfaults the gmsh/sfepy stack, do not cap it that way.
- **Driving WSL from Git Bash:** put the commands in a script file and run
  `MSYS_NO_PATHCONV=1 wsl.exe -d Ubuntu -- bash /mnt/c/.../script.sh`. An inline
  `bash -c '...'` has its `$variables` expanded away before bash sees them, and without
  `MSYS_NO_PATHCONV` the `/mnt/c` path is rewritten into a Windows one.

## Conventions for these files

- **Dated history lives in `git log`, not here.** A doc carries the current rule, the
  current number and a one-line reason. A superseded number is deleted, not annotated.
- A number marked **verify** is unmeasured. Keep the marker until it has met hardware.
- When a bench number changes it goes in two places: the tree that uses it, and
  `ST3215_STS3215_measured_parameters.md`, the public reference.
- Commit only when asked. The mac and the WSL2 box share `main`: `git pull --rebase` first.
