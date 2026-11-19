# Replace a foil shaver’s cutter and foil

Remove the worn foil and cutter. Fit the new cutter and foil, briefly run the shaver to check the assembly, then switch it off.

## Setup

A 147 mm moulded shaver lies switch-up in a 110 × 170 mm rubber cradle on a 400 × 400 mm mat. Its 58 × 22 mm steel cutter and 82 × 40 mm domed steel foil start installed at (84, 18) mm. Separate 82 × 52 mm plastic trays hold a compatible cutter at (158, −84) mm and foil at (166, 126) mm. Empty discard trays sit at (3, −135) and (10, 140) mm. The power switch at (80, −46) mm and its indicator face up. Power starts off.

Coordinates are fixture-local. The entire station fixture is offset −67 mm
along global x. The source specifies exact geometry, hard stops and initial
poses. Each trial resets fixed variant 0.

Reset power off, put both old parts in the head, return the new parts to their stock trays and empty the discard trays.

## Scoring

Milestones are sequential. 0.25 when both old parts are released in their discard trays, within 26 mm horizontally and 9 mm vertically of their reference centres. 0.50 when the new cutter is also within 6 mm on each axis of its seat with absolute long-axis and positive up-axis dot products above 0.97. 0.75 when the new foil is also released, within 5.5 mm on each horizontal axis and 6 mm vertically of its seat, with the same orientation bounds and both tabs within 6.5 mm crosswise and vertically. 1.00 after at least 0.35 s powered with both replacements seated and at least 1 mm of cutter travel, with power off at the end. Powering with neither foil registered gives 0.00.

## Reproduce

Use Python 3.12, EGL and FFmpeg with libx264. Run from this folder.

```bash
python -m pip install -r requirements.txt
MUJOCO_GL=egl python run.py shaver --out runs/demo --video
python run.py shaver --out runs/idle --control
python run.py shaver --out runs/incomplete --detail-control
python run.py shaver --out runs/replay --replay evidence/demo/commands.json
```

The scripted run took 647 actions and 32.35 simulated
seconds. Trials are limited to 1,200 actions at 20 Hz. Native scores were
1.00 for the demo and replay, 0.00 while idle and 0.75 for the
incomplete attempt. The evidence folder contains the original actions and
measurements.

Use the pinned Dream adapter and the authorized DREAM_API_KEY configured in
the runtime environment. Keep all credentials out of the working tree.

```bash
python -m pip install -r requirements-dream.txt
MUJOCO_GL=egl python evaluate.py shaver --out runs/dream --steps 1200 --video
```

The adapter uses the same MuJoCo scene and scorer, with three dual-YAM camera
streams at each observation. The model video, action trace and score are
separate from the scripted run. Wall time varies with the processor,
renderer and inference service.

## Fidelity and physical judging

Two loose steel parts, spring-loaded foil registration and a motor-driven cutter are simulated with simplified contact shapes. The stainless through-holes are render geometry. Bounded spring forces represent the locating faces, drive and supported two-finger grasp.

Use a compatible appliance in a padded cradle. Mark tray and head positions, compare fit against the appliance's flush-fit and latch checks, and video the fully covered power test. A contact vibration sensor can verify operation with the foil shut. Cutter travel is a direct simulation measurement and has not been physically validated. Never power the appliance with the cutter exposed.

## Dream-1 result

Dream-1 beta scored 0.00 in 1,200 actions with 25 model responses on the same fixture and seed. The recorded model actions are in evidence/dream/commands.json.

```bash
python run.py shaver --out runs/dream-replay --replay evidence/dream/commands.json
```

The policy adapter retried 24 expired service sessions and received its 25 responses across 25 service sessions. The simulator continued from its current state through those retries.
