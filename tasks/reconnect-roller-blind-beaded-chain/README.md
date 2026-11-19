# Reconnect a roller blind’s beaded chain

Reconnect the two ends of the chain through the coupler. Use opposite sides of the closed loop to lower the blind, then raise it back.

## Setup

A tabletop stand holds a 198 mm wide linen-like shade on a 64 mm diameter roll with its axle 443 mm above the simulation origin. A 196 mm aluminium hem bar starts 133 mm below the axle. Two steel uprights and an aluminium headrail support the roll. The side clutch has a 64 mm pitch diameter. Its pre-routed ball chain has approximately 8.4 mm pitch. The clasp rests in a plastic jig at (124, −103, 241) mm and the loose end starts 24 mm farther out along negative y. The 380 × 400 mm mat supports the fixture. The shade starts raised and the chain ends disconnected.

Coordinates are fixture-local. The entire station fixture is offset −67 mm
along global x. The source specifies exact geometry, hard stops and initial
poses. Each trial resets fixed variant 0.

Return the hem to the raised mark, disconnect the loose end, place both ends on the jig marks and leave the chain pre-routed over the clutch.

## Scoring

0.00 until the terminal passes 23 mm through the clasp and remains beyond 22 mm insertion. Award 1/3 for retaining the join. Award 2/3 when the repaired lowering side also moves the hem more than 22 mm below the raised reference. Award 1.00 when pulling the opposite side raises the hem more than 18 mm from that deepest position to less than 12 mm below the raised reference. Both hands must release the fixture and the join must hold.

## Reproduce

Use Python 3.12, EGL and FFmpeg with libx264. Run from this folder.

```bash
python -m pip install -r requirements.txt
MUJOCO_GL=egl python run.py blind --out runs/demo --video
python run.py blind --out runs/idle --control
python run.py blind --out runs/incomplete --detail-control
python run.py blind --out runs/replay --replay evidence/demo/commands.json
```

The scripted run took 481 actions and 24.05 simulated
seconds. Trials are limited to 1,200 actions at 20 Hz. Native scores were
1.00 for the demo and replay, 0.00 while idle and 0.33 for the
incomplete attempt. The evidence folder contains the original actions and
measurements.

Use the pinned Dream adapter and the authorized DREAM_API_KEY configured in
the runtime environment. Keep all credentials out of the working tree.

```bash
python -m pip install -r requirements-dream.txt
MUJOCO_GL=egl python evaluate.py blind --out runs/dream --steps 1200 --video
```

The adapter uses the same MuJoCo scene and scorer, with three dual-YAM camera
streams at each observation. The model video, action trace and score are
separate from the scripted run. Wall time varies with the processor,
renderer and inference service.

## Fidelity and physical judging

The simulation models the clasp retention and two working chain runs as guided contacts. A compliant transmission drives the roller and fabric payout. The bead positions along the inextensible rendered loop are driven by this mechanism.

Clamp a short shade to a stand with the loop guarded. Place a ruler behind the hem. Record the connector seating and retention, followed by the hem lowering and returning to the raised mark. Check that the grippers are clear at the end. No physical trial was performed.

## Dream-1 result

Dream-1 beta scored 0.00 in 1,200 actions with 25 model responses on the same fixture and seed. The recorded model actions are in evidence/dream/commands.json.

```bash
python run.py blind --out runs/dream-replay --replay evidence/dream/commands.json
```

The policy adapter retried 14 expired service sessions and received its 25 responses across 15 service sessions. The simulator continued from its current state through those retries.
