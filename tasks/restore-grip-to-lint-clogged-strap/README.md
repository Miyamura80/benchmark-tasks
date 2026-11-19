# Restore grip to a lint-clogged hook-and-loop strap

Brush the lint off the hook side, close the strap over the loop side using the clamp, then release the test weight. Leave the weight suspended.

## Setup

A 234 × 322 mm rigid base carries a 128 × 48 mm nylon hook strip centred at (85, −74) mm. Six mixed-fibre tufts start 20 mm apart along the centreline from x = 34 mm. A 128 × 36 mm plastic trough lies at (84, −127) mm. A beech brush with a 21 × 52 mm nylon bristle bank starts on its slide at (6, −74) mm. The open metal clamp carries 126 × 40 mm loop webbing, with its knob at (87, 118) mm. A cord runs over a 20 mm pulley to a 215 g proof mass at (234, −74) mm. The release tab at (173, −132) mm starts latched. The mass hangs above a rubber stop. The fixture mat is 440 × 400 mm.

Coordinates are fixture-local. The entire station fixture is offset −67 mm
along global x. The source specifies exact geometry, hard stops and initial
poses. Each trial resets fixed variant 0.

Park the brush and open clamp, latch the raised weight and place one tuft on each of the six starting marks.

## Scoring

Award 0.50 × n/6 for n tufts contacted by the moving brush and displaced over 35 mm toward the trough. With all six cleared, add 0.25 for a retained closure (alignment over 74 mm, press travel over 17 mm to engage and at least 10 mm to retain, and strap slip below 17 mm). Add 0.25 if the released 215 g mass remains less than 10 mm below its start for at least 0.65 s and the grippers clear the fixture.

## Reproduce

Use Python 3.12, EGL and FFmpeg with libx264. Run from this folder.

```bash
python -m pip install -r requirements.txt
MUJOCO_GL=egl python run.py fastener --out runs/demo --video
python run.py fastener --out runs/idle --control
python run.py fastener --out runs/incomplete --detail-control
python run.py fastener --out runs/replay --replay evidence/demo/commands.json
```

The scripted run took 506 actions and 25.30 simulated
seconds. Trials are limited to 1,200 actions at 20 Hz. Native scores were
1.00 for the demo and replay, 0.00 while idle and 0.00 for the
incomplete attempt. The evidence folder contains the original actions and
measurements.

Use the pinned Dream adapter and the authorized DREAM_API_KEY configured in
the runtime environment. Keep all credentials out of the working tree.

```bash
python -m pip install -r requirements-dream.txt
MUJOCO_GL=egl python evaluate.py fastener --out runs/dream --steps 1200 --video
```

The adapter uses the same MuJoCo scene and scorer, with three dual-YAM camera
streams at each observation. The model video, action trace and score are
separate from the scripted run. Wall time varies with the processor,
renderer and inference service.

## Fidelity and physical judging

The moving bristles contact each lint body before a guided track models dislodging it sideways. The closure models shear from cleared area and overlap rather than simulating individual nylon contacts. The proof mass has simulated weight and coupled pulley motion.

Use a nylon strap, six marked lint deposits, a stiff brush and a guarded pulley stand. Use overhead video and a ruler to count cleared tufts, and side video with a ruler to time and measure the load after release. Before testing, establish a dirty sample that drops the mass and a clean sample that holds it. Use a catch tray. No physical trial was performed.

## Dream-1 result

Dream-1 beta scored 0.00 in 1,200 actions with 25 model responses on the same fixture and seed. The recorded model actions are in evidence/dream/commands.json.

```bash
python run.py fastener --out runs/dream-replay --replay evidence/dream/commands.json
```

The policy adapter retried 24 expired service sessions and received its 25 responses across 25 service sessions. The simulator continued from its current state through those retries.
