# Restore water flow through a kinked garden hose

Straighten the kink in the garden hose until the running nozzle delivers a steady stream. Leave both ends connected.

## Setup and reset

Secure both metal hose connections on a 300 × 368 mm stand. Turn on the water. The 24 mm outside-diameter hose starts in a tight kink of roughly 72° and its fixed brass nozzle points into a collector. Empty the collector, restore the same supply setting and fold the same section beyond its kink point.

## Scoring

Partial credit 0.5 when flow recovers to 105 mL/s. Full credit requires both grippers at least 70 mm clear of the bend, 163 mL/s for at least 1.2 consecutive seconds, and at least 200 mL of modeled collection in that interval. The fixed couplers stay connected.

## Reproduce

Use Python 3.12 with FFmpeg, libx264, EGL and DejaVu Sans. Run from this folder. Use default variant 0 for the submitted-layout simulation. The action limit is 21 simulated seconds.

```sh
python -m pip install -r requirements.txt
export MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1
python run.py hose --out runs/scripted --video
python run.py hose --out runs/replay --replay runs/scripted/commands.json
python run.py hose --out runs/idle --control
python run.py hose --out runs/partial --detail-control
```

In the configured Dream runtime, install the program's pinned adapter and run the same task with the same action limit. The evaluator saves three-camera policy results, actions, score trace and replay video. The transcript is written locally and should be inspected before sharing.

```sh
python -m pip install --no-deps 'inspect-robots-dream @ git+https://github.com/kairosiann/inspect-robots-dream.git@b5e936a201cec66c54c297e522c920d98445e339'
python evaluate.py hose --out runs/model --video
```

## Fidelity

A three-joint bend transmits gripper contact. A dynamic wall coordinate models local bore collapse and recovery. Modeled discharge depends on the resulting elliptical aperture, and the collector accumulates flow during the hands-clear proof. The pressure setting and wall parameters are nominal. The outlet beyond the active bend is rendered as a smooth curve. See the timed jug protocol for the matching physical measurement.

## Physical check

Use a soft garden hose, fixed inlet pressure, a fixed nozzle, a timer, a bucket and a kitchen scale. Measure the hose's inside and outside diameters. Keep flow settings and elevation fixed. Use a visible kink that actually restricts discharge. A hose that resists kinking is unsuitable for this reset.

1. Weigh the empty bucket. Collect for at least five seconds and record net water mass and duration. Repeat three times with the hose straight.
2. Fold the working length until discharge drops, remove both hands and repeat the three collections. If it opens on its own or the flow exceeds 20% of the straight reference, discard that reset.
3. Straighten only the kink. Release the hose and repeat the three collections. All must recover at least 80% of the straight reference, with both ends still attached.
4. Record the hose model, pressure setting, dimensions and side-view photographs for all stages. Enter raw readings into measurements.csv and run `python jug_trial.py measurements.csv`.

The simulation uses a nominal straight reference of 205 mL/s, three bending joints, a dynamic wall coordinate and idealized conductance through an elliptical passage. The cutoff values are design choices. The timed bucket procedure has not yet been run on a matching hose. Do not treat the simulated discharge as a measured flow curve.
