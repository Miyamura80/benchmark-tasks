# Fit a rollator meal tray

A removable meal tray must remain clipped during ordinary handling of the walker.

## Setup

An empty, stationary full-height four-wheel rollator has a tubular frame, handgrips about 865 mm high, 126 mm rubber tyres, a seat and fabric basket. Two horizontal metal mounting rails are 196 mm apart. A 198 × 164 mm polymer tray starts above the rails on a lowering guide, with both contrasting clips open. The two YAM arms face the tray.

Raise the tray to its starting guide position, re-open both clips and leave the walker stationary.

## Scoring

0.25 for each clip advanced more than 18 mm. Award 1.00 when the tray lowers more than 27 mm, both clips stay engaged during an upward tug that deflects the seat by at least 1.2 mm for over 0.25 s, and the assembly remains seated after release for 0.45 s.

Use an unoccupied walker on a level floor. Read clip and tray marks during a short pull with a spring scale.

## Reproduce

Use Python 3.12, the dual-YAM assets shipped with the pinned Dream Labs
package, EGL and FFmpeg with libx264. Use nominal variant 0. The trial
budget is 1,200 actions at 20 Hz. The scripted run reproduced
the full score in 436 actions (21.80 simulated seconds).

```sh
python -m pip install -r requirements.txt
MUJOCO_GL=egl python run.py rollator-tray --out runs/demo --video
python run.py rollator-tray --out runs/replay --replay runs/demo/commands.json
python run.py rollator-tray --out runs/idle --control
python run.py rollator-tray --out runs/incomplete --detail-control
```

Install the program's Inspect Robots Dream adapter (version 0.1.0) and
configure the Dream Labs credential in the runtime to evaluate the reference policy.

```sh
MUJOCO_GL=egl python evaluate.py rollator-tray --out runs/dream --steps 1200 --video
```

Dream-1 result. 0.00 in 1,200 actions.

## Simulation fidelity

The full-height frame is stationary. Both clips and the tray have constrained motion. A measured spring deflection gives an approximate retention load with the clips engaged.
