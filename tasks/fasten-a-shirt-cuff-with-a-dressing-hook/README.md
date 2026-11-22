# Fasten a cuff with a dressing hook

A wire-loop dressing hook helps fasten small buttons when direct finger dexterity is limited.

## Setup

Two YAM arms face a 296 × 338 mm mat. A stationary, roughly 385 mm silicone forearm-and-hand form rests palm-down on a padded block. A blue cotton-style shirt sleeve covers the full forearm and overlaps a 69 mm long wrist cuff, leaving the hand exposed. The 22 mm button starts outside its buttonhole. A polymer-handled steel wire-loop hook rests to the right in its guide.

Unbutton the cuff, release tension from its edge and park the hook to the right.

## Scoring

0.33 when the gripped hook catches behind the button with the cuff edge tensioned over 17 mm. 0.67 when the engaged button travels more than 25 mm through the guided passage. Award 1.00 when travel stays above 25 mm, the tool lifts clear by over 8 mm, and the cuff stays fastened for 0.48 s after both grippers release.

Use a dressed arm trainer, a dressing hook and a timed close-up showing that the button remains fastened after release.

## Reproduce

Use Python 3.12, the dual-YAM assets shipped with the pinned Dream Labs
package, EGL and FFmpeg with libx264. Use nominal variant 0. The trial
budget is 1,200 actions at 20 Hz. The scripted run reproduced
the full score in 269 actions (13.45 simulated seconds).

```sh
python -m pip install -r requirements.txt
MUJOCO_GL=egl python run.py button-hook --out runs/demo --video
python run.py button-hook --out runs/replay --replay runs/demo/commands.json
python run.py button-hook --out runs/idle --control
python run.py button-hook --out runs/incomplete --detail-control
```

Install the program's Inspect Robots Dream adapter (version 0.1.0) and
configure the Dream Labs credential in the runtime to evaluate the reference policy.

```sh
MUJOCO_GL=egl python evaluate.py button-hook --out runs/dream --steps 1200 --video
```

Dream-1 result. 0.00 in 1,200 actions.

## Simulation fidelity

The hand and full sleeve are continuous rendered meshes. A guided cloth edge, button and hook approximate the motion and retention. The silicone form stays fixed throughout the trial.
