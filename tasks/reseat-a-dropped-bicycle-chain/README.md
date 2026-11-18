# Reseat a dropped bicycle chain

A dropped bicycle chain can be restored by guiding its upper run back into the tooth plane while turning the crank.

## Setup and reset

A bench-scale bicycle drivetrain with 26 front teeth, 16 rear teeth, a crank and pedal, and a loose upper chain run starting 28 mm outboard of its target plane.

Shift the upper chain run outboard and reset crank angle, sprocket angle and the engagement state.

## Trial and score

0.5 once lateral travel exceeds 21 mm towards the tooth plane. Full credit once the engaged upper run has settled by more than 5 mm and the rear sprocket rotates over 1.40 rad.

Use Python 3.12, the Dream Labs dual-YAM reference and FFmpeg with libx264.
The nominal layout has a 60-second action cap. Calibration dependencies are
listed in requirements.txt.

```sh
python -m pip install -r requirements.txt
MUJOCO_GL=egl python run.py chain --out runs/demo --video
python run.py chain --out runs/idle --control
python run.py chain --out runs/incomplete --detail-control
python run.py chain --out runs/replay --replay runs/demo/commands.json
```

With the Dream Labs API credential available as `DREAM_API_KEY`, run the same nominal trial with Dream-1 beta. The rollout writes a video, final measurements and a request summary.

```sh
MUJOCO_GL=egl python evaluate.py chain --out runs/dream --video
```

## Fidelity

The visible chain is a continuous moving loop. Contact still uses a guided upper-run proxy. Engaged rotation transfers load through a compliant 26:16 coupling with backlash. The script verifies a partial turn and does not model tooth-by-tooth chain contact.

## Physical benchmark

With the bicycle supported and the drivetrain unloaded, guide the chain onto the teeth and rotate the crank manually. For a real-world durability check, two full crank revolutions would verify engagement. The simulation measures rear sprocket motion over a partial turn.
