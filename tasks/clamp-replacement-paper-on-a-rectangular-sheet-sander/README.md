# Clamp replacement paper on a rectangular sheet sander

Clamp-style sanders use a flexible strip that must stay taut as the second clamp closes.

## Setup and reset

A rectangular 120 × 196 mm sanding pad is face-up. One end of the replacement abrasive strip begins at the rear clamp, the front clamp is open, and the removed strip lies beside the tool.

Open the front clamp, reset the clamp and sheet joints, and restore slack to the replacement strip.

## Trial and score

0.5 after the front clamp travels 18 mm while the robot holds more than 14 mm of sheet take-up. Full credit when a proof tug lasts at least 150 ms while keeping the sheet within 6 mm of its locked position and developing more than 3 N of measured holding force.

Use Python 3.12, the Dream Labs dual-YAM reference and FFmpeg with libx264.
The nominal layout has a 60-second action cap. Calibration dependencies are
listed in requirements.txt.

```sh
python -m pip install -r requirements.txt
MUJOCO_GL=egl python run.py sander --out runs/demo --video
python run.py sander --out runs/idle --control
python run.py sander --out runs/incomplete --detail-control
python run.py sander --out runs/replay --replay runs/demo/commands.json
```

With the Dream Labs API credential available as `DREAM_API_KEY`, run the same nominal trial with Dream-1 beta. The rollout writes a video, final measurements and a request summary.

```sh
MUJOCO_GL=egl python evaluate.py sander --out runs/dream --video
```

## Fidelity

The visible paper remains anchored at the rear clamp and bows as it loses tension. Contact uses a spring-backed slider. The closed jaw limits its holding force by clamp travel. The old sheet is removed and the rear end is preloaded at reset.

## Physical benchmark

Use an unplugged clamp-style finishing sander. Replace the paper and pull on the exposed edge after closing both clamps. Verify both ends are retained and there are no slack folds.
