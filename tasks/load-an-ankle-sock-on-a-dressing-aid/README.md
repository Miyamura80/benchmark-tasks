# Load a sock on a dressing aid

Loading a sock aid is a preparatory step for people who have difficulty reaching their feet.

## Setup

The two YAM arms face a 310 × 380 mm rubber work mat. A 152 mm long, 72 mm wide polymer sock-dressing trough lies lengthwise on the mat, with two nylon pull cords out to the sides. A stretch-knit ankle sock has its toe started over the narrow end. The two sides of its mouth start square, short of the rolled retaining lips.

Return both cuff edges to the starting marks and lay the cord ends away from the fingers.

## Scoring

Give 0.25 for each cuff edge advanced more than 50 mm from reset. Award 1.00 when both edges clear 50 mm, differ by less than 7 mm, and hold for 0.50 s with both grippers released.

Mark both start edges with removable tape. Use a ruler and a timed close-up to check advance, skew and retention.

## Reproduce

Use Python 3.12, the dual-YAM assets shipped with the pinned Dream Labs
package, EGL and FFmpeg with libx264. Use nominal variant 0. The trial
budget is 1,200 actions at 20 Hz. The scripted run reproduced
the full score in 214 actions (10.70 simulated seconds).

```sh
python -m pip install -r requirements.txt
MUJOCO_GL=egl python run.py sock-aid --out runs/demo --video
python run.py sock-aid --out runs/replay --replay runs/demo/commands.json
python run.py sock-aid --out runs/idle --control
python run.py sock-aid --out runs/incomplete --detail-control
```

Install the program's Inspect Robots Dream adapter (version 0.1.0) and
configure the Dream Labs credential in the runtime to evaluate the reference policy.

```sh
MUJOCO_GL=egl python evaluate.py sock-aid --out runs/dream --steps 1200 --video
```

Dream-1 result. 0.00 in 1,200 actions.

## Simulation fidelity

Finger contact loads both guided cuff edges. Spring detents approximate retention by the trough lip. The sock surface deforms with edge travel. The cloth and cords are visual geometry on a reduced-order fixture.
