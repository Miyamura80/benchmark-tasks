# Add an easy-grip zipper loop

An attached cord loop makes a small zipper pull easier to grasp. Closing the zipper tests that the loop carries load.

## Setup

Two YAM arms face a 380 × 354 mm rubber work mat. A short textile garment panel is laid flat with its metal zipper partly open. Its slider has an 18 mm diameter pull eye and 90 mm available closing travel. A soft orange cord loop about 7 mm thick is staged beside the slider, with its leading bight to the left and its tail to the right.

Open the slider to its start mark and remove and restage the loop beside the pull eye.

## Scoring

0.33 for feeding the bight through the eye by over 31 mm. 0.67 for forming the hitch with both bight and tail advances over 24 mm. Award 1.00 when pulling the added loop moves the slider more than 83 mm toward closed and it holds there for 0.55 s after release.

Use a zipper panel and an unattached loop. Inspect the hitch and verify the slider went past a closing mark when pulled by the loop.

## Reproduce

Use Python 3.12, the dual-YAM assets shipped with the pinned Dream Labs
package, EGL and FFmpeg with libx264. Use nominal variant 0. The trial
budget is 1,200 actions at 20 Hz. The scripted run reproduced
the full score in 280 actions (14.00 simulated seconds).

```sh
python -m pip install -r requirements.txt
MUJOCO_GL=egl python run.py zipper-loop --out runs/demo --video
python run.py zipper-loop --out runs/replay --replay runs/demo/commands.json
python run.py zipper-loop --out runs/idle --control
python run.py zipper-loop --out runs/incomplete --detail-control
```

Install the program's Inspect Robots Dream adapter (version 0.1.0) and
configure the Dream Labs credential in the runtime to evaluate the reference policy.

```sh
MUJOCO_GL=egl python evaluate.py zipper-loop --out runs/dream --steps 1200 --video
```

Dream-1 result. 0.00 in 1,200 actions.

## Simulation fidelity

The bight and tail use guided motions. A coupled force acts on the slider only after measured threading and cinching. The cord is rendered as a deforming curve rather than a full rope-contact solve.
