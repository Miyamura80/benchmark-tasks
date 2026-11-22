# Replace a hearing-aid wax guard

Replacing a clogged wax guard restores a routine serviceable part on a receiver.

## Setup

Two YAM arms face a 274 × 290 mm work mat. A receiver mock-up with a 22 mm diameter carrier sits port-up in a metal nest. The used polymer guard in the port and the fresh guard are approximately 6.4 mm across the flange. A 48 mm stem extraction applicator is parked 69 mm to the left. The insertion applicator with the fresh guard is parked 69 mm to the right. The receiver and its cable stay on the bench.

Seat the spent guard in the receiver, load a fresh guard on the insertion tool and park both applicators.

## Scoring

0.33 when the spent guard breaks free and is parked more than 36 mm to the left and 13 mm above the seat. 0.67 when a new guard releases from the aligned applicator under at least 0.55 N of pressure for 0.05 s and sits within 1.7 mm vertically and 1.8 mm laterally of its seat. Award 1.00 if both tools are parked and released for 0.40 s.

Use a receiver trainer and side-view macro photo against a scale to check flushness and tool clearance.

## Reproduce

Use Python 3.12, the dual-YAM assets shipped with the pinned Dream Labs
package, EGL and FFmpeg with libx264. Use nominal variant 0. The trial
budget is 1,200 actions at 20 Hz. The scripted run reproduced
the full score in 458 actions (22.90 simulated seconds).

```sh
python -m pip install -r requirements.txt
MUJOCO_GL=egl python run.py wax-guard --out runs/demo --video
python run.py wax-guard --out runs/replay --replay runs/demo/commands.json
python run.py wax-guard --out runs/idle --control
python run.py wax-guard --out runs/incomplete --detail-control
```

Install the program's Inspect Robots Dream adapter (version 0.1.0) and
configure the Dream Labs credential in the runtime to evaluate the reference policy.

```sh
MUJOCO_GL=egl python evaluate.py wax-guard --out runs/dream --steps 1200 --video
```

Dream-1 result. 0.00 in 1,200 actions.

## Simulation fidelity

Both guards are independent jointed parts. The extractor barb and bore shoulder use compliant force models. Planar channels guide the tools and parts at enlarged bench-fixture scale.
