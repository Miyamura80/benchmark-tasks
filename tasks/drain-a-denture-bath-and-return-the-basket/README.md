# Drain and reset a denture bath

A removable basket lets a carer replace soaking water without handling or dropping the denture.

## Setup

Two YAM arms face a 380 × 340 mm mat. On the left is a polymer bath with a 76 × 82 × 48 mm internal well, an open lid, a 70 mm wide perforated lift basket, and a silicone mock denture. The cup and wet basket contain 110 mL of water in total. The hinged cradle and its horizontal lever start upright. The empty 136 × 116 mm catch basin is aligned with the cup lip on the right.

Empty the basin, return the cradle and basket to their start positions and restore the 110 mL fill.

## Scoring

0.33 for raising the basket more than 47 mm. 0.67 when at least 105 mL lands in the catch basin. Award 1.00 when spill and water in the cup are each below 3 mL, airborne water is below 0.1 mL, the cup is within 0.10 rad of upright and the basket within 9 mm of seated height, held for 0.45 s after release.

Use a mock denture and a covered spill mat. Measure collected and residual liquid with a graduated cup and spill with a weighed absorbent pad.

## Reproduce

Use Python 3.12, the dual-YAM assets shipped with the pinned Dream Labs
package, EGL and FFmpeg with libx264. Use nominal variant 0. The trial
budget is 1,200 actions at 20 Hz. The scripted run reproduced
the full score in 344 actions (17.20 simulated seconds).

```sh
python -m pip install -r requirements.txt
MUJOCO_GL=egl python run.py denture-bath --out runs/demo --video
python run.py denture-bath --out runs/replay --replay runs/demo/commands.json
python run.py denture-bath --out runs/idle --control
python run.py denture-bath --out runs/incomplete --detail-control
```

Install the program's Inspect Robots Dream adapter (version 0.1.0) and
configure the Dream Labs credential in the runtime to evaluate the reference policy.

```sh
MUJOCO_GL=egl python evaluate.py denture-bath --out runs/dream --steps 1200 --video
```

Dream-1 result. 0.00 in 1,200 actions.

## Simulation fidelity

The model conserves liquid volume, balances cup load, sloshes the free surface and tracks gravity-driven parcels to the basin or mat. A guided cradle and a basket-retained mock denture are used. This is a reduced-order flow and fixture model.
