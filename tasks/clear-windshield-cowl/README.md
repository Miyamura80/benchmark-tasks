# Clear leaves from the base of the windshield

Pick up the leaves gathered below the windshield and put them in the tray. Clear any leaves covering the drain grilles.

## Setup

The scene is a 3.76 m long, approximately 1.64 m wide compact hatchback with four wheels, lamps, windows, doors, parked wipers and a contoured hood. The 90 cm wide cowl fixture has a 15 cm gutter and recessed drain grilles. Removing the ordinary visible leaves is the main task. Clearing the drain areas also matters because they allow collected rainwater to leave the gutter. Two leaves cover the drains beneath shallow retaining edges. The 25 by 21 cm collection tray is centred on the rear of the hood.

Two articulated I2RT YAMs with linear parallel grippers mount on external cantilever pedestals, one at each front fender. They each handle two leaves and share the tray. In world coordinates the base plates are at x = ±465 mm, y = −120 mm, z = 872 mm, with the inside yaw offsets at 40° and 140°. In the render, floor-standing uprights sit outside the body at x = ±920 mm and support the base plates through horizontal cantilevers. The models' bases are fixed in the simulator. The stand geometry represents the mounting arrangement and has no structural or physical stability validation.

The four leaves start at the coordinates below, in millimetres. The origin is at the hood-facing edge of the cowl gutter. Positive x points toward the car's right side and positive y toward the windshield. Leaves 1 and 4 overlap the grilles. Each leaf is about 52–73 mm long, 28–40 mm wide and 1.5 g in the simulation.

| Leaf | x | y |
| --- | ---: | ---: |
| 1 | -340 | 126 |
| 2 | -135 | 83 |
| 3 | 75 | 104 |
| 4 | 302 | 124 |

Reset by rebuilding the scene and allowing 1.32 seconds for settling. The scripted trial uses a fixed layout and no random seed.

## Scoring

Each leaf deposited in the tray earns 3 points. Each clear grille earns 2 points. Divide by 16 for the task score. A grille is blocked when leaf surfaces cover at least 5% of its sampled open area. For each slot, use 20 vertical rays between y = 124 and 153 mm and z = 94 and 135 mm relative to the cowl frame. Weight each sample by the slot width. A leaf is in the tray if its center is within 115 mm of the tray center laterally, 88 mm front to back, and between 2 mm below the tray floor and 27 mm above it. Score after both grippers have retreated and the leaves have settled.

## Replay

Requires Python 3.12. Install `requirements.txt`. Video export also needs EGL and ffmpeg.

```bash
python task.py
MUJOCO_GL=egl python task.py --output ./preview
```

The first command prints the score. The second exports an H.264 video, setup and result images, and a JSON score report. The scripted run takes 27.32 seconds of simulated time and scores 1.0. The video opens with the complete car, shows both drain grilles together before the first pickup, cuts to each leaf during the grasp, and returns to the same drain view after the last deposit. The export also includes matched `drains-before.png` and `drains-after.png` views. Add `--trace` for the individual tip errors and leaf poses at each waypoint.

To reproduce the gripper controls, run `python check_controls.py`. With either gripper held open the score is 0.5, with two leaves collected and one grille clear. With both held open the score is 0.

Both arm models and their joint limits come from I2RT's licensed MuJoCo description in MuJoCo Menagerie. Joint position actuators execute inverse-kinematics waypoints. The linear fingers use replacement collision pads for thin leaves. All pickup and release motion comes from the pad contacts, gravity and dynamics. The curled leaves are approximated as rigid objects with wider colliding patches around the midrib. The drain check estimates open-slot coverage using the visible leaf meshes. Water flow is not simulated. The active cowl, tray and nearby geometry have collision shapes. The full car's exterior panels outside the active area are primarily render meshes. Trajectories are scripted using settled leaf pose. No physical-arm test was run.

Dream-1 completed at **0.00** after 600 actions. The model received RGB and measured joint positions through Inspect Robots. The policy evidence is in `evidence/dream`.

Run the same policy evaluation from this task directory with Python 3.12, Inspect Robots 1.0.0, dreamlabs-client 1.0.14, DreamLabs 1.0.23, and the Inspect Robots Dream adapter at commit b5e936a201cec66c54c297e522c920d98445e339. With Dream-1 credentials configured, use `MUJOCO_GL=egl python evaluate.py cowl --out dream-results --video`.

The baseline made 12 successful model requests in 12 sessions after session expiry recoveries.

Ford and Volvo owner guides describe removing leaves from the area at the windshield base. The simulation represents visible leaves over two cowl grilles. It does not model water flow.
