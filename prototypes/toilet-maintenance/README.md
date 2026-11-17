# Toilet maintenance prototypes

The two tasks share a toilet with an open seat ring, raised lid, rear hinges and cistern, plus a four-axis Cartesian gripper. Position actuators command the gripper and its jaws. Free rigid tools move through jaw contact. A fixed camera and a cutaway show the repair in each run.

| Task | Scripted behavior | Measured outcome |
| --- | --- | --- |
| Rim jet | Compare the four collection volumes, align the fork with the weak outlet, withdraw the obstruction and repeat the rinse. | All four fault locations scored 1.00. The affected outlet rose from 20 mL to 100 mL in the model. Each misaligned control scored 0.25 because it diagnosed the correct outlet and did not restore enough flow. |
| Mixed clog | Pick up the plunger, seat the cup, apply pressure strokes and repeat the drain test. | Score 1.00. Modeled water remaining fell from 88.75% to 1.25%. An off-centre plunger did not engage the diaphragm and scored 0.00. |

## Run

Install Python 3.12, FFmpeg and EGL. Run from this directory.

```bash
python -m pip install 'mujoco==3.3.7' Pillow
MUJOCO_GL=egl python bench.py rim --blocked 1 --out runs/rim --video
MUJOCO_GL=egl python bench.py trap --out runs/trap --video
python bench.py rim --blocked 1 --control --out runs/rim-control
python bench.py trap --control --out runs/trap-control
```

Rim faults 0–3 are front left, front right, rear left and rear right. Outputs include `result.json`, a 25 Hz trace, and a video when requested. The rim policy identifies the fault from the first rinse volumes. The hidden fault index configures the reset. The clip uses it only to frame the outlet cutaway.

## Physics and verification

- The ceramic shape has segmented collision geometry that leaves the bowl opening clear. Four recessed openings sit beneath the porcelain lip and direct the rinse inward and down. The rim obstruction moves along a constrained, counterbalanced slide inside its opening. The fork engages its shoulder through collision. Flow is a reduced-order function of measured obstruction travel. The cup-volume display represents observed collection for the prototype.
- The plunger moves as a free rigid body. Compression of a diaphragm through cup contact produces a modeled pressure pulse on the clog. Paper and simulated waste form one composite rigid blockage in a straight cutaway section. The drainage rate depends on measured clearance. Flexible paper, waste deformation, water simulation and siphon mechanics need separate validation.
- The 1.00 scores are local CPU prototype results. Wet calibration, the YAM port, Dream-1 runs and submission media remain pending.
