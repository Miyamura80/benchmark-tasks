# Toilet maintenance prototypes

The two tasks share a toilet with an open seat ring, raised lid, rear hinges and cistern, plus a four-axis Cartesian gripper. Position actuators command the gripper and its jaws. Free rigid tools move through jaw contact. A fixed camera and a cutaway show the repair in each run.

| Task | Scripted behavior | Measured outcome |
| --- | --- | --- |
| Rim jet | Compare the four collection volumes, align the fork with the weak outlet, withdraw the obstruction and repeat the rinse. | All four fault locations scored 1.00. The first two-second rinse delivers 200 mL at each healthy jet and 40 mL at the restricted one. The repaired jet delivers 200 mL on the retest. All misaligned controls scored 0.25. |
| Mixed clog | Seat the plunger and clear six separate pieces of wet toilet paper and feces. | Eight strokes cleared all six pieces, reducing modeled water remaining from 95% to 1.25%. Seven strokes left a paper wad in the pipe. The off-centre control cleared zero pieces. |

## Run

Install Python 3.12, FFmpeg and EGL. Run from this directory.

```bash
python -m pip install 'mujoco==3.3.7' Pillow
MUJOCO_GL=egl python bench.py rim --blocked 1 --out runs/rim --video
MUJOCO_GL=egl python bench.py trap --out runs/trap --video
python bench.py rim --blocked 1 --control --out runs/rim-control
python bench.py trap --control --out runs/trap-control
python bench.py trap --strokes 7 --out runs/trap-seven-strokes
```

Rim faults 0–3 are front left, front right, rear left and rear right. Outputs include `result.json`, a 25 Hz trace, and a video when requested. The rim policy identifies the fault from the first rinse volumes. The hidden fault index configures the reset. The clip uses it only to frame the outlet cutaway.

## Physics and verification

- The ceramic shape has segmented collision geometry that leaves the bowl opening clear. Four recessed openings sit beneath the porcelain lip and direct the rinse inward and down. Each runs independently during both rinses. The fork engages the captive obstruction through contact, and that outlet's flow depends on the obstruction's measured position. The overhead view shows all four outlets.
- The plunger moves as a free rigid body. Compression of a diaphragm through cup contact produces a modeled pressure pulse. Three paper wads and three fecal pieces have separate positions, contact bodies and resistance. Pressure acts on pieces still in the pipe, and a piece's rear edge must cross the outlet marker before it counts as cleared. The drainage model depends on how many pieces have passed. Flexible paper, fecal deformation, water simulation and siphon mechanics need separate validation.
- The 1.00 scores are local CPU prototype results. Wet calibration, the YAM port, Dream-1 runs and submission media remain pending.
