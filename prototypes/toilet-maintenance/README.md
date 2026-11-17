# Toilet maintenance prototypes

The two tasks share a toilet with an open seat ring, raised lid, rear hinges and cistern, plus a four-axis Cartesian gripper. Position actuators command the gripper and its jaws. Free rigid tools move through jaw contact. A fixed camera and a cutaway show the repair in each run.

| Task | Scripted behavior | Measured outcome |
| --- | --- | --- |
| Rim jet | Compare the four collection volumes, scrape limescale from the weak outlet and repeat the rinse. | All four fault locations scored 1.00. The first two-second rinse delivers 200 mL at each healthy jet and 40 mL at the scaled one. Removing all four deposits restores the scaled jet to 200 mL. Misaligned controls score 0.25. |
| Mixed clog | Seat the plunger and clear a compact jam of three wet toilet-paper wads and three pieces of simulated feces. | Eight strokes cleared all six pieces, reducing modeled water remaining from 95% to 1.25%. Seven strokes left a paper wad in the pipe. The off-centre control cleared zero pieces. |

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

- Four recessed openings sit beneath the porcelain lip and direct the rinse inward and down. Each runs independently during both rinses. Four uneven deposits narrow one outlet. MuJoCo welds each deposit to the ceramic on reset. A bond breaks when the scraper applies enough contact force while moving tangentially. The resulting pieces move in the physics simulation. Modeled flow recovers as pieces clear the mouth. Adhesion and flow parameters are prototype values that need wet calibration.
- The plunger moves as a free rigid body. Compression of a diaphragm through cup contact produces a modeled pressure pulse. Three wet paper wads and three fecal pieces start in a staggered, compact jam. The paper uses visible folded layers with irregular edges, carried by rigid contact bodies. The pieces each have their own resistance and a piece's rear edge must cross the outlet marker to count as cleared. The drainage model depends on how many pieces have passed. Flexible paper, fecal deformation, water simulation and siphon mechanics need separate validation.
- The 1.00 scores are local CPU prototype results. Wet calibration, the YAM port, Dream-1 runs and submission media remain pending.

## Dual YAM and Dream-1

Both fixtures also run with two stock I2RT YAM arms and three cameras from Dream's MuJoCo reference. The left gripper physically depresses a blue spring-loaded switch to start each measurement. The right gripper grasps and operates the tool in the nominal scripted trials. The rim controller selects the weak outlet from the first collection. The stock YAM joint and gripper contact geometry is retained. The arms and top camera sit on the same rigid transform relative to the original reference. The switch is 160 mm in front of the bowl centre, 212 mm to its left and 231 mm above the fixture floor. The rim scraper is docked on the centreline at (−245, 0, 123) mm. The trap plunger remains at (−245, −144, 123) mm.

| Nominal scripted trial | Score | Simulated runtime |
| --- | ---: | ---: |
| [Rim jet](yam-results/rim-scripted.json) | 1.00 | 29.0 s |
| [Mixed clog](yam-results/trap-scripted.json) | 1.00 | 26.6 s |
| [Off-target rim control](yam-results/rim-off-target.json) | 0.00 | 29.0 s |
| [Off-centre clog control](yam-results/trap-off-centre.json) | 0.00 | 26.6 s |

The front-right outlet is the submission reset. The alternative front-left and rear-left resets also reached 1.00. The rear-right reset reached 0.25 and still needs scraper-path tuning. Alternate reset evidence is in `yam-results`.

Both policies use the same scene and grader. A model run is limited to 1,200 control steps at 20 Hz, with a deterministic reset and Inspect Robots seed 0. The scripted demos were run on CPU with EGL. The local Dream interface check verified the camera and joint formats. A live Dream-1 run is pending.

Use Python 3.12, FFmpeg, EGL and the DejaVu Sans fonts. Install the Dream wheel without its optional hardware stack. The pinned policy installs the network client separately.

```sh
python -m pip install 'mujoco==3.3.7' 'numpy==2.2.6' 'inspect-robots==1.0.0' Pillow
python -m pip install --no-deps 'dreamlabs==1.0.23'
python -m pip install 'inspect-robots-dream[client] @ git+https://github.com/kairosiann/inspect-robots-dream.git@b5e936a201cec66c54c297e522c920d98445e339'

MUJOCO_GL=egl python yam_trial.py trap --video --out runs/yam-trap
MUJOCO_GL=egl python yam_trial.py rim --video --out runs/yam-rim
MUJOCO_GL=egl python yam_trial.py trap --control --out runs/yam-trap-control

MUJOCO_GL=egl python yam_eval.py trap --out runs/dream-trap
MUJOCO_GL=egl python yam_eval.py rim --out runs/dream-rim
inspect-robots video runs/dream-trap/toilet-yam-trap_*.json --out runs/dream-trap-video
inspect-robots video runs/dream-rim/toilet-yam-rim_*.json --out runs/dream-rim-video
```

Run the Dream-1 commands in the authorised environment with its existing Dream API credential. Each model run records the measured actions, camera frames, final score and request transcript. The wet-bench calibration remains pending.
