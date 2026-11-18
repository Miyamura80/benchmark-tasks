# Restore flow through a limescale-blocked toilet rim jet

Run a rinse, identify the weak outlet from the four collections, scrape away the limescale and repeat the same rinse. This task measures restored flow at one recessed outlet while the other three remain in range.

## Starting layout

The fixture has a ceramic-look bowl, a hinged polymer seat ring and raised lid, and a cistern. Use the bowl centre as the origin in X and Y. The floor is Z = 0. Four independent outlets sit at Z = 211 mm with X and Y coordinates ±105.6 mm and ±79.2 mm. In reading order they are front left (−X, +Y), front right (−X, −Y), rear left (+X, +Y), and rear right (+X, −Y). The outlet mouths are 7.6 mm across and point inward and downward.

The front-right outlet starts with four rough bonded chips. A steel-edged scraper rests in the centreline dock with its handle at (−245, 0, 123) mm. The model code defines the remaining geometry and material approximations. A complete two-second rinse updates the four volume readings. The reset's fault index is private. The observations include the three YAM camera views and joint state.

## Reproduce and score

Two I2RT YAM arms with stock parallel jaws sit on bases at (−250, ±305, 175) mm. Both start with grippers open. The overhead camera and both wrist cameras feed Dream-1. The blue spring-loaded test switch sits at (−160, 212, 231) mm. Finger contact and at least 10 mm of physical switch travel trigger each measurement. The tool must remain clear of the bowl while the test runs.

Use Python 3.12, FFmpeg, EGL and DejaVu Sans. Install the robot scene, evaluator and Dream client.

```sh
python -m pip install -r requirements.txt
python -m pip install --no-deps 'dreamlabs==1.0.23'

MUJOCO_GL=egl python yam_trial.py rim --video --out runs/scripted
MUJOCO_GL=egl python yam_trial.py rim --control --out runs/control
MUJOCO_GL=egl python yam_eval.py rim --out runs/dream
inspect-robots video runs/dream/toilet-yam-rim_*.json --out runs/dream-video
```

Set the Dream API credential in the authorised environment before running the model. Both runs use the same workcell and scorer. The model uses seed 0 with a deterministic reset and 1,200 control steps at 20 Hz. The scripted demo takes 29.0 seconds of simulated time. Wall time varies with rendering and inference.

The two-second open-flow reference is 200 mL per outlet. In the nominal demo the front-right outlet starts at 40 mL. All four chips detach and all four outlets finish at 200 mL. A no-contact control scores zero. See the [scripted result](evidence/scripted.json) and [off-target control](evidence/control.json).

The [Dream-1 baseline](evidence/dream.json) scored 0.00 after 1,200 steps. It used 25 model requests in 101.4 seconds of wall time. The baseline and scripted video are both supplied.

## Physical reset and measurement

See [wet-bench protocol](fixture/README.md) for a clear-flow baseline, reset tolerances, CAD and the physical scorer. The rig and deposit adhesion parameters still need wet calibration. MuJoCo models contact between the gripper, scraper and bonded chips. The displayed water and per-outlet discharge use a reduced-order model.
