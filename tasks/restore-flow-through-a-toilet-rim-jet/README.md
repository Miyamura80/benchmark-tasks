# Restore flow through a limescale-blocked toilet rim jet

Run a rinse, identify the weak outlet from the four collections, scrape away the limescale and repeat the same rinse. This task measures restored flow at one recessed outlet while the other three remain in range.

## Starting layout

The fixture has a ceramic-look bowl, a hinged polymer seat ring and raised lid, and a cistern. Use the bowl centre as the origin in X and Y. The floor is Z = 0. Four independent outlets sit at Z = 211 mm with X and Y coordinates ±105.6 mm and ±79.2 mm. In reading order they are front left (−X, +Y), front right (−X, −Y), rear left (+X, +Y), and rear right (+X, −Y). The outlet mouths are 7.6 mm across and point inward and downward.

The front-right outlet starts with four rough bonded chips. A steel-edged hand scraper rests at the front-left dock and the single Cartesian gripper starts open above it. The scraper handle centre is (−245, −144, 123) mm. The gripper starts at (−245, −144, 265) mm. The model code defines the remaining geometry and material approximations. A complete two-second rinse updates the four volume readings. The alternate resets put the same obstruction in one of the other outlets. The policy receives the test readings and workcell camera. The reset's fault index is private.

## Reproduce and score

Use Python 3.12, FFmpeg, EGL and the DejaVu Sans fonts. Install the dependencies with `python -m pip install -r requirements.txt`.

```sh
MUJOCO_GL=egl python bench.py rim --blocked 1 --video --out runs/scripted
python bench.py rim --blocked 1 --control --out runs/off-target
MUJOCO_GL=egl python inspect_trial.py rim --blocked 1 --smoke --out runs/smoke
MUJOCO_GL=egl python inspect_trial.py rim --blocked 1 --model "$MODEL_ID" --out runs/model
inspect-robots video runs/model/toilet-rim_*.json --out runs/model-video
```

Use the model provider's environment credential. For an OpenAI-compatible endpoint, add `--base-url "$MODEL_BASE_URL" --api-key-env NAME_OF_KEY_VARIABLE`. The Inspect Robots seed is 0 and the scene reset is deterministic. Set `--blocked` to 0, 1, 2 or 3 in reading order. Native runs produce `result.json`, a 25 Hz trace, and optional H.264 video. The scripted nominal trial lasts about 15 seconds of simulated time. Each model trial is bounded by 60 seconds of physical simulation and 100 model calls. Wall time depends on inference and rendering.

The reference is 200 mL per outlet in a two-second rinse. The nominal demo starts at 40 mL on the weak side, reaches 200 mL on all four sides, and clears the four chips. See [scripted measurements](evidence/scripted.json). The native off-target control earns diagnosis credit only. Inspect Robots requires physical contact at the restricted outlet for partial credit, plus a second valid rinse with the tool clear. Full credit also requires clearing all four deposits and restoring all four outlets to 160–240 mL. The two-rinse no-contact smoke check should score zero.

## Physical reset and measurement

See [wet-bench protocol](fixture/README.md) for a clear-flow baseline, reset tolerances, CAD and the physical scorer. The rig and deposit adhesion parameters still need wet calibration. MuJoCo models contact between the gripper, scraper and bonded chips. The displayed water and per-outlet discharge use a reduced-order model.
