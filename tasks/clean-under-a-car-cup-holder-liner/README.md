# Clean under a car cup-holder liner

Lift the rubber liner out by its tab, scrub the sticky residue with the sponge, then put the liner back flat in the cup holder.

The trial starts with a two-well automotive centre console on the work surface. The target is a 94 mm bore and a 74 mm ribbed insert with a rubber pull loop. Uneven dried liquid splashes cover nine regions on the floor under the liner. A handled 27 × 23 mm foam pad starts in the console's utility pocket.

## Reproduce

From this task directory, use Python 3.12, EGL, FFmpeg with libx264, and DejaVu Sans fonts. Install the pinned packages, then generate the demonstration, controls and reviewer stills:

    python -m pip install -r requirements.txt
    export MUJOCO_GL=egl LIBGL_ALWAYS_SOFTWARE=true OPENBLAS_NUM_THREADS=1
    python run.py cup-holder --out evidence/cup --video
    python run.py cup-holder --out evidence/cup-idle --control
    python run.py cup-holder --out evidence/cup-no-refit --detail-control
    python run.py cup-holder --out evidence/cup-air --air-wipe-control
    python run.py cup-holder --out evidence/cup-replay --replay evidence/cup/commands.json
    python review.py --commands evidence/cup/commands.json --out evidence/cup/review

The trial is limited to 1,200 control steps at 20 Hz. The full reference takes 567 steps (28.35 s). The saved actions replay at 1.00. The idle control scores 0.00. Sweeping 35 mm above the floor while otherwise following the same sequence scores 0.25, and cleaning all nine regions without refitting scores 0.75.

The reviewer sheet is captured during the verified native replay. At 7.50 s the well contains all nine marks. At 20.20 s the well is clean and the liner is still out. At 28.35 s the liner is refitted with 0.32 mm horizontal error, 0.21 mm height error and no detectable tilt at 0.01° precision.

## Score

- 0.25 after grasped liner removal.
- 0.50 after at least four of the nine splash regions are scrubbed.
- 0.75 after all nine are scrubbed.
- 1.00 after cleaning and refitting the liner, with at least 0.35 s at rest in its seat. Position thresholds are 9 mm horizontally and 3 mm vertically, with at most 13° of pitch or roll and an aligned pull loop.

Removing a region requires 3 mm of equivalent sliding while the pad overlaps the stain in all three axes. Floor force must read at least 0.10 N. Pressure scales stain wear up to the full rate at 0.60 N. The layered stain fades with measured tool travel. The rigid insert and sponge approximate compliant materials, and a bounded two-finger hold assists the grasp only after two opposed contacts. The smooth console and liner skins render over a simpler compound collision geometry. Liquid transport and foam deformation are approximated by the contact-driven stain model.

## Dream-1 replay

The evaluation used Python 3.12, MuJoCo 3.3.7, NumPy 2.2.6, Inspect Robots 1.0.0,
Dream Labs client 1.0.14, and Dream Labs SDK 1.0.23. The Dream policy adapter was
pinned to commit `b5e936a201cec66c54c297e522c920d98445e339` of
`kairosiann/inspect-robots-dream`. Set `DREAM_API_KEY` in the runtime environment
and run:

    MUJOCO_GL=egl python evaluate.py cup-holder --out evidence/dream --video

The attached videos show the same MuJoCo station and are labeled by controller.
The model-run score is measured by the task scorer at the final action.
