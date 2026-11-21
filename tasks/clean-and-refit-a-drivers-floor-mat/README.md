# Clean and refit the driver’s floor mat

Brush dried dirt out of the ribs, vacuum the loosened grit, release both retention clips, tip the remaining grit into the tray, and refit the mat flat below the pedals. This represents routine interior cleaning after shoes track dry dirt into the footwell.

## Nominal reset

Run each trial from a fresh process. The scene starts with both clips seated on the posts, 204 mm apart. The mat is flat and the tray is empty. The scene uses seed 461 for the 36 loose pieces and places 12 bonded clumps in three rows of four across the ribbed heel section. Both YAMs start from the joint reset in `yam_station.py`, with the bristle brush mounted on the left arm and the vacuum on the right. Use the default start with no extra mat rotation or side-entry flag. Angled starts have not been validated.

## Reproduce

Use Python 3.12, NumPy 2.2.6, MuJoCo 3.3.7, DreamLabs 1.0.23, and the DreamLabs YAM mesh assets included with its SDK. Install Pillow, FFmpeg with libx264, EGL, and DejaVu Sans. The Dream-1 runner additionally uses Inspect Robots 1.0.0, dreamlabs-client 1.0.14, and the Inspect Robots Dream adapter at commit b5e936a201cec66c54c297e522c920d98445e339.

```sh
export MUJOCO_GL=egl LIBGL_ALWAYS_SOFTWARE=true OPENBLAS_NUM_THREADS=1
python dual_yam.py results/scripted --video
python dual_yam.py results/control --clean-only --vacuum-off
python evaluate.py --out results/dream --video
```

The scripted controller uses simulator positions to plan joint targets. The Dream-1 policy receives three RGB views and measured joints and gripper openings. Both use the same MuJoCo station and scorer, at 20 Hz with a 1,200-action limit.

Full credit requires both retention clips to have been released, all 12 bonded clumps dislodged, at least 44 of 48 pieces in the tray or vacuum, and the released mat lying flat with both clips seated. Bristle flex, suction, mat bending, and clip engagement are reduced physics models. The vacuum motor is controlled by a simulated reed interlock on actual right-jaw travel. A compliant mat hold requires contact from both finger pads and releases when they open.

For a physical recreation, place a ribbed rubber mat with two retention posts in a fixed footwell fixture, put dry grit and dried dirt in the ribs, and place a shallow tray beside the sill. Check the cleaned ribs and heel by sight and verify that both clips engage with the mat flat and clear of the pedals. Hardware repeatability has not been tested.
