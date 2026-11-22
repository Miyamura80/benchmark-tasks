# Dual-YAM floor mat

Brush loose the dirt stuck to the mat, vacuum the ribs and the heel below the pull lip, then remove, empty, and refit the mat with both clips seated. The scripted replay uses native MuJoCo joint actuators, two opaque YAM arms, and an exterior camera.

The left arm carries a passive bristle brush. At every physics step, sum contact forces from penetrating or touching bristles on each bonded clump. Resolve the net load in the mat's tangent plane. At 1.00 N or more the bond opens. The result log names the clump and records the force and time.

The right arm runs the vacuum with its normal gripper-opening command. A simulated reed interlock detects actual right-jaw travel between 31 and 37 mm and powers the motor during that window. The tool indicator and video overlay show its state. Closing the hand stops suction for the mat transfer. The tool still uses the same bounded 44 mm suction-force model, throat crossing, and compliant bin capture. A mat hold forms only after both pads touch the rubber lip and releases when the fingers open.

The replay dislodged all 12 stuck clumps, vacuumed 26 pieces, and emptied 18 into the tray. It met the 44-of-48 collection threshold and left both clips seated, the mat flat, and the motor off at 57.7 seconds. The score was **1.0**. A below-threshold control dislodged 12 clumps and vacuumed zero pieces. Score thresholds and clip tolerances are unchanged.

Install the RoboCurve YAM SDK with `dream_examples.yam_mujoco` and its mesh assets, plus MuJoCo, NumPy, Pillow, FFmpeg and DejaVu Sans.

```sh
MUJOCO_GL=egl python dual_yam.py ./results --video
MUJOCO_GL=egl python dual_yam.py ./vacuum-off --clean-only --vacuum-off
```

The JSON trace reports the motor state, the bristle shear forces at release, and the final socket errors. The controller uses simulated crumb and socket positions for short follow-up moves. These runs validate a scripted replay.
