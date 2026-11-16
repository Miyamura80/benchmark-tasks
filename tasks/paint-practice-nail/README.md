# Paint the middle nail on a silicone practice hand

**Instruction:** Paint the middle fingernail on the silicone practice hand red. Keep the silicone around that nail clean.

**Category:** Impact on jobs. Applying a clean coat is a discrete manicure service. A fixed silicone hand isolates brush control and edge precision from hand tracking. [Clockwork’s robot manicure](https://developer.nvidia.com/blog/?p=32624) is a commercial example of nail coating as a standalone task. The [U.S. Bureau of Labor Statistics](https://www.bls.gov/news.release/ocwage.t01.htm) counted 152,770 manicurist and pedicurist jobs in May 2025.

## Setup

Run from a RoboCurve YAM station with Inspect Robots 1.0 and the YAM Arena package pinned at `0059d02a1a91bc6834c1d0846022891df0c78730`. The task uses both YAM arms with parallel grippers. The script uses the left arm. Any policy can use either.

| Object | Dimensions | Fixed starting pose |
| --- | --- | --- |
| Dark gray plastic manicure rest | 163 × 113 × 6 mm | Centre at (289, 9) mm |
| Silicone practice hand | Approximately 144 × 102 × 20 mm, five digits and individual artificial nails | Middle nail centred at (235, 0) mm. Wrist extends in +x, fingers point in −x. |
| Middle nail | 17 × 12 mm rounded outline, shallow transverse crown | Glossy pale nail on the longest finger. The other four nails retain their natural colour. |
| Preloaded red applicator with foam tip | 27 × 24 × 38 mm, 23 g, 5.6 mm effective tip diameter, 4 mm compliant extension | Upright at (163, 175) mm with its base on the table |

Coordinates are world x and y measured from the station origin. The arm bases are at y = +305 mm and y = −305 mm, so the middle nail is 385.03 mm from each base in plan view. The table is at z = 747.5 mm. The nail's contact surface is 24 mm above the table. Objects reset to the same pose every run. Red transfer is cleared at reset.

## Run and score

From the task folder with the Arena environment active:

```bash
python run_task.py dream --log-dir runs/dream
NAIL_SETUP_PNG=setup.png python run_task.py yam_scripted --log-dir runs/demo
```

Each trial allows 2,400 control steps at 20 Hz, or 120 s of simulated control. The checkout pins the station cameras to 320 × 176 for both policies.

The scorer records brush sweeps on a 0.75 mm surface grid. Transfer requires an upright, gripped applicator with its modelled tip within 1.5 mm of the contact plane and consecutive contact samples during a stroke. Static contact or a jump of more than 4 mm per sample does not deposit polish. A trial scores 1.0 when at least 85% of the nail is covered and skin spill covers at most 2% of the nail's area, held for 0.3 s. Otherwise the final score is the covered fraction times max(0, 1 − 4 × spill fraction), capped at 0.99. Both fractions use the nail area as denominator. Run with `NAIL_TRACE=1` to log coverage and spill fractions per control step.

## Fidelity

The simulator uses rigid contacts and a fixed-width geometric transfer model. The compliant tip is an analytical 4 mm extension below the rigid handle. The scripted demo replays the fixed sequence of 14-axis joint targets in `scripted_actions.json`. Playback reads no simulator object poses or success signals. Simulator state is used separately by the scorer to calculate brush contact and paint coverage. The sculpted hand is fixed. Rounded visual surfaces represent the hand and its five nails. Hidden box colliders approximate the fixed anatomy. The scorer uses the middle nail’s projected outline and the original planar contact tolerance. It measures spill within the adjacent middle-finger patch. Polish colour remains on contact until reset. The model omits bristle flex, pressure-dependent flow, drying, and deforming skin. A physical replication uses a silicone practice hand and a foam-tip applicator.

## Evidence for submission

Use the same scene and starting pose for the model, scripted demo, and screenshot. Convert completed run logs with `inspect-robots video <run.json>`. Upload a scored model clip and a setup screenshot via the benchmark repository's `tools/upload_media.py`. Include the scored scripted clip when the model falls short of full score. The uploader writes the board media ID to `task.json`. Submit the task folder as one pull request after checking the live board for duplicates.

The hand mesh ships with the task. Regenerate it with `blender -b --python build_hand_mesh.py` if editing the hand proportions.

## Verified runs

| Policy | Score | Steps | End condition |
| --- | ---: | ---: | --- |
| Dream-1 | 0.0 | 2,400 | Step limit, no trial errors |
| Fixed joint-target replay | 1.0 | 448 | 85.40% coverage, 1.10% spill |
