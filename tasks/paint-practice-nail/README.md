# Paint the middle nail on a silicone practice hand

**Instruction:** Paint the middle fingernail on the silicone practice hand red. Keep the silicone around that nail clean.

**Category:** Societal relatability. Painting a nail is part of an everyday home manicure and tests control of a loaded applicator at a small curved boundary.

## Repeatable physical setup

Use an empty 5.6 mm flat foam micro-applicator with a 4 mm exposed tip and a rigid 27 × 24 × 38 mm gripper sleeve. Use an opaque red water-based peel-off polish. For each trial load 20 µL with a pipette onto the flat face, allow 10 seconds for uptake, then start within 30 seconds. Stand the handle on the two dock rails so the foam hangs free. Keep polish below the sleeve; wrap each gripper pad in one strip of removable nitrile film and replace if soiled.

Photograph the clean nail and surrounding skin with the fixed overhead camera, a millimetre scale and constant lighting. After the trial, use the same crop and count red pixels inside and outside the baseline nail mask. Peel the dried polish from the practice nail, wash the nail and silicone with water and neutral soap, dry them, and confirm the clean reference before loading a new applicator. The prior measurements below come from the proximity-scored revision.

## Setup

Run from a RoboCurve YAM station with Inspect Robots 1.0 and the YAM Arena package pinned at `0059d02a1a91bc6834c1d0846022891df0c78730`. The task uses both YAM arms with parallel grippers. The script uses the left arm. Any policy can use either.

| Object | Dimensions | Fixed starting pose |
| --- | --- | --- |
| Dark gray plastic manicure rest | 163 × 113 × 6 mm | Centre at (289, 9) mm |
| Silicone practice hand | Approximately 144 × 102 × 20 mm, five digits and individual artificial nails | Middle nail centred at (235, 0) mm. Wrist extends in +x, fingers point in −x. |
| Middle nail | 17 × 12 mm rounded outline, shallow transverse crown | Glossy pale nail on the longest finger. The other four nails retain their natural colour. |
| Preloaded red applicator with foam tip | 27 × 24 × 38 mm, 23 g, 5.6 mm effective tip diameter, 4 mm compliant extension | Upright at (163, 175) mm on two 4 mm-high dock rails with the tip clear |

Coordinates are world x and y measured from the station origin. The arm bases are at y = +305 mm and y = −305 mm, so the middle nail is 385.03 mm from each base in plan view. The table is at z = 747.5 mm. The nail's contact surface is 24 mm above the table. Objects reset to the same pose every run. Red transfer is cleared at reset.

## Run and score

From the task folder with the Arena environment active:

```bash
python run_task.py dream --log-dir runs/dream
NAIL_SETUP_PNG=setup.png python run_task.py yam_scripted --log-dir runs/demo
```

Each trial allows 2,400 control steps at 20 Hz, or 120 s of simulated control. The checkout pins the station cameras to 320 × 176 for both policies.

The scorer records brush sweeps on a 0.75 mm surface grid. Transfer requires an upright, gripped applicator with consecutive measured tip contacts with the nail or neighbouring middle-finger silicone during a stroke. Static contact or a jump of more than 4 mm per sample does not deposit polish. A trial scores 1.0 when at least 85% of the nail is covered and skin spill covers at most 2% of the nail's area, held for 0.3 s. Otherwise the final score is the covered fraction times max(0, 1 − 4 × spill fraction), capped at 0.99. Both fractions use the nail area as denominator. Run with `NAIL_TRACE=1` to log coverage and spill fractions per control step.

## Fidelity

The simulator uses rigid contacts and a fixed-width geometric transfer model. The 4 mm foam extension is a rigid collision body shape. The scripted demo replays the fixed sequence of 14-axis joint targets in `scripted_actions.json`. Playback reads no simulator object poses or success signals. Simulator state is used separately by the scorer to calculate brush contact and paint coverage. The sculpted hand is fixed. Rounded visual surfaces represent the hand and its five nails. Hidden box colliders approximate the fixed anatomy. The scorer uses the middle nail’s projected outline and native contact reports. It measures spill within the adjacent middle-finger patch. Polish colour remains on contact until reset. The model omits bristle flex, pressure-dependent flow, drying, and deforming skin. A physical replication uses a silicone practice hand and a foam-tip applicator.

## Evidence for submission

Use the same scene and starting pose for the model, scripted demo, and screenshot. Convert completed run logs with `inspect-robots video <run.json>`. Upload a scored model clip and a setup screenshot via the benchmark repository's `tools/upload_media.py`. Include the scored scripted clip when the model falls short of full score. The uploader writes the board media ID to `task.json`. Submit the task folder as one pull request after checking the live board for duplicates.

The hand mesh ships with the task. Regenerate it with `blender -b --python build_hand_mesh.py` if editing the hand proportions.

## Historical runs before the contact revision

| Policy | Score | Steps | End condition |
| --- | ---: | ---: | --- |
| Dream-1 | 0.0 | 2,400 | Step limit, no trial errors |
| Fixed joint-target replay | 1.0 | 448 | 85.40% coverage, 1.10% spill |
