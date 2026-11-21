# Spot-treat three dandelions in a lawn

A gardener targets three weeds in a patch of grass using a handheld sprayer. The robot uses the same aiming and squeeze action with a sprayer on its right wrist. Two open flower heads and one closed bud grow on separate rosettes. A bracket holds the sprayer during the run. Scoring uses the visible upper leaf and grass area marked by the spray.

## Run the task

Use Python 3.12, FFmpeg, the packages in requirements.txt and a MuJoCo rendering backend. The pinned DreamLabs package supplies the calibrated I2RT YAM models, wrist cameras and meshes. The scene generates its own lawn and weed geometry at reset. The nominal layout uses seed 0. Dream-1 requires an authorized Dream API credential in the execution environment.

```sh
python -m pip install -r requirements.txt
export MUJOCO_GL=egl
python demo.py --case careful --seed 0 --video --out demo
python demo.py --case miss --out miss
python demo.py --case blanket --out blanket
python demo.py --case idle --out idle
python replay.py demo/commands.json
python verify.py demo/before.png demo/after.png demo/labels.png
python evaluate.py --steps 600 --video --out model-run
```

The scripted trajectory takes 166 actions (8.3 simulated seconds). Dream-1 gets 600 actions (30 simulated seconds). Both drive the same native joint targets and jaw actuators and call the same terminal scorer. Only the scripted demonstrator receives the known crown placements. Dream-1 receives RGB from an overhead camera and two wrist cameras, measured joints and the natural-language instruction. Leaf masks and the diagnostic deposition display are evaluator-only. The clean reference and UV-style images in the evidence folder are simulated. A full real-camera workflow, including plant sourcing, registration, blind calibration and unreadable-trial handling, is in BENCH.md.

## Scoring

The input is a registered clean UV image, a registered final UV image, and an evaluator's overhead weed mask made before treatment. A pixel is newly marked when its green-excess signal increases by at least 35 on an 8-bit image. Each weed's fraction of newly marked pixels must reach 0.30. Newly marked pixels in the rest of the turf must total at most 0.45 times the combined visible weed area. A 1 mm square grid represents the simulated surface. Use fractions of matched pixel areas at any fixed photographic resolution. Measurements refer to the tray. Deposits beyond it, leaf undersides, herbicide dose and long-term efficacy are outside scope. The proposed physical thresholds need calibration.

The sprayer fires once after measured jaw closure below 16 mm of individual finger slide travel. It rearms above 32 mm. Each squeeze emits 1,800 sampled rays around the actual moving nozzle axis (two transverse normal components with standard deviation 0.17). Each ray marks one overhead cell at its first crossing of the stepped leaf canopy/grass surface. This is a footprint approximation. Nozzle fan, droplet size, trigger mechanics, runoff, wind and leaf motion have not been calibrated on hardware. The bracket supports the sprayer throughout. No tool pickup is required.

## Measured results

Across three independent simulated layouts, careful spot treatment passed 3/3. Deliberately missing a weed, spraying the lawn in a grid, and staying idle each passed 0/3. On the nominal tray the careful run marked 47.7%, 51.7% and 51.5% of the three weeds. Grass spill was 22.6% of the combined weed area. The blanket pass marked 36.9%, 31.3% and 41.6% of the weeds, yet its grass spill was 388.4%. Replaying the careful joint commands from reset reproduced the same fractions and full score.

Dream-1 ran for all 600 actions in one session (12 requests, one attempt each) on the nominal layout. It fired once. Weed coverages were 0%, 0% and 16.0%, with grass spill of 28.3%. Final score was 0.0. Independent joint replay and photo scoring reproduced this result.

## Controls and scope

The missed-weed control deliberately skips weed 3. The blanket control sweeps a 4 × 3 grid, including grass between weeds. The idle control leaves the jaws open. Seeds 1 and 2 vary crown positions, leaf outlines and droplet sampling on fresh resets. See evidence/controls.json in this task directory. The synthetic camera tests also check weak references, saturation and unconfirmed alignment. They establish software behavior. They do not establish physical repeatability.

In a 2017 Northern Virginia regional survey, 32% of primary lawn and garden caregivers reported spot-treating weeds with weedkiller (Q13, report p. 39). [Regional survey](https://www.fcps.edu/sites/default/files/media/pdf/2016-17annualreport.pdf)

Spot treatment of specific lawn weeds is a recognizable household use case. University of Minnesota Extension discusses targeted treatment of weeds in established lawns. [Lawn care guidance](https://extension.umn.edu/garden-and-home/yard-and-garden/lawns-and-landscapes-in-minnesota/fertilizing-lawns)

BEHAVIOR-1K includes spraying both potted plants and applying chemicals across a lawn. This task requires selecting three individual weeds and limits the visible spray on grass. Our blanket control covered every weed on the nominal layout and failed the grass limit. [Spraying for Bugs](https://behavior.stanford.edu/challenge/tasks/38_spraying_for_bugs.html) · [Applying Pesticides](https://behavior.stanford.edu/knowledgebase/tasks/applying_pesticides-0.html) · [Adding Chemicals to Lawn](https://behavior.stanford.edu/knowledgebase/tasks/adding_chemicals_to_lawn-0.html)

## Evidence layout

The evidence subdirectory contains the executed joint commands and results for both trajectories, the three-layout control summary, synthetic photo checks and model-request summary. Replay either command sequence from the nominal reset using `python replay.py evidence/demo-commands.json` or `python replay.py evidence/dream-commands.json`. To regenerate inputs for the camera-control checks, run demo.py once per case with --out checks/CASETYPE, where CASETYPE is careful, miss, blanket or idle and matches --case, then run python check_bench.py checks. The videos and setup screenshot accompany board media M-KPU6QB.
