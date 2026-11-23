# Break down a sealed carton for recycling

Turn over the empty carton, cut its bottom tape with the guarded cutter, park the cutter, unfold all four bottom flaps and leave the intact carton flat. All eight flaps stay attached. The west inner flap has a 28 x 10 mm rectangular finger notch at its free edge; it is present at reset in both trials.

## Fixed setup

Use an empty 220 x 180 x 180 mm regular slotted carton made from 3 mm corrugated cardboard, with an areal density of 450 g/m² (about 126 g total). Its four top flaps are already folded outward to 270 degrees. All four bottom flaps start closed. One 48 mm polypropylene packing-tape strip seals the 220 mm bottom centre seam and wraps 40 mm up both end walls. There are no pre-cut tape paths. Place the carton upright at world XY (250, -100) mm on the YAM station table, whose surface is Z=747.5 mm. The walls align with the world XY axes; `config.py` and `scene.py` give every panel pose and size.

The 55 g plastic-bodied guarded cutter with a steel blade starts at (130, 420, 767.5) mm on a 170 x 70 x 2 mm flat polymer pad. It has a 100 x 25 x 25 mm handle, a 20 x 30 x 20 mm guard and a 14 x 0.5 x 8 mm blade. Use two standard YAM arms with their stock parallel jaws. No carton fixture, handle, external force or object-pose controller is used.

For a physical reset, use a matching carton, refold its bottom flaps, apply fresh tape and return the cutter to its pad. Replace torn or permanently damaged cardboard. Use a guarded carton cutter and keep people outside the robot workspace while it moves.

## Scoring

Scores are exclusive milestones: 0 before complete valid tape cutting; 0.25 after cutting all three bottom tape paths; 0.50 after also opening each bottom flap outward to at least 150 degrees; 1.00 after also leaving the carton flat and released for one second. A measured inverted hold within 5 degrees of upside down for at least 0.5 seconds must precede cutting. Turnover alone earns no credit.

Full score requires all eight flaps within 15 degrees of coplanar with their walls, every cardboard vertex within a horizontal slab no thicker than 24 mm, the highest vertex at most 30 mm above the table, and the lowest between -2 and +8 mm. Carton panel translation must be below 10 mm/s; both tool centres must be at least 70 mm outside the carton bounding box and 65 mm from the cutter. The cutter must be within 50 mm of its reset location and moving below 20 mm/s.

The simulation samples all three tape-cut paths at 2 mm intervals (158 cells). Coverage counts only a held, aligned blade at the tape plane moving no more than 15 mm per 20 Hz sample. In the physical task, inspect the entire centre seam and both taped end folds for a continuous cut; partial cuts do not earn 0.25. Use a ruler, overhead and side video, and printed angle references for opening and flatness checks. No tape-cell instrumentation is required on the real carton.

Torn/disconnected creases (more than 4 mm separation), opening a sealed outer flap more than 12 degrees before cutting, missing angle evidence, inward bottom-fold motion beyond the scorer tolerance, or a carton/cutter drop more than 30 mm below the tabletop invalidate the trial. Continuous 240 Hz native angle tracking prevents a wrapped inward angle from counting as outward opening. Scoring reads native states, independent of policy phases.

## Demonstrated result

The scripted trial started upright and sealed, cut 158/158 tape cells, opened all four bottom flaps and reached 1.00 at 399.15 simulated seconds. It ended after 7,998 controls / 399.90 seconds with no controller failure. Final carton height span: 18.61 mm. The full release condition held for 1.75 seconds. An independent scorer replay matched all 7,998 logged controls. `evidence/result.json` records the score and source fingerprints. The demonstration video is a replay of measured native actor poses from that continuous trial; it contains no invented object motion or interpolated trajectory.

Both policies have a 12,000-control limit at 20 Hz (600 simulated seconds). Source and action prefix for the successful demonstration are included. The model result and media ID are recorded in `task.json` and `evidence/model-result.json` after the model trial.

## Runtime and commands

Linux, NVIDIA RTX GPU, Python 3.12. Isaac Lab-Arena commit `c7edf3a9d5d8276565208f72422f8f8a0806d6cf` (release/0.3.1), Isaac Lab commit `91a0906f93802eb859192dcb0bca879f891e92d6`, Isaac Sim 6.1, YAM Arena commit `0059d02a1a91bc6834c1d0846022891df0c78730`, and Inspect Robots Dream commit `b5e936a201cec66c54c297e522c920d98445e339`. Install the pinned YAM Arena checkout with its `setup.sh`, then source the resulting `yam.env`. See the upstream setup documentation for system requirements. Trial output directories must not already exist.

```sh
source /path/to/IsaacLab-Arena/yam.env
export CARTON_LEVEL_SEAM_EDGE=1 CARTON_LEVEL_FOLD_EDGE=1
export CARTON_SEAM_YAW_RAD=1.0 CARTON_CUTTER_GRIP_MM=8
export CARTON_GRASP_SITE=tip CARTON_SEAM_DEPTH_MM=-1.35
export OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
xz -dk sealed-three-flap-actions.jsonl.xz
python -u run.py --out scripted-trial --mode scripted-demo-probe \
  --tapered-flap-hook --elevated-pickup --blade-tilt --light-tape-pressure \
  --level-right-pad --tip-lift --compliant-push \
  --third-flap-action-prefix sealed-three-flap-actions.jsonl --west-finger-notch
```

The prefix replays ordinary joint commands from a fresh reset through the third flap opening. Physics, tape cutting, angle integrity and scores are recomputed throughout; it does not load object states. The remaining controller uses measured native feedback. GPU/contact variation can affect reproducibility; the supplied sealed-start evidence is the demonstrated trial.

For Dream-1, configure `DREAM_API_KEY` through the SDK's supported credential mechanism and run:

```sh
python -u run_dream.py --out model-trial --mode scripted-demo-probe \
  --tapered-flap-hook --elevated-pickup --blade-tilt --light-tape-pressure \
  --level-right-pad --tip-lift --compliant-push --west-finger-notch
```

The model command accepts no recorded action prefix. Dream-1 receives only the overhead/wrist images, measured 14D joints and instruction. `run_dream.py` reuses the verified scene and scorer, replacing policy dispatch and output labels; `run.py` remains byte-identical to the verified source. Model calls pause the simulation while awaiting replies. Native actor traces and model request transcripts are written in the trial folder. Runtime depends on first-launch shader compilation, GPU and model response latency.

## Simulation fidelity

The carton uses twelve finite-thickness rigid cardboard panels and twelve physical revolute crease joints; it does not simulate continuous bending, crushing or tearing. Bottom crease drag is a passive angular damper with a torque cap scaled to each flap's mass and lever arm, with zero spring stiffness. The notch geometry is collision-enabled. Tape is a discrete restraint released after measured complete blade coverage; peel adhesion, tape-film deformation and cutting forces are not modelled. The visual tape remains a surface marking. Bottom/corner adjacent-panel joint collision is disabled; independent board and robot contact remain physical. Crease separation and tabletop penetration are scored explicitly. These approximations and the notched flap must be retained when comparing the two policies or reproducing the setup.

## Submitted model trial and media

Dream-1 beta scored **0.00** after the full **12,000 controls / 600 simulated seconds**. All 250 inference requests completed; there was no model or native-scoring failure. It did not turn over the carton or cut any tape. Wall runtime was **31.7 minutes** on one RTX PRO 6000 Blackwell. An independent scorer replay matched **12,000/12,000** logged native score samples. The H.264 model recording contains exactly 12,000 frames at 20 fps. The scripted demonstration remains **1.00**.

Board media **M-DGS96R** contains the complete model video, full-score native-state demonstration replay, and sealed-start setup screenshot. `evidence/model-result.json` records the score, model checkpoint, replay count and content hashes. `evidence/runtime-versions.json` records the installed runtime. The 43 focused scoring/angle/crease tests passed.
