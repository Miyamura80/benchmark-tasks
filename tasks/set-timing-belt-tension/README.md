# Set robot-joint timing-belt tension and secure the motor mount

**Robot instruction:** Set the timing belt to the specified tension and secure the motor mount. Leave the belt seated on both pulleys, then put down the tools and release the assembly.

**Category:** Impact on AI timelines. Setting and retaining belt tension is a mechanical assembly step for belt-driven robot joints. Incorrect preload can make a drive skip teeth or overload shafts and bearings. A final force–deflection check tests the assembled drive under load.

## Fixture and reset

This is an unpowered bench fixture with the pulley axes horizontal in Y. All coordinates are in metres from the YAM station origin, with +X away from the bases and +Y to the left. The tabletop is at z = 0.7475 m.

| Part | Specification and placement |
| --- | --- |
| Base and joint housing | Aluminium base, 224 × 175 × 14 mm, centred at (0.266, 0.004). Fixed 51 × 48 × 38 mm bearing support near the driven pulley. Leave at least one pulley free to rotate. |
| Belt | Gates PowerGrip GT3 300-5MGT-09, 60 teeth, 5 mm pitch and 9 mm width. Thread it over both pulleys before reset. |
| Pulleys | Equal 16-tooth, 5 mm pitch, flanged pulleys. Pitch radius 12.73 mm. Driven axis centred at x = 0.202 and z = 0.7885. Start the motor axis at x = 0.31125. Pulley faces offset 0.10 mm axially. |
| Motor and mount | Unpowered Ø46 × 35 mm motor; 103 × 106 × 11 mm steel plate on two parallel X guides, with 108–112 mm centre-distance travel. Two M4 × 0.7 headed clamp bolts at x = 0.288 and 0.332, y = −0.072. Reset each bolt to a 0.105 mm clamp gap from snug (54° at 0.7 mm pitch). |
| Pull handle | Back-drivable 8:1 bellcrank and 23 × 22 × 25 mm orange hand grip, starting at (0.271, 0.095, 0.7965). It acts on the carriage along X and releases when the gripper lets go. |
| Driver and cradle | 7 mm socket for the M4 hex heads, 53 mm from grip centre to socket tip, with orange T-handle. Start in its 63 × 51 × 16 mm cradle at (0.165, −0.149), at +60° yaw. |
| Test gauge | Fixture-mounted spring probe centred over the upper belt span. Press the blue cap at (0.257, 0.032, 0.8335) to its 10 mm detent. A kinematically linked 10:1 scale shows the loaded measurement in the green band at 1.40–1.95 mm, and retains its reading on release. Reset its follower to zero. |

The specified component and nominal spacing are reproducible. The 110 mm free centre distance is calculated from the pitch geometry (300 mm belt minus an 80 mm pulley circumference, divided by two). The 2.5 N force and 1.40–1.95 mm pass band are **fixture-specific design inputs**, pending physical bench calibration. The [Gates force–deflection procedure](https://www.gates.com/content/dam/documents-library/catalogs/powergrip-gt3-drive-design-manual-en.pdf) supports a perpendicular test at midspan. Its 1/64-of-span reference is 1.72 mm for this fixture. The test applies to this unpowered fixture. Use a straightedge and feeler gauges across the two pulley faces for the alignment check.

## Run

Use Inspect Robots 1.0 and the YAM Arena station from the RoboCurve walkthrough. From this folder, with the Arena environment active:

```bash
BELT_SETUP_PNG=setup.png python run_task.py yam_scripted --log-dir runs/demo --max-steps 650
python run_task.py dream --log-dir runs/dream
python run_task.py yam_scripted --control release-before-tightening --log-dir runs/release --max-steps 650
python controls.py --out runs/controls
python make_evidence.py runs/demo demo.mp4
python make_evidence.py runs/dream dream.mp4 --title 'Dream-1 baseline'
```

The wrapper registers station reset and the task scorer, then calls the standard Inspect Robots runner with `yam_arena` and `task_score`. The run limit is 1,400 steps at 20 Hz (70 simulation seconds). The scripted evidence runs use a 650-step cap. The station seed is 42 and there is no object jitter. Recorded JSON files contain the trial scores, and `mechanism.jsonl` records clamp load, plate position, test state, and hand release at each control step. `controls.py` tests the reduced-order mechanism independently of robot IK. It exports the exact trace for each negative control.

## Revised native runs

The revised native scripted demo scored 1.0 in 595 control steps. It recorded
1.4185 mm on both post-release loads and 0.196 N m of estimated torque per bolt.
The two readings showed zero measured drift. The test estimates torque from
simulated clamp load. The physical fixture still needs calibration.

Dream-1 completed the same revised setup at the 1,400-step limit with a score
of 0.00. Both runs have native video and a setup image.

On the L40S, the scripted run took 3m 39s and Dream-1 took 3m 13s,
including startup.

## Historical trials using the previous scorer

| IsaacLab / YAM trial | Score | Control steps | Wall time | Final result |
| --- | ---: | ---: | ---: | --- |
| Scripted demo | 1.00 | 499 | 4m 36s | Left arm held tension through both right-arm turns. Both bolts reached 245 N. The post-release gauge read 1.4185 mm with zero measured plate drift. |
| Release before tightening | 0.00 | 340 | 4m 03s | Released the unclamped mount at 110.4458 mm centre distance. It fed back to 110.00 mm before the gauge and then reached 108.00 mm under the test. The final reading was 10.6098 mm. |
| Dream-1 | 0.00 | 1,400 | 7m 00s | Step limit. The policy engaged neither the pull handle nor the driver. |

Wall times include startup. The controls below exercise the mechanism and scorer directly.

| Mechanism control | Score | Measured failure |
| --- | ---: | --- |
| Slack | 0.25 | 4.6735 mm deflection with both bolts secured |
| Overtight | 0.25 | 1.0946 mm deflection with both bolts secured |
| Single-bolt slip | 0.00 | 0.05382 mm total slip and only one secured bolt |
| Off motor pulley | 0.00 | Belt unseated, so no valid force–deflection measurement |
| Pulley misalignment | 0.00 | 1.20 mm axial offset |

## Scoring

Use a low-range calibrated torque driver with a 7 mm socket. Both M4 bolts must reach 0.15 N m. After releasing the pull fixture, apply a 2.50 N perpendicular load at the unloaded belt midpoint for at least 0.45 s. Read displacement, unload fully, then repeat. Each reading must be 1.40–1.95 mm and the readings must differ by at most 0.20 mm. For full score also leave the belt seated and aligned (face offset ≤0.50 mm, yaw ≤0.5°), park the driver and clear both hands. A torqued, aligned assembly with two repeatable readings outside the target condition scores 0.25. Otherwise score zero. The model estimates tightening torque using T = 0.20 × bolt diameter × clamp load; the real assessment uses the driver reading.

### Buildable test-fixture options

- Pull lever. Cut the 100 × 16 × 6 mm steel link lever from the [OpenSCAD drawing](pull_lever.scad) and mount it on an 8 mm shoulder bolt in a rigid, double-sided clevis. Drill the pull-grip clevis at a radius of 80 mm and the carriage-link clevis at 10 mm. Place both link holes above the pivot at neutral. Orient the pivot axis parallel to Y, so both link ends move along X at neutral. Use an M4 ball-jointed link to the carriage, a free pivot and end stops at the 108 and 112 mm centre-distance marks. Confirm the unloaded near-neutral 8:1 displacement ratio with a dial indicator and confirm the carriage can back-drive the handle after release.
- Deflection gauge. Use a clamp-mounted 0–5 N push force gauge (0.01 N resolution or finer) on a linear slide with a smooth 6 mm rounded foot, plus a coaxial 0–10 mm displacement indicator (0.01 mm resolution or finer). Align with the free span midpoint. Tare the gauge in its measurement direction, confirm 2.50 N against a calibrated reference force, and zero displacement at unloaded first contact. Use the same setup for both readings. The real test can read the indicator directly and the simulation retains the amplified pointer for visibility.
- Calibrate the torque driver across 0.15–0.20 N m on a suitable bench reference. Trial runs are still needed to calibrate the belt, mount and clamp under load.

## Fidelity limits and bench measurements to collect

The belt is a deforming loop with a reduced-order axial law and midpoint geometry, rather than finite-element rubber or individual tooth contact. Tightening an aligned, seated hex driver rotates each bolt. A 0.7 mm thread lead and 0.105 mm gap convert angular travel to clamp compression, with a 1.9 kN/mm effective stack stiffness. Clamp preload increases friction on the plate. A single bolt can be insufficient under the belt’s axial load. The simulated driver, handle, bolt and probe interactions are geometric contact constraints. The stock fixture's linear rail fixes pulley alignment. The off-pulley and misaligned controls start in injected fault states. Native YAM links collide with the table and base, but collision and contact forces for the custom fixture parts are simplified.

No physical bench measurements were available. Before using this as a hardware specification, sweep centre distance from 109.25 to 110.61 mm, record the probe deflection at 2.5 N, measure the pull force at each held position, and check the plate with a dial indicator through one-bolt, two-bolt and no-bolt release. Compare these measurements with the CSVs from `controls.py`. The assumed 100 N/mm effective span stiffness, clamp stiffness, and 0.29 friction coefficient remain unvalidated.
