# Install and field-check a magnetic motor encoder

Seat a keyed magnet carrier on an unpowered motor shaft, lower the sensor bracket onto its guides and close the catch. Press the field-test button, release it and clear both grippers.

## Setup

| Part | Material, dimensions and reset |
| --- | --- |
| Motor fixture | Ø80 mm aluminium motor housing on a 265 × 205 × 12 mm base. The shaft axis is at (306, 10) mm. The motor face and shaft end are 77 mm and 92 mm above the table, respectively. The Ø8 mm D-shaft flat faces +X. Start with the shaft bare and power disconnected. |
| Carrier | Orange polymer D-bore carrier with a 20 × 20 mm grasp flange, a centred Ø6 × 2.5 mm diametric magnet and a spring-ball plunger that engages a groove in the shaft. Reset the carrier in the right-hand dock at (179, −148) mm. |
| Sensor bracket | Green PCB, silver bracket and orange insulated grip. Guide posts are spaced 76 mm along Y. Their shoulders and the crossbar stops establish a nominal 1.5 mm magnet-to-sensor gap. Reset in the left-hand dock at (269, 139) mm. |
| Catch | Reset the red cap open at (383, −82) mm. The external cap leaves room for the gripper beside the bracket. Moving it 18 mm toward the shaft and 21 mm down carries the tongue over the bracket tab. |
| Field test | Ø18 mm momentary button at (250, −124) mm with 6 mm vertical travel. Start with the button released and the diagnostic LEDs off. |

Coordinates are from the midpoint of the arm bases, +X away from the robot and +Y to its left. The [fixture drawings, parts and field-check circuit](fixture/README.md) specify the physical build.

## Run

Use the RoboCurve YAM Arena environment with Inspect Robots 1.0. Run from this directory.

```bash
python run_task.py yam_scripted --log-dir runs/demo --setup setup.png
python run_task.py dream --log-dir runs/dream
python run_task.py yam_scripted --case skip-catch --log-dir runs/open-catch
python run_task.py yam_scripted --case missing-magnet --log-dir runs/no-magnet
python run_task.py yam_scripted --case wrong-yaw --log-dir runs/wrong-yaw
python run_task.py yam_scripted --case no-detent --log-dir runs/no-detent
python make_evidence.py runs/demo demo.mp4 --title 'Scripted YAM demo'
python make_evidence.py runs/dream model.mp4 --title 'Dream-1 baseline'
```

The runner uses seed 42, zero object jitter and a 1,400-step limit at 20 Hz.

## Scoring

| Score | Required observation |
| --- | --- |
| 0.25 | The keyed carrier is seated within 0.65 mm of the datum and rests upright with the flat facing +X within 0.20 rad (11.5°). |
| 0.45 | The carrier passes and the bracket is within 3 mm of its seats, upright and aligned with both guides. |
| 0.65 | The bracket passes and the catch has been depressed by contact to at least 90% of its 27.7 mm stroke. The tongue overlaps the tab by at least 3 mm and sits within 3 mm above it. |
| 1.00 | The catch retains the bracket, the field check passes after a 0.30 s press, and the carrier lifts no more than 0.50 mm under a 2.0 N upward proof load for 0.30 s. The key returns and both grippers clear the fixture. |

Native collisions drive grasp, keyed insertion and bracket seating. The 8.50 mm bore has a 3.25 mm locating flat, which overlaps the round side of the 8 mm shaft at a 90° misalignment. The entrance taper gives a lead-in. The shaft ball-plunger detent is a reduced-order model of axial retention, with a 3.2 N closing force over the retained travel. The simulator applies the 2.0 N proof load to the native carrier body and measures its actual displacement. The physical proof check uses a spring scale and a split fork. The bracket catch detent engages after measured robot contact and physical catch travel. Flux and sensor electronics are evaluated by the physical circuit.

## Verified simulation runs

| Policy | Score | Control steps |
| --- | ---: | ---: |
| Scripted YAM | 1.00 | 601 |
| Dream-1 | 0.00 | 1,400 |

The scripted trial seated both parts, engaged the catch by measured finger contact,
passed the field check and cleared both grippers. The carrier moved 0.0043 mm under
the 2.0 N proof load. The wrong-yaw control maintained a misalignment during the
attempted insertion and stopped 4.41 mm above the seating datum.
Removing the detent caused 19.39 mm of lift during the proof test and a final
score of 0.00. Dream-1 reached the step limit without seating the carrier.

The printable parts and diagnostic logic were checked separately. The physical assembly and field circuit still need the bench checks described in the fixture instructions.
