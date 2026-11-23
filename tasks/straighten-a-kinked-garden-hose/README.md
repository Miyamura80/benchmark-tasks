# Restore water flow through a kinked garden hose

Straighten the pinched section of a running hose and release it. Both couplers stay fixed. The task ends on full credit or after 21 seconds.

## Build the bench

Use 12 mm ID × 16 mm OD clear, flexible, unreinforced PVC water tubing (for example [Harfington PVC tubing](https://www.harfington.com/en-gb/products/p-1875976), select the 12 × 16 mm size). Cut 510 mm. Seat 15 mm on each of two 12 mm hose barbs and secure with clamps. Set the exposed centerline length between the barb faces to 480 ± 2 mm. Mark the crease 100 mm along the tube from the inlet face. Fit a thin, loose identification band approximately 60 mm downstream of the crease, directly over the grasp point. Check that the band leaves open flow unchanged.

Use a horizontal 300 × 344 mm working surface on a 300 × 368 mm stand. In the plan below, the front-left corner is (0, 0), x runs right and y runs toward the back. Set each barb axis 8 mm above the surface. At the inlet the hose runs left from the barb face. At the outlet the hose runs right into the barb. Extend the rigid discharge pipe past the right edge and aim it down into a 2 L jug on a 1 g scale. The spout is at (336, 115) mm in plan, about 10 mm below the board surface. Keep the board and robot dry.

Supply room-temperature water (20 ± 2 °C) from a constant-head reservoir with its free surface 1.00 ± 0.02 m above the spout. With the tube open in a broad curve between the same fixed couplers, adjust an upstream valve for 500 ± 25 g in five seconds. Record reservoir height, valve setting, tube manufacturer and size. Leave the supply and spout fixed throughout calibration and trial.

## Create and check the starting kink

1. With the tube open and hands away, collect three separate 5.00 s net water masses. Use their median for the open reference. For a continuous stream, switch between a waste jug and a tared weighing jug at each interval boundary from below the spout.
2. At the crease mark, fold the tube back around a smooth 6 mm diameter pin, hold the reverse fold for 10 s, remove the pin and arrange it on the board. The table gives the nominal simulated starting centerline, rounded to the nearest millimetre. Mark these points on the board. Use them as a placement guide, with no clips or pins remaining on the working length.

   | Position along tube | Board coordinates (x, y), mm |
   |---|---|
   | Inlet face to crease | (282,246), (252,246), (217,246), (182,246) |
   | Immediately after crease | (176,230), (171,214), (168,179) |
   | Return loop | (140,159), (109,139), (88,107), (97,66), (140,61) |
   | Return to outlet face | (168,92), (197,115), (234,115) |

3. Release the hose. Check that the first segment after the crease turns 68–74° relative to the entering segment in plan. Verify that the kink holds for five seconds with both hands clear and that water collected during those five seconds is at most 20% of the open reference. If it slips open, repeat the reset or replace the tube. Photograph the working reset with a ruler and record the kinked mass. The proposed physical build and reset still require a bench trial.

## Score the repair

Start the clock with the supply running and the kink verified. Clear means both tool centres stay at least 70 mm from the nearest point on the entire hose centerline, and neither arm touches the tube. In the simulation both outlet attachment points must also stay within 2 mm of their fixed locations. On the bench, mark the insertion depths and check that both clamps hold throughout.

At trial end, weigh water collected during the last uninterrupted 5.00 s with hands clear and both ends connected. Video or a scale recording allows the window to be read retrospectively. A shorter window scores zero even if the stream has already recovered. Divide the collection by this setup’s median open reference. Full credit is a fraction of at least 0.80, half credit is 0.50 to below 0.80, and lower recovery scores zero. The simulation integrates discharge at 500 Hz and uses the same five-second rolling rule. Mass and volume use the same ratio for water at a fixed temperature.

In measurements.csv, enter three reference rows, one kinked row and one trial row. Record net grams, seconds and literal yes/no flags. For a failed clearance check use the final five seconds of the trial and mark hands_clear=no. The same Python score function is called by the simulator and the CSV reader.

## Simulation and validation

Every hose section can collide. Two compliant inlet links and three crease links join eight downstream links to the fixed outlet. Two points on the final link fix the exit position and direction. The robot moves the hose by contact. A rotational spring at the crease has torque −0.67 q(q−0.73)(q−1.25) N·m for angle q in radians. The inlet yaw/pitch stiffnesses are 0.15/0.12 N·m/rad and the downstream values are 0.018/0.025 N·m/rad. Those springs and dampers transmit resistance back to the gripper.

Local curvature drives a compliant wall with 62 N/m stiffness and 0.25 N·s/m damping. If r is half the open height divided by the nominal 6 mm inner radius, the normalized flow is 2r³/(1+r²), limited to the open flow of 100 mL/s. Green tint makes the tube visible. The model approximates a crease at a known location. Torsion, material hysteresis beyond that crease, and fluid transients are outside the model. The force and flow parameters have not been fitted to physical measurements.

| Native simulation test | Collection | Score |
|---|---:|---:|
| Scripted repair, nominal layout | 500.0 mL | 1.0 |
| Independent replay of the same joint commands | 500.0 mL | 1.0 |
| Scripted repair, alternate starting angle | 499.9 mL | 1.0 |
| Dream-1, 420 actions in 21 seconds | 0.0 mL eligible | 0.0 |
| Untouched kink | 0.2 mL | 0.0 |
| Same repair, compliant 4 mm half-gap limit | 303.0 mL | 0.5 |
| Same repair, compliant 1 mm half-gap limit | 66.6 mL | 0.0 |
| Same repair stopped after 3.98 s clear | 397.8 mL | 0.0 |

The nominal open reference was 499.93 mL. The nominal repair finished in 12.4 s. The largest recorded outlet attachment error during the repair was 0.277 mm. The sleeve checks constrain wall travel after open-reference calibration.

## Reproduce

Use Python 3.12, FFmpeg with libx264, EGL and DejaVu Sans. Run in this folder.

```sh
python -m pip install -r requirements.txt
export MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1
python run.py hose --out runs/scripted --video
python run.py hose --out runs/replay --replay runs/scripted/commands.json
python run.py hose --variant 1 --out runs/alternate
python run.py hose --control --out runs/idle
python blocked_lumen.py runs/scripted/commands.json runs/sleeve4.json --half-gap-mm 4
python blocked_lumen.py runs/scripted/commands.json runs/sleeve1.json --half-gap-mm 1
python jug_trial.py measurements.csv  # after entering physical readings
```

In a configured Dream runtime, install the pinned policy adapter. To use a separately hosted private Dream bridge, pass its address with --server-url. The evaluator saves actions, the score trace, a transcript and the model video.

```sh
python -m pip install --no-deps 'inspect-robots-dream @ git+https://github.com/kairosiann/inspect-robots-dream.git@b5e936a201cec66c54c297e522c920d98445e339'
python evaluate.py hose --out runs/model --video
```
