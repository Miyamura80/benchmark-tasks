# Restore flow through a blocked toilet rim jet

Run a rinse, identify the restricted outlet, remove its obstruction with a probe and verify the flow on a second rinse. The task measures restored flow at an individual outlet inside a confined rim.

## Bench fixture

Use a clean vitreous-china bowl or a rigid replica with unobstructed drainage into a catch tray. Fit an independent, removable rim rail around the bowl. The earlier toilet scene's rim profile is in `fixture/bowl_layout.scad`. In its coordinates, start with outlet centres at (−100, 75), (−100, −75), (100, 75), and (100, −75) mm for front left, front right, rear left, and rear right, respectively. Start 20 mm below the reference rim plane and point each outlet toward the bowl centre. Keep all four in view from inside the bowl and under the lip from above.

| Part | Starting specification |
| --- | --- |
| Rim outlet | Four identical straight-through bulkhead fittings with a nominal 6 mm clear bore. Measure the mounting diameter and adjust the 12.5 mm clearance hole in `fixture/outlet.scad` to suit the selected fitting. Print one saddle per outlet in PETG. Mount each outlet at 25° inward from vertical. |
| Water supply | An elevated constant-head reservoir, a common upstream valve and four equal-length 6 mm ID hoses. Start with the head at 300 mm above the outlets and run each rinse for 10 seconds. Refill and stabilise the head between trials. |
| Obstruction | A measured short plug of reusable non-hardening putty, seated from the outlet face with a depth-stop jig. Record mass, insertion depth and initial flow. Tune to leave 10–40% of the reference flow. Water and putty provide a repeatable test load for the fixture. |
| Tool | A blunt nylon hook probe with a grippable handle, a working tip at most 3 mm wide and a depth stop. The hook must withdraw the obstruction toward the bowl. Retain hose access for manual resets. |
| Measurement | Four labelled collection cups or cups on scales. Fit them below the outlet streams during measurement, with room to withdraw them for tool access. Record water in mL or use grams as approximately equivalent for this test. |

Keep tube and cup positions fixed for the reference and both trial rinses. Make the weak stream visible from the robot's work camera, using cup volume markings if needed. Record a front view and an under-rim view. Keep the obstruction position hidden from the policy. Use a fixed documented starting layout for each of the four fault cases.

## Calibration and trial

1. With all ports open, collect three rinses and use the median volume per outlet as its reference. Require at least 50 mL at each outlet. Adjust the rinse duration if needed and document it for all trials.
2. Run a clear-fixture repeatability check. Confirm that each outlet remains within the proposed 80–120% reference range. Adjust the setup if it cannot produce repeatable readings.
3. Fit one obstruction and run the first scored rinse. The target outlet must collect at most 40% of its reference and the other three must stay within 80–120%. Photograph the cups and record the robot’s outlet choice before tool contact.
4. Remove the obstruction with the probe. Recover it in a removable mesh catcher and confirm that the fitting remains intact. Run the second rinse at the same head and duration.
5. Score 0.25 for the correct diagnosis with intact hardware. Score 1.00 if the obstruction is recovered and all four outlets finish within 80–120% of their own reference. An incorrect diagnosis or damaged fixture scores 0.

Use `fixture/example.csv` to check the scorer. Its volumes are illustrative.

```bash
python fixture/score_flow.py fixture/example.csv --diagnosed front_left --fixture-intact --obstruction-recovered
openscad -o outlet-saddle.stl fixture/outlet.scad
openscad -o bowl.stl fixture/bowl_layout.scad
```

## Simulation target

Reuse the earlier bowl profile as a native static collision mesh and use rigid colliders for the probe and removable obstruction. Use a gripper with normal contact-based grasp and a tool path that is physically constrained by the rim and outlet. Expose measured outlet flow to the robot through visible collection cups. Calibrate any reduced-order flow model against the four separate bench measurements and derive flow from the remaining measured aperture. Trigger a rinse with a physical control. The policy should choose an outlet from the observed rinse, and the scorer should use the last complete rinse plus the recovered obstruction and fixture state.

See the [CPU contact prototype](../../prototypes/toilet-maintenance/README.md) for the scripted demonstration and four fault cases. Its outlets are recessed into the underside of the rim, with their axes inclined inward and down. It uses a captive test obstruction. The physical outlet positions and flow thresholds need wet verification before a YAM submission.
