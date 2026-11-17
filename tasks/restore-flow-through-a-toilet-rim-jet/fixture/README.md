# Restore flow through a blocked toilet rim jet

Run a rinse, identify the restricted outlet, scrape off its limescale and verify the flow on a second rinse. The task measures restored flow at an individual outlet inside a confined rim.

## Bench fixture

Use a clean vitreous-china bowl or a rigid replica with unobstructed drainage into a catch tray. Fit an independent, removable rim rail around the bowl. The bowl profile is in `bowl_layout.scad`. In its coordinates, start with outlet centres at (−100, 75), (−100, −75), (100, 75), and (100, −75) mm for front left, front right, rear left, and rear right, respectively. Start 20 mm below the reference rim plane and point each outlet toward the bowl centre. Keep all four in view from inside the bowl and under the lip from above.

| Part | Starting specification |
| --- | --- |
| Rim outlet | Four identical straight-through bulkhead fittings with a nominal 6–8 mm clear bore. Measure the mounting diameter and adjust the 12.5 mm clearance hole in `fixture/outlet.scad` to suit the selected fitting. Print one saddle per outlet in PETG. Mount each outlet at 25° inward from vertical. |
| Water supply | An elevated constant-head reservoir, a common upstream valve and four equal-length 6 mm ID hoses. Start with the head at 300 mm above the outlets and run each rinse for 10 seconds. Refill and stabilise the head between trials. |
| Obstruction | Four uneven deposits of a chalk-based limescale surrogate cured around the inside edge of one removable outlet. Record deposit mass, cure time and initial flow. Tune the coverage to leave 10–40% of reference flow. |
| Tool | A slim blunt scraper with a grippable handle and a depth stop. Draw the edge across the crust in shallow passes and recover the detached pieces in a mesh catcher. |
| Measurement | Four labelled collection cups or cups on scales. Fit them below the outlet streams during measurement, with room to withdraw them for tool access. Record water in mL or use grams as approximately equivalent for this test. |

Keep tube and cup positions fixed for the reference and both trial rinses. Make the weak stream visible from the robot's work camera, using cup volume markings if needed. Record a front view and an under-rim view. Keep the obstruction position hidden from the policy. Use a fixed documented starting layout for each of the four fault cases.

## Calibration and trial

1. With all ports open, collect three rinses and use the median volume per outlet as its reference. Require at least 50 mL at each outlet. Adjust the rinse duration if needed and document it for all trials.
2. Run a clear-fixture repeatability check. Confirm that each outlet remains within the proposed 80–120% reference range. Adjust the setup if it cannot produce repeatable readings.
3. Fit one obstruction and run the first scored rinse. The target outlet must collect at most 40% of its reference and the other three must stay within 80–120%. Photograph the cups and record the outlet the robot selects and verify scraper contact there.
4. Scrape the deposits free. Recover the fragments in the mesh catcher and confirm that the fitting remains intact. Run the second rinse at the same head and duration.
5. Score 0.25 for contacting the correct outlet with intact hardware and completing the second rinse. Score 1.00 if the obstruction is recovered and all four outlets finish within 80–120% of their own reference. An incorrect diagnosis or damaged fixture scores 0.

From the task root, use `fixture/example.csv` to check the scorer. Set `--diagnosed` to the outlet actually contacted. Its volumes are illustrative.

```bash
python fixture/score_flow.py fixture/example.csv --diagnosed front_left --fixture-intact --obstruction-recovered
openscad -o outlet-saddle.stl fixture/outlet.scad
openscad -o bowl.stl fixture/bowl_layout.scad
```
