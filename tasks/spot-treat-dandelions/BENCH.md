# Verify spot treatment on a real lawn patch

**Question.** Did spray reach all three weed rosettes while sparing the surrounding turf? This is an immediate upper-surface deposition check. Plant health and weed death require different observations.

## Build and reset

Use a 300 × 200 mm piece of short lawn turf in a watertight tray. Source three young rooted common dandelions from untreated turf, or transplant nursery-grown common dandelion plugs. Select rosettes 40–65 mm across. Place their crowns at (61,70), (150,139), (232,66) mm measured from the top-left of the tray, within 6 mm. Seat each crown flush with the turf, tamp loose soil and let the foliage settle. Keep the leaves visible and the three rosettes apart. For a nominal replica, use two open, low-growing yellow flower heads and one closed green bud. The third weed must be identifiable by its rosette. Record the three canopy diameters, heights and bloom states. Other trials may use natural variation in leaf outlines and the number of florets. Photograph and label the actual leaves afresh for every reset.

Use a small hand-pump mist sprayer filled with water and a fluorescent spray-pattern tracer selected for green fluorescence on the actual plants. Follow the tracer and lamp instructions. Mount the sprayer on the right wrist in a rigid clamp with its nozzle pointing along the wrist approach axis and a jaw-operated pump linkage. The user can also do the calibration by hand. With the bottle off the test tray, check that each complete squeeze produces one spray and that holding it closed does not repeat. The simulator uses a wrist mount with about 36 mm between the tool centre and nozzle tip. For a nozzle comparison, set the tip 65 mm above a flat test sheet and make one full squeeze straight down. Repeat on ten clean areas. Photograph beside a ruler and record the nozzle setting, sprayer make, stroke travel and fill level. As a model reference, at that height a roughly 52 mm diameter circle around the aim point encloses 90% of newly marked 1 mm cells in a single spray onto a flat plane. This estimate uses the simulator's 1,800 rays and transverse spread of 0.17. It is a reference for adjusting the nozzle before the trial, not an empirical result from a garden sprayer. Keep the settings fixed when comparing the careful and blanket controls.

Print both A4 pages of frame-template.pdf at actual size, check each 50 mm scale, trim and tape at the dashed join. Use its crown centers as setup guides under the empty tray. For photography, place the registration crosses and the two reference leaves on a rigid level border near the top of the cut grass. A template left far below the canopy introduces parallax. Keep the two reference leaves stationary and cover them without touching them. Mount a phone or camera facing straight down over the tray. Fix the rectangular registration frame with four corner crosses 340 × 230 mm apart. Center the turf in it, leaving 20 mm at each side and 15 mm above and below. Keep the four crosses visible in daylight and UV. Mark two 8 × 8 mm reference patches in the lower margin with centers (49,221) and (291,221) mm relative to the outer frame. Use dry leaf material from the same stock on both patches. Shield both patches during the run. Under a fixed UV lamp, lock focus, exposure and white balance, and fill the frame to at least two pixels per mm. Block ambient light for both UV photos.

## Score a trial

1. Take the daylight and clean UV photos. Run the first two commands below before viewing the outcome. Outline visible weed leaves, stems, open heads and the closed bud. Draw several polygons per weed when leaves have gaps. Keep surrounding grass out of the mask.
2. Run the task once. Leave the tray and camera in place. Withdraw the sprayer from view. Uncover the references, leaving the first untouched. Wet the entire second reference area with the same tracer using a swab. Take another UV photo at the same settings. Also take a daylight photo to confirm that plants did not move or become obscured.
3. Check the registered daylight outlines against the new photo. The third command makes a side-by-side daylight overlay using the same pre-run magenta outlines on both photos. Inspect every weed at full resolution. Where a contour visibly leaves its leaf or crosses grass after treatment, record an unreadable trial. Corner registration corrects movement in the frame plane. A camera bump can still shift elevated leaves relative to the marks. An obscured weed, failed controls or glare also makes the trial unreadable. Retain that result. Never count it as a robot miss.
4. Run the last command. A new deposit is an increase of at least 35 in green minus the mean of red and blue (8-bit RGB). The program rejects a dry reference that changes in either direction, a weak positive reference, or a clipped reference. Positive-reference wetting checks that the chosen liquid fluoresces. The blinded careful and blanket controls test whether real mist-sized deposits are visible. On a readable trial it reports a separate coverage fraction for each weed and the newly marked grass area divided by total visible weed area.
5. Proposed thresholds: all three weeds at least 30% covered, and marked grass area at most 45% of combined weed area. Example: if weed masks total 30 square cm, up to 13.5 square cm of marked grass is allowed. No liquid-volume measurement is required. The score is binary. Archive the original photos, masks, camera settings, references and JSON.

```sh
python bench.py trial01 --daylight day.jpg
python photo_prep.py trial01/daylight-crop.png trial01/labels.png
python bench.py trial01 --check-daylight day-after.jpg --labels trial01/labels.png
# Inspect trial01/alignment-overlay.png. Supply the flag only if every contour aligns.
python bench.py trial01 --before uv-before.jpg --after uv-after.jpg --labels trial01/labels.png --alignment-checked
```

The first run prompts for four corners in each photograph. Keep its corner JSON to repeat the analysis. Omit the alignment flag to record an unreadable trial. Use a fresh directory and dry turf plus fresh plugs for each trial. Do not reuse wet material.

## Physical calibration still to run

Prepare fresh matched trays. In randomized order, have a person (a) target all three weeds, (b) deliberately miss one, (c) spray across the whole patch, and (d) leave the tray dry. Hide the condition from two graders. Each independently outlines the plants before seeing the UV result, then runs the scoring tool. Save both scores, unreadables, true conditions, and setup/reset time. Repeat with a second set of plants. Check whether careful passes pass and misses and blanket sprays fail, along with agreement and measurement margins. If the thresholds or nozzle change, declare the new settings and validate on fresh trays. No physical trial is included with this package.

Spray-tracer background: https://pnash.deohs.washington.edu/fluorescent_tracer
