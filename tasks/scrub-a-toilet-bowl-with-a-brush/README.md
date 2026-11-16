# Clean hidden deposits behind an overhanging toilet rim

Lower a bent brush through the bowl opening, reach behind the lip, scrub both hidden marks on the underside and return the brush upright. The two exposed wall targets are diagnostic and earn no score.

The closest published comparator is [BEHAVIOR-1K cleaning_bathrooms-0](https://behavior.stanford.edu/knowledgebase/tasks/cleaning_bathrooms-0.html), which lists a scrub brush and makes clearance of toilet stains a goal at fixture level. This test isolates tool access behind an obstructing lip. The endpoint requires accessing two occluded, downward-facing targets with native bristle contact, then reversing the access motion. The exposed-only negative control checks whether an ordinary wall scrub suffices. The air-pass control checks whether proximity alone suffices.

## Physical protocol

Use a clean vitreous-china bowl with a downward-facing rim lip. The fixture is unplumbed, stable and begins dry. Mark the two locations on the fixture drawing at azimuths −144.4° and −83.1°, measured from +X at the bowl centre. For each target, use a flexible 35 × 8 mm stencil against the underside and spread 0.10 mL of undiluted brown paint from the [Crayola Washable Paints set, item 541205](https://www.crayola.com/en-gb/products/paint-and-brushes/washable-paints-pack-of-10-assorted-colours-541205) in one pass. Give it 10 minutes to dry. Pilot the paint on the actual glaze first to confirm it stays put at that thickness and can be removed with the specified brush; verify the same product and zero dilution are used across trials. Clean and dry the brush and fixture before reapplying the marks.

Use two fixed indexed under-rim camera poses with even illumination. Record a baseline photo of each mark after drying and an end photo without moving the cameras or changing exposure. In each image, mask the stain relative to the same clean-glaze reference and measure the remaining stained area divided by the baseline area. Require 92% area removal at **each** marked region. Use the recorded motion to verify bristle contact at both regions. Verify the brush is stationary, upright, supported by its holder and fully released. The simulation grades contact-driven removal of samples in each target separately and excludes the exposed wall from credit.

## IsaacLab validation

With the pinned RoboCurve YAM Arena environment active, run from this folder:

```sh
python run_task.py yam_scripted --case nominal --log-dir runs/demo --setup runs/demo/setup.png
python run_task.py yam_scripted --case exposed-only --log-dir runs/exposed
python run_task.py yam_scripted --case air-pass --log-dir runs/air
python run_task.py dream --log-dir runs/dream
```

Use a fresh reset and 1,400 actions at 20 Hz. The log records exposed and hidden cleaning percentages, load-bearing bristle contacts, grasp status and release. The native collider and revised choreography need fresh recordings before submission.
