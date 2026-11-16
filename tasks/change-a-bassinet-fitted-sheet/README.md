# Change a bassinet fitted sheet

Strip a used sheet, put it in the laundry zone, and fit a fresh sheet onto all four mattress corners. This is a real-size bassinet fixture, chosen so the entire mattress fits within the two fixed YAM arms' working area.

## Reproduce the scene

| Part | Material, dimensions and fixed reset |
| --- | --- |
| Mattress | Foam, 660 × 275 × 44 mm on a 14 mm plywood platform, 59 mm above tabletop at its top. Long dimension along Y. The near and far X edges are 206 and 481 mm from the arm bases. Side edges are at Y ±330 mm. No infant or bassinet side rails are present. |
| Used and clean fitted sheets | Two lightweight cotton jersey fitted sheets for this mattress, with sewn elastic hems and 37 mm fabric pull tabs at the four corner seams, so the jaws can pinch outside the foam. Coral begins fully seated under all four corners. The blue sheet is accordion-folded on the narrow shelf ahead of the mattress, with four distinguishable pull tabs at the fixed coordinates in `task.py`. |
| Clean shelf | 45 × 526 × 20 mm horizontal shelf centred at X = 141 mm. Place the folded blue sheet on top. |
| Laundry zone | 207 × 600 mm floor pad with yellow edges, centred at X = 90 mm. Leave it empty at reset. |

Coordinates are from the midpoint of the arm bases, +X away from the robot and +Y to its left. Reproduce fabric folds by placing the four hem corners at the listed coordinates.

## Run

From this directory, in the RoboCurve YAM Arena environment with Inspect Robots 1.0:

```bash
python run_task.py yam_scripted --log-dir runs/demo --setup setup.png
python run_task.py dream --log-dir runs/dream
python run_task.py yam_scripted --case miss-one-pocket --log-dir runs/miss
python run_task.py yam_scripted --case leave-used-sheet --log-dir runs/leave
python make_evidence.py runs/demo demo.mp4 --title 'Scripted YAM demo'
python make_evidence.py runs/dream model.mp4 --title 'Dream-1 baseline'
```

The runner uses seed 42, 20 Hz and a 1,400-step (70 s) limit, with zero object jitter.

## Scoring and fidelity

Full credit requires the released used sheet entirely clear of the mattress (no material points within its footprint at or above 3 mm below its underside) and at least 65% of its material sample points inside the laundry zone, all four clean elastic pockets seated and released, and at least 88% of the clean sheet's interior sample points lying on the mattress-height band. Score 0.17 for stripping the used sheet, another 0.08 for placing it in the laundry zone, and another 0.155 for each seated clean pocket after the used sheet has been stripped. The last 0.13 requires the complete result. A physical trial uses a top view, corner photos and an inspection of the mattress surface.

Cloth is simulated as a mass-point grid with stretch and shear constraints and mattress collision. The fitted hem remains caught until it is stretched clear; installation follows an outside-to-under motion and a release. The loose clean corners are kept in their initial folds until picked, representing tray friction. Jaw pinch and elastic edge retention are geometric approximations; the model omits fibre mechanics, seam damage, and cloth-on-cloth collisions. A denser visible fabric mesh follows the underlying contact grid. The elastic hem wraps down the mattress sides as its corner pockets seat, and the sewn tabs droop when released. Both arms and their cameras run in IsaacLab. The scripted policy commands fixed robot waypoints.

## Recorded results

The recorded videos and setup image show the earlier cloth rendering. A replay of the recorded arm endpoints checked the revised sheet view against the same contact model.

| Trial | Score | Control steps | Runtime including startup |
| --- | ---: | ---: | ---: |
| Scripted YAM | 1.00 | 987 | 368 s |
| Dream-1 | 0.00 | 1,400 | 489 s |

The released used sheet was clear of the mattress with 86.8% of its mesh in the laundry zone. Four clean pockets were fitted, with 90.9% of the clean interior in the mattress surface band. In separately replayed contact-model controls, leaving one clean pocket loose scores 0.715 and leaving the used sheet on the mattress scores 0.0.
