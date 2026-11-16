# Scrub the inside of a toilet bowl with a toilet brush

Remove four brown marks inside the bowl, including two under the rim, then return the brush to its holder. The task measures targeted cleaning contact on a curved, partly occluded surface. Moving the right tool nearby earns no cleaning credit.

## Reproduce the scene

| Part | Material, dimensions and fixed reset |
| --- | --- |
| Toilet bowl | Bench-scale vitreous-china toilet with a tapered pedestal. Outer rim 386 × 304 mm, opening 294 × 222 mm, and rim 134 mm above the table. The bowl is centred at (263, −124) mm. |
| Toilet trim | The scene includes an open seat ring with hinges and a rear cistern with an overhanging lid and flush button. The cistern is behind the brush working area. |
| Brush and holder | Nylon bristles on a rounded 46 mm head, round shaft, and 22 mm rubber grip. The robot grasps the brush handle 42 mm above the bristle centre. Start upright in the holder at (155, −306) mm, grip centre 112 mm above the table. |
| Stains | Four clusters of five washable brown pigment patches. Two opposite patches near the back-facing lip, 10 mm below the rim, and two on the sloping bowl wall, 18 mm below the rim. Angles and radii are listed in `task.py`. Reset each patch to fully soiled. |
| Waterline | Fixed visual surface at 75 mm below the rim. Use clean water and washable marks for a bench trial. No cleaners are needed. |

Coordinates are from the midpoint of the arm bases, +X away from the robot and +Y to its left. The fixture is secured by its pedestal with the seat opening clear.

## Run

From this directory, in the RoboCurve YAM Arena environment with Inspect Robots 1.0:

```bash
python run_task.py yam_scripted --log-dir runs/demo --setup setup.png
python run_task.py dream --log-dir runs/dream
python run_task.py yam_scripted --case air-pass --log-dir runs/air
python run_task.py yam_scripted --case one-side --log-dir runs/one-side
python make_evidence.py runs/demo demo.mp4 --title 'Scripted YAM demo'
python make_evidence.py runs/dream model.mp4 --title 'Dream-1 baseline'
```

The runner uses seed 42, 20 Hz and a 1,400-step (70 s) limit, with zero object jitter. It logs each region's remaining deposit, the brush pose, holder state and score.

## Scoring and fidelity

A marked region is clean when at least 92% of its pigment is gone. Score 1.0 when all four regions pass and the released brush is upright in the holder. Otherwise score 0.20 times the sum of the cleaned fraction in each of the four regions. On a bench, compare before/after photos from fixed viewpoints with a mirror for the under-rim patches. Inspect the released brush in its holder.

YAM runs in IsaacLab with stock station cameras. The bristles use a swept contact footprint and tangential travel to remove dirt from the curved ceramic. A brush tip outside the inner bowl, above the mark or stationary removes no pigment. The holder and brush grasp use geometric constraints. Bristle flex, scrubbing force, aqueous flow and chemical cleaning are approximated or omitted. The tool path is driven by actual robot joint positions. The scripted controller uses fixed robot targets.

## Recorded results

The attached IsaacLab recordings and setup image are from the original 101 mm rim revision. An offline test of the 134 mm revision with the YAM URDF kinematics and brush contact model scored 1.0 nominal, 0.0 for an air pass, and 0.40 for cleaning only two marks.

| Trial | Score | Control steps | Runtime including startup |
| --- | ---: | ---: | ---: |
| Scripted YAM | 1.00 | 622 | 352 s |
| Dream-1 | 0.00 | 1,400 | 562 s |

All four final regions read 100%, 100%, 100%, 100% clean, and the brush was released upright in its holder. In separately replayed contact-model controls, an air pass scores 0.0 and scrubbing only two regions scores 0.40.
