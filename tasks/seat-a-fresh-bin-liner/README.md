# Seat a fresh bin liner around the rim

**Instruction.** Fold the loose liner over the entire wastebasket rim. Let
go with the opening clear.

**Reset.** An open 3.6 g simulated liner rests inside a 270 mm-high round
wastebasket with a 248 mm outer lip diameter. The hem is loosely draped
around the rim. It ranges from 263.4 to 294.1 mm above the bench. None of
the twelve sectors starts seated. A repeatable simulated reset turns up
four evenly spaced points on the free edge and releases all four 0.20 s
before the robot starts. Gravity and the basket support the bag throughout
the episode. A physical setup can be staged by hand using the same
zero-sector check and an overhead reference image.

Put the bin centre at the origin on the bench. Place the YAM base centres
20 mm toward the front camera, 390 mm left and right of the bin centre,
and yaw the bases 90 degrees inward. The fingers start open around the
two high folds. The 56 mm diameter, 20 mm-thick puck is centered above
the bin and its underside starts 95 mm above the lip. Use a smooth
plastic or rubber-faced puck of the specified mass in the physical trial.

**Simulation.** Both YAMs initially pinch opposing folds and draw them
across the rim. The right YAM keeps its hold while the left lets go,
regrips the raised far fold and seats it. Both arms then release. The simulated
film is held by a bounded spring that engages only after both opposing
pads record native contact and enclose a nearby film vertex. The demo
uses privileged mesh positions for scripted targets.

## Reproduce

Run from the archive root with Python 3.12. MuJoCo, NumPy, SciPy and Pillow
versions are pinned in `requirements-cpu.txt`. Replay generation also needs
an EGL implementation and FFmpeg with libx264.

The I2RT YAM model and meshes are from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/370c43965323c0944b433b202aac5058b5f907a9/i2rt_yam)
under the MIT license in `vendor/i2rt_yam/LICENSE`. The basket and liner
are procedural and the scene contains no other third-party meshes.

```sh
python -m pip install -r requirements-cpu.txt
python fetch_yam.py
python seat_liner.py --yam-xml vendor/i2rt_yam/yam.xml --output cuff.json
python seat_liner.py --yam-xml vendor/i2rt_yam/yam.xml --mode open --output open.json
python seat_liner.py --yam-xml vendor/i2rt_yam/yam.xml --mode pinch --output friction.json
python seat_liner_reference.py --output reference.json
python audit_trace.py cuff.json --yam-xml vendor/i2rt_yam/yam.xml
MUJOCO_GL=egl python render_native_trace.py cuff.json vendor/i2rt_yam/yam.xml demo.mp4 --mode assisted
```

The trace records a continuous 9.55 s simulation with 20 Hz commands and
1.25 ms physics steps. The final release is at 7.35 s. The separate
evaluator drops a 100 g puck at 8.15 s.

The hosted model test uses the same reset, robot actuators, compliant
pinch and score. It supplies RGB from three fixed views and measured
joint positions to Inspect Robots' Dream-1 adapter. Dream-1's joint targets
are applied at 20 Hz. The model has up to 360 controls. At the end of that
window, its last command stays in place while the evaluator observes the
cuff, drops the puck after 0.8 seconds and checks 1.4 seconds of retention.
The evaluator never opens the grippers for the model.

```sh
python -m pip install 'inspect-robots==1.0.0' 'dreamlabs-client==1.0.14'
python -m pip install --no-deps 'dreamlabs==1.0.23' 'inspect-robots-dream @ git+https://github.com/kairosiann/inspect-robots-dream.git@b5e936a201cec66c54c297e522c920d98445e339'
MUJOCO_GL=egl python evaluate.py --yam-xml vendor/i2rt_yam/yam.xml --output dream-run
```

Configure `DREAM_API_KEY` outside the task directory.

## Success checks

A sector needs film running from the interior across the lip to an exterior
hem hanging at least 9 mm below the rim. Each of the twelve marked sectors earns one point if that same sector stays seated at **every** 1.25 ms physics step from final release through the end of the puck test. The score is sector points divided by 13, plus one extra point divided by 13 only for the original full pass. For full credit all twelve must hold, at least 85% of the aperture must remain clear, and the puck must pass the checks below. A simulator warning, active grip assistance at or after release, or robot–film contact at or after the drop gives zero. For physical scoring use numbered sectors on the basket and check their identity in the continuous release/drop recording. The robot clears the liner before the independent drop.
The puck center must remain above the 20 mm base and settle on the lining
inside the basket. Positive puck–film contact must persist throughout the
last 0.35 s with speed below 20 mm/s. The rigid bottom supports most of the
puck's weight.

The one-pair control uses the same reset, arms, and independently timed load. It skips the second grasp. In this run 11 sectors were seated at the end, but only five specific sectors held continuously. It earned **5/13 (0.385)**. If all twelve stay seated but the opening or puck fails, a hands-clear valid run earns 12/13. To reproduce the partial trial add `--one-pair` to the scripted command, then pass its trace to the same auditor and renderer. The auditor reconstructs retention by intersecting the recorded twelve-bit sector masks over every physics step. Older traces without per-step sector identities can still be audited for their original full pass.

The successful native episode starts at **0/12**, then retains **12/12**
and **100% aperture** after final release. The puck center stays above
24.48 mm and settles at 29.994 mm. The robot's last film contact is at
7.68625 s, before the drop. The audit in `validation-summary.json`
summarizes 192 recorded frames and 7,641 consecutive physics samples. At the
recorded 20 Hz poses, no rigid collision involving the robot overlapped
by more than 0.5 mm. The maximum sampled overlap was 0.35 mm. Two local
assisted repeats produced byte-identical traces and the independently
recorded submission demo also passed.

| Trial | Sectors at reset | Minimum sectors after release | Minimum opening after release |
| --- | ---: | ---: | ---: |
| YAMs with contact assistance | 0/12 | 12/12 | 100% |
| Open-jaw control | 0/12 | 0/12 | 25% |
| Native friction only | 0/12 | 0/12 | 3.75% |
| Hands-free control at matched duration | 0/12 | 0/12 | 35% |

The open-jaw and native-friction controls replay the scripted stages with
the same reset. They can contact the rim and disturb the film as their
grasp behaviour diverges. The hands-free control independently checks
that the rim does not seat on its own.

The hosted Dream-1 beta completed **360 controls** and scored **0.0**.
After 32 warm-up controls, the adapter received seven action chunks across
seven service sessions. The episode continued through the session renewals.
It seated no sectors and the minimum opening after release was 11.25%.
The separate demo scored **1.0**.

## Eligibility and remaining checks

The score distinguishes cuffing from putting a loose plastic bag inside a
bin. Stanford's garden-path cleanup accepts the latter. The pinned
BEHAVIOR-1K review found no equivalent score requiring a seated rim cuff
that remains open after a load. [Stanford task definition](https://behavior.stanford.edu/knowledgebase/tasks/cleaning_garden_path-0.html)

Virginia's custodial training documents folding a fresh liner over a
wastebasket rim. Washington's workload survey reports 66% daily for the
combined emptying-and-relining operation. BLS counts 2.341 million U.S.
janitors and building cleaners, so 21.4% of that occupation would reach
500,000. National prevalence of this exact step is still an inference.
[Custodial training](https://vspma.org/wp-content/uploads/2023/03/CustodianTrainingManualFinal2023-2024.pdf)
· [Washington study](https://lni.wa.gov/safety-health/safety-research/files/2022/102-124-2022%20Janitorial%20Workload%20Study.pdf)
· [BLS CPS Table 11b](https://www.bls.gov/cps/cpsaat11b.htm) · [O*NET 37-2011](https://www.onetonline.org/link/details/37-2011.00)

The film and compliant pinch remain approximations. Peak edge-length
deviation reaches about 28% in the demo. The reset starts at about 13%.
Film properties and grip response still need physical characterization on
the proposed Staples ST63898 basket and LK Packaging ALL18BG liner. The
retail basket's taper and ventilation differ from the model. The accepted
and pending RoboCurve board searches turned up no matching liner-seating
task.

For a bench check, mark twelve sectors and the 9 mm depth on the basket.
Lay the open bag into the basket by hand and loosely drape the mouth over
the edge. Lift any folds that have already seated. Record the reset from
above and around the rim. Film a hands-free control and a manually fitted
cuff with a centered 100 g drop from 95 mm above the lip. Use the overhead
view to score aperture and the side views to verify the exterior hem.

The original board videos remain valid at their reported scores. The scorer-only revision also includes a fresh scripted full run, one-pair control and offline regrade of the archived model actions in [the revised results](evidence/revised-grades.json). The local action replay reproduced the model's zero score, with a minimum opening of 5% versus 11.25% in the original model trace. No new model inference was used.
