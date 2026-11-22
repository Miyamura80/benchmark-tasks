# Move a latched Ethernet plug on a crowded switch

Move the orange cable from its lower socket to the empty socket directly above it. Depress the tab before pulling, then release both grippers once the plug latches. The other six cables must remain connected.

## Reproduce the runs

Use the IsaacLab-Arena YAM environment at commit `c7edf3a9d5d8276565208f72422f8f8a0806d6cf` with Inspect Robots 1.0.0, Isaac Sim 6.1.0.0, and an NVIDIA GPU. Activate the YAM environment and run from this folder. Each command needs a new output directory. Dream-1 uses its configured API credential.

```bash
python model_runner.py --policy dream --out /tmp/ethernet-dream
python model_runner.py --policy recorded --state-only --out /tmp/ethernet-scripted
python audit_score.py /tmp/ethernet-scripted
```

Both runs start with seed 0 and zero object jitter. The limit is 2,400 control steps at 20 Hz, or 120 simulated seconds. Inspect writes the final task score to the evaluation JSON. The scripted run finished in 1,582 steps (79.1 simulated seconds).

## Scripted demonstration

The recorded policy reads only the clock and sends 14 joint/gripper targets per step. A bimanual feedback controller generated the recorded actions using the simulated plug and clip poses. Neither policy writes object poses or supplies external forces.

To reproduce the three-camera demo, use the review camera positions in `demo_camera.py` in place of `review.py` in a separate copy of this folder, then run:

```bash
python recorded_run.py --mode release-probe --recorded . --out /tmp/ethernet-camera
```

To run the original feedback controller:

```bash
python run.py --mode release-probe --pinch-spec winning-plan.json --no-video --out /tmp/ethernet-feedback
```

## Physical model

Seven articulated cables occupy a two-row, eight-port panel. Each plug has a spring-loaded slider with a retaining hook. Cables use twelve jointed capsules each. Contact between the hook and socket ledge provides retention. Dimensions approximate an RJ45 connector. The simulation does not model electrical continuity, plastic fracture, or conductor strain.

The scripted run scored 1.0 with all six neighboring latches retained. A separate joint diagnostic measured 2.33° peak clip rotation and 0.122 mm transverse anchor drift under load. Its engineering limits are 0.5° and 0.100 mm. Exact scoring conditions are in `scoring.py` and `native_episode.py`.

## Recorded results

Dream-1 scored 0.00 in 2,400 steps (120.00 simulated seconds, 21.0 minutes elapsed). The scripted policy scored 1.00 in 79.1 simulated seconds (6.3 minutes elapsed). The board media ID is `M-EL6DX9`.
