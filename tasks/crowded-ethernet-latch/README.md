# Move a latched Ethernet plug on a crowded switch

Move the orange patch lead from its lower port to the empty port directly above it. Press the latch tab, pull the plug out, push it fully into the upper port until it latches, then let go with both hands. The six other leads must stay plugged in.

## Reproduce the runs

Use the IsaacLab-Arena YAM environment at commit `c7edf3a9d5d8276565208f72422f8f8a0806d6cf` with Inspect Robots 1.0.0, Isaac Sim 6.1.0.0 and an NVIDIA RTX GPU. Run from this folder; each command needs a new output directory. Dream-1 uses its configured API credential.

```bash
python model_runner.py --policy dream --out /tmp/ethernet-dream
python model_runner.py --policy recorded --out /tmp/ethernet-scripted
python audit_score.py /tmp/ethernet-scripted
```

Both runs start with seed 0 and zero object jitter. The limit is 2,400 control steps at 20 Hz (120 simulated seconds). Inspect writes the final task score to the evaluation JSON, and `trajectory.jsonl` records every scored sample. Add `--state-only` to the scripted command for a faster run without cameras; it gives the same score.

## Scoring

`scoring.py` implements the checks in `task.json`. Each one corresponds to something a tester can repeat with a ruler, a luggage scale and their eyes:

| Score | Simulation check | Real-trial check |
|---|---|---|
| 0.25 | orange plug front ≥ 2 mm outside the switch face | lower port visibly empty |
| 0.5 | orange plug in the upper port, no more than 1.5 mm shallower than the median neighbour, within 2 mm of the port centre and 5° of its axis | ruler: rear protrusion within 1.5 mm of the neighbours |
| 0.75 | that state held for 2 s with both grippers open, both gripper centres ≥ 70 mm away and ≥ 5 mm clearance | stopwatch and ruler |
| 1.0 | then a scorer-applied 3 N pull on the orange boot for 1 s moves it < 1 mm and it stays fully home | luggage scale: 3 N (about 300 g) straight back for 1 s |

Any blue/green plug pulled out more than 2 mm, the orange tab bent past its stop (broken), or the orange plug dropped below the tabletop scores 0. The tug is applied by the scorer, never by the policy, and only after the hands-off hold; the policy keeps running during it. After a full score the episode records one more second so the final frames show the result.

## Scripted demonstration

The recorded policy reads only the clock and sends 14 joint/gripper targets per step from `clock_actions.json`. A bimanual feedback controller generated those actions using the simulated plug and clip poses. The policy never writes object poses or applies forces.

## Physical model

Seven articulated leads occupy a two-row, four-column block of ports. Each plug has a spring-loaded latch slider with a retaining hook; contact between the hook and the socket ledge provides retention. Dimensions approximate RJ45 hardware, not vendor CAD. Each lead's jacket is twelve 16 mm jointed capsules (19 cm), not the full lead length. The simulation does not model electrical continuity, plastic fracture or conductor strain.

RESULTS_PLACEHOLDER
