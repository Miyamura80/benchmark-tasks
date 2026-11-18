# Clear a mixed toilet trapway clog with a plunger

Test drainage, seat and pump the plunger, then verify that the trapway drains after the plunger is lifted. The obstruction consists of wet toilet-paper wads interleaved with soft fecal simulant.

## Starting layout

Use the central drain as the origin in X and Y and the floor as Z = 0. The ceramic-look bowl has a raised hinged seat ring and lid, and a cistern. The model drain sits over a 34 mm spring-loaded diaphragm. The plunger has a rigid handle and a 38 mm rubber-cup contact body. It starts upright at (−245, −144, 123) mm, measured at the grip. A fixed drain gauge reports water remaining after a 1.25-second test.

The six pieces start as a tight staggered plug downstream. Centres and full bounding dimensions are in millimetres.

| Piece | Centre X / Y / Z | Size |
| --- | --- | --- |
| Paper 1 | 52 / −9 / 114 | 24 × 18 × 16 |
| Simulant 1 | 60 / 7 / 100 | 28 × 20 × 16 |
| Paper 2 | 72 / 9 / 115 | 26 × 18 × 16 |
| Simulant 2 | 81 / −8 / 100 | 32 × 18 × 16 |
| Paper 3 | 87 / −9 / 116 | 24 × 18 × 14 |
| Simulant 3 | 101 / 7 / 102 | 28 × 16 × 18 |

## Reproduce and score

Two I2RT YAM arms with stock parallel jaws sit on bases at (−250, ±305, 175) mm. Both start with grippers open. The overhead camera and both wrist cameras feed Dream-1. The blue spring-loaded test switch sits at (−160, 212, 231) mm. Finger contact and at least 10 mm of physical switch travel trigger each measurement. The tool must remain clear of the bowl while the test runs.

Use Python 3.12, FFmpeg, EGL and DejaVu Sans. Install the robot scene, evaluator and Dream client.

```sh
python -m pip install -r requirements.txt
python -m pip install --no-deps 'dreamlabs==1.0.23'

MUJOCO_GL=egl python yam_trial.py trap --video --out runs/scripted
MUJOCO_GL=egl python yam_trial.py trap --control --out runs/control
MUJOCO_GL=egl python yam_eval.py trap --out runs/dream
inspect-robots video runs/dream/toilet-yam-trap_*.json --out runs/dream-video
```

Set the Dream API credential in the authorised environment before running the model. Both runs use the same workcell and scorer. The model uses seed 0 with a deterministic reset and 1,200 control steps at 20 Hz. The scripted demo takes 26.6 seconds of simulated time. Wall time varies with rendering and inference.

Full credit requires at least 70% retained water before repair, cup contact and compression, the rear edge of each piece crossing X = 193 mm, and at most 15% retained water in the final test. The scripted run starts at 95%, ends at 1.25%, and clears all six pieces. An off-centre control clears no pieces and scores zero. See the [scripted result](evidence/scripted.json) and [off-centre control](evidence/control.json).

The [Dream-1 baseline](evidence/dream.json) scored 0.00 after 1,200 steps. It used 25 model requests in 103.3 seconds of wall time. The baseline and scripted video are both supplied.

## Physical reset and measurement

See the [wet-bench protocol](fixture.md) for exact reset and measurements. The model uses rigid bodies under folded paper surfaces, a contact-driven pressure pulse and a simplified drainage model. The physical procedure covers paper and fecal-simulant deformation and real siphon behaviour.
