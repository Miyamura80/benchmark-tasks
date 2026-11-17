# Clear a mixed toilet trapway clog with a plunger

Test drainage, seat and pump the plunger, then verify that the trapway drains after the plunger is lifted. The obstruction consists of wet toilet-paper wads interleaved with soft fecal simulant.

## Starting layout

Use the central drain as the origin in X and Y and the floor as Z = 0. The ceramic-look bowl has a raised hinged seat ring and lid, and a cistern. The model drain sits over a 34 mm spring-loaded diaphragm. The plunger has a rigid handle and a 38 mm rubber-cup contact body. It starts upright at (−245, −144, 123) mm, measured at the grip. The Cartesian gripper starts open at (−245, −144, 265) mm. A fixed drain gauge reports water remaining after a 1.25-second test.

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

Use Python 3.12, FFmpeg, EGL and the DejaVu Sans fonts. Install the dependencies with `python -m pip install -r requirements.txt`.

```sh
MUJOCO_GL=egl python bench.py trap --video --out runs/scripted
python bench.py trap --strokes 7 --out runs/seven-strokes
python bench.py trap --control --out runs/off-centre
MUJOCO_GL=egl python inspect_trial.py trap --smoke --out runs/smoke
MUJOCO_GL=egl python inspect_trial.py trap --model "$MODEL_ID" --out runs/model
inspect-robots video runs/model/toilet-trap_*.json --out runs/model-video
```

Use the model provider's environment credential. For an OpenAI-compatible endpoint, add `--base-url "$MODEL_BASE_URL" --api-key-env NAME_OF_KEY_VARIABLE`. The Inspect Robots seed is 0 and the scene reset is deterministic. Local runs produce `result.json`, a 25 Hz trace, and optional H.264 video. The scripted trial lasts about 11.5 seconds of simulated time. Each model trial is bounded by 60 seconds of physical simulation and 100 model calls. Wall time depends on inference and rendering.

Full credit requires at least 70% retained water before repair, cup contact and compression, the rear edge of every piece crossing X = 193 mm, and at most 15% retained water in the final test with the tool clear. The scripted run starts with 95% retained water, ends at 1.25%, and clears all six pieces in eight strokes. See [scripted measurements](evidence/scripted.json). Seven strokes leave a paper wad and score zero. An off-centre plunger clears zero pieces. The no-contact Inspect Robots smoke should score zero.

## Physical reset and measurement

See the [wet-bench protocol](fixture.md) for exact reset and measurements. The model uses rigid bodies under folded paper surfaces, a contact-driven pressure pulse and a simplified drainage model. The physical procedure covers paper and fecal-simulant deformation and real siphon behaviour.
