# Remove pet hair from a cloth boot liner

The right YAM rakes two strips of embedded hair out of the carpet, including one along the raised trim. The left YAM switches on the cordless car vacuum at its dock, collects the loose clumps, redocks the vacuum and switches it off.

## Setup

| Component | Setup |
|---|---|
| Cargo liner | 232 × 122 mm carpet coupon on a rigid base. A molded sill borders the short edge. The trim rises 33 mm above the pile along the long edge. |
| Hair | Two strips of five groups at 26 mm pitch, with 45 mm between rows. The strip by the trim sits 29 mm from its inside edge. Use the same placement for each reset. |
| Tools | A short, 36 mm-wide rubber-bristle head sits at the sill. A cordless handheld vacuum with a narrow nozzle and a visible dust bin starts empty and off in the dock. Its toggle button has 6 mm nominal travel. |

For a physical reproduction, press equal amounts of pet fur into the pile, brush both rows, and vacuum with a low nozzle pass. Check the liner and empty the bin into a tray to compare the recovered fur. The fixture dimensions and the physical procedure describe a proposed setup. The evidence runs use the simulator.

## Simulation and scoring

- Hair is represented by ten moving 3.8 g proxies, each with 28 visual strands. The carpet pile and the strands provide visible texture. A forward brush sweep near the pile and overlapping a group marks it loose and pushes the proxy along the stroke. This stands in for individual fibre entanglement and flex.
- The robot pinch grips each tool at measured fingertip contacts. Tool bodies translate on three axes, keeping orientation fixed. The switch toggles after 3.6 mm of depression and rearms below 1 mm.
- The suction force falls with distance and off-axis angle, with a 40 mm maximum radius. It applies to loosened groups while the vacuum is powered. This represents intake airflow with a local force field.
- A group can enter the bin after powered suction exceeds 45 mN and the group reaches within 7 mm of the mouth. A compliant retention force holds captured groups. Each captured group earns 0.09 while within 16 mm of the bin center. The final 0.10 requires all ten groups in the bin and the vacuum released in its dock with power off.

## Reproduce

Use the dual-YAM runtime, MuJoCo 3.3.7, NumPy, and an EGL renderer with FFmpeg, Pillow and DejaVu fonts for video.

```sh
MUJOCO_GL=egl python run.py hair --out output/scripted --video
python run.py hair --out output/power-off --power-off-control
python run.py hair --out output/no-brush --no-brush-control
MUJOCO_GL=egl python run.py hair --out output/replay --video \
  --replay output/scripted/commands.json --phase-trace output/scripted/trace.jsonl
```

| Run | Score | Result |
|---|---:|---|
| Native joint replay | 1.00 | All 10 bundles entered the intake and settled in the bin. The vacuum ended docked and off. |
| Power off | 0.00 | The brush loosened all 10 bundles. The unpowered nozzle passed within 9.1 mm and collected zero. |
| Powered with no brushing | 0.00 | The powered nozzle swept across the embedded hair within 10.1 mm. All bundles stayed on the liner. |

The matching exterior demo and setup still show both arms and the workbench. The scripted trial uses native joint commands.

Dream-1 completed at **0.00** after 1,200 actions. The model received RGB and measured joint positions through Inspect Robots. The policy evidence is in `evidence/dream`.

Run the same policy evaluation from this task directory with Python 3.12, Inspect Robots 1.0.0, dreamlabs-client 1.0.14, DreamLabs 1.0.23, and the Inspect Robots Dream adapter at commit b5e936a201cec66c54c297e522c920d98445e339. With Dream-1 credentials configured, use `MUJOCO_GL=egl python evaluate.py hair --out dream-results --video`.

The baseline made 25 successful model requests in 25 sessions after session expiry recoveries.

For the scripted run, install with `python -m pip install -r requirements.txt`.
