# Roll a loose insect-screen spline back into its frame

Repairing a local spline failure can restore insect screening without replacing the full frame.

## Setup and reset

A flat aluminum-profile screen frame, a locally slack mesh sheet, a 96 mm loose spline section divided into seven modeled segments, and a 38 mm diameter grooved roller in line with the frame channel.

Return the roller to the start of its path, relax the mesh edge, and reset all seven spline segments to the loose height.

## Trial and score

Score the fraction of seven segments pressed at least 9 mm while mesh take-up exceeds 14 mm and still retained at the end.

Use Python 3.12, the Dream Labs dual-YAM reference and FFmpeg with libx264.
The nominal layout has a 60-second action cap. Calibration dependencies are
listed in requirements.txt.

```sh
python -m pip install -r requirements.txt
MUJOCO_GL=egl python run.py screen --out runs/demo --video
python run.py screen --out runs/idle --control
python run.py screen --out runs/incomplete --detail-control
python run.py screen --out runs/replay --replay runs/demo/commands.json
```

With the Dream Labs API credential available as `DREAM_API_KEY`, run the same nominal trial with Dream-1 beta. The rollout writes a video, final measurements and a request summary.

```sh
MUJOCO_GL=egl python evaluate.py screen --out runs/dream --video
```

## Fidelity

The roller is constrained to the local groove plane. Its wheel spins passively and the seating load follows local rim compression. The visible mesh deforms along fixed boundaries, with one physical tension degree of freedom. Seven separate contact joints control the continuous-looking spline.

## Physical benchmark

Use an ordinary frame, mesh, rubber spline and hand roller. Tension the mesh while seating the loose run and check that the repaired part stays flush after a gentle tug.
