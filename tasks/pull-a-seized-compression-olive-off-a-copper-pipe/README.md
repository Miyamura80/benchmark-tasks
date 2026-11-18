# Pull a seized compression olive off a copper pipe

When servicing a compression fitting, removing the old olive allows the pipe stub to be retained.

## Setup and reset

A copper tube model stands vertically from a base with a brass olive retained at its end. A crossbar puller is already coaxially engaged. A spare brass ring rests on a staging peg beside the pipe. The robot can lift and park the extractor and old ring away from the original tube.

Return the extractor, both ring positions and the transfer guides to their initial positions. Leave the spare ring on its staging peg.

## Trial and score

0.5 when the extractor breaks the old olive free and parks it clear of the original tube. Full credit when the spare ring is aligned to within 2 mm of that same tube and inserted over 22 mm.

Use Python 3.12, the Dream Labs dual-YAM reference and FFmpeg with libx264.
The nominal layout has a 60-second action cap. Calibration dependencies are
listed in requirements.txt.

```sh
python -m pip install -r requirements.txt
MUJOCO_GL=egl python run.py olive --out runs/demo --video
python run.py olive --out runs/idle --control
python run.py olive --out runs/incomplete --detail-control
python run.py olive --out runs/replay --replay runs/demo/commands.json
```

With the Dream Labs API credential available as `DREAM_API_KEY`, run the same nominal trial with Dream-1 beta. The rollout writes a video, final measurements and a request summary.

```sh
MUJOCO_GL=egl python evaluate.py olive --out runs/dream --video
```

## Fidelity

The screw-lead constraint transmits the crossbar rotation to the spent olive. The tool begins pre-engaged. Contact-gated guides constrain the extractor and replacement-ring transfer, and the spare ring has physical clearance around the original tube.

## Physical benchmark

Use a bench-mounted copper stub and a compatible mechanical olive puller. After extraction, visually check the original tube and slide the replacement ring down that same tube.
