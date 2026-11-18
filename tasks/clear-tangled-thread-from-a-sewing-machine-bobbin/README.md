# Clear tangled thread from a sewing-machine bobbin

Recovering the lower thread path involves work below the needle plate before the machine can be restarted.

## Setup and reset

A domestic sewing-machine mock-up with the cover removed, a 38 mm diameter bobbin in its well, trapped blue thread, a raised foot and a handwheel coupled to a moving needle. The mock-up handwheel turns in the horizontal plane.

Seat the bobbin, restore the trapped thread and reset the handwheel and needle travel.

## Trial and score

0.5 for lifting the bobbin over 33 mm and drawing the thread bundle over 33 mm out of the path. Full credit when the bobbin is back within 11 mm of its seat and the needle strokes over 4 mm in the handwheel check.

Use Python 3.12, the Dream Labs dual-YAM reference and FFmpeg with libx264.
The nominal layout has a 60-second action cap. Calibration dependencies are
listed in requirements.txt.

```sh
python -m pip install -r requirements.txt
MUJOCO_GL=egl python run.py bobbin --out runs/demo --video
python run.py bobbin --out runs/idle --control
python run.py bobbin --out runs/incomplete --detail-control
python run.py bobbin --out runs/replay --replay runs/demo/commands.json
```

With the Dream Labs API credential available as `DREAM_API_KEY`, run the same nominal trial with Dream-1 beta. The rollout writes a video, final measurements and a request summary.

```sh
MUJOCO_GL=egl python evaluate.py bobbin --out runs/dream --video
```

## Fidelity

The visible thread is loose and changes shape as it is extracted. Its physical guide uses lift-dependent resistance to approximate the pinch. A slider-crank models the needle drive. The proof verifies needle movement, without stitch formation or bobbin thread pickup.

## Physical benchmark

Use an unpowered sewing-machine fixture, raise the foot, clear the lower thread path and reinstall the bobbin. For the full design, show that turning the handwheel picks up a loop of bobbin thread.
