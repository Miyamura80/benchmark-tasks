# Clear a thermal label printer jam

Thermal label printers are familiar to small shops and people shipping parcels from home. Clearing the backing from the opened platen and feeding fresh stock tests that the label path works again.

## Setup and reset

A 208 × 254 mm clamshell label-printer model with the cover open, a roll of 126 mm wide stock, a rubber platen with a left-side bearing and one folded backing strip caught in the feed path. A teal feed button is on the right edge of the front housing. Stock is already routed to the inlet.

Lower the platen to its start, put the folded strip in the pinch path, and reset stock travel and feed-button state to zero.

## Trial and score

0.5 for lifting the platen over 27 mm and pulling the scrap over 66 mm. Full credit also requires the platen to return within 12 mm of its seat, fresh stock to advance over 74 mm on the feed button, and the platen to turn over 5.5 rad.

Use Python 3.12, the Dream Labs dual-YAM reference and FFmpeg with libx264.
The nominal layout has a 60-second action cap. Calibration dependencies are
listed in requirements.txt.

```sh
python -m pip install -r requirements.txt
MUJOCO_GL=egl python run.py printer --out runs/demo --video
python run.py printer --out runs/idle --control
python run.py printer --out runs/incomplete --detail-control
python run.py printer --out runs/replay --replay runs/demo/commands.json
```

With the Dream Labs API credential available as `DREAM_API_KEY`, run the same nominal trial with Dream-1 beta. The rollout writes a video, final measurements and a request summary.

```sh
MUJOCO_GL=egl python evaluate.py printer --out runs/dream --video
```

## Fidelity

The backed paper flexes visually with the extractor displacement and curls as it feeds past the tear bar. A compliant actuator couples the platen rotation to the advancing stock. Paper contact uses a rigid guided proxy, with the roll prethreaded and no label sensor.

## Physical benchmark

Use a desktop thermal printer and scrap backing. Clear the strip, place the platen back between its bearings, and check that one fresh label reaches the tear bar. Add manual rerouting to match the full design.
