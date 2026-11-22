# Reseat a dropped bicycle chain

A chain has fallen between the front chainring and the frame. One robot grips the upper run, lifts it clear of the chainstay and lays it on the front teeth. The other turns the crank. The repair passes when the released chain transmits two uninterrupted crank revolutions to the rear sprocket and the pedal tool withdraws.

## Frame-version evidence

| Run | Score | What happened |
| --- | ---: | --- |
| Continuous scripted repair | 1.00 | 810 actions at 20 Hz (40.5 s). After the guide cleared, 3.5870 front turns drove 6.2880 rear turns. All 86 rollers visited front pockets. The run ended with 17 front and 10 rear pockets occupied. |
| Independent command replay | 1.00 | Every sampled score, pocket count, roller count and shaft angle matched. Zero MuJoCo solver warnings. |
| Crank only | 0.00 | The front made 2.371 turns. The rear moved −0.014 turns from reset, including settling. No rollers occupied front pockets at the end. |
| Idle | 0.00 | The chain stayed off the front ring. |
| Dream-1 | 0.00 | 1,200 actions on the same frame and jaws. It finished with 0 occupied front pockets and 9 rear pockets. The model video is 60.9 s. |

The first crank turn in the scripted video demonstrates the fault. Between actions 48 and 158 the crank turns approximately once while the rear index stays still. The repair follows in the same episode. After the guiding jaws withdraw, the rear sprocket follows the crank. The pedal socket then releases. The episode ends at action 810. The video contains 414 H.264 frames at 10 fps, including the final hold (41.4 seconds).

## Reproduce

The retained runs used Debian Linux x86_64, Python 3.12.16, MuJoCo 3.3.7, the pinned dual-YAM assets and FFmpeg with libx264. Install the dependencies in a fresh virtual environment. The scripted controller observes simulated roller positions to select its initial grasp. The Dream policy receives the standard three camera views and robot state.

```sh
python -m pip install -c constraints-tested.txt numpy Pillow mujoco inspect-robots dreamlabs-client
python -m pip install --no-deps dreamlabs==1.0.23 https://codeload.github.com/kairosiann/inspect-robots-dream/tar.gz/b5e936a201cec66c54c297e522c920d98445e339
export MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1
python run.py chain --show-fault --video --out runs/demo
python run.py chain --replay runs/demo/commands.json --out runs/replay
python run.py chain --control --out runs/idle
python run.py chain --crank-only --out runs/crank
python evaluate.py chain --video --out runs/dream
```

The last command requires a configured Dream account. The submitted numeric results are in `evidence/`. The demonstration command above generates the 14-dimensional robot commands and trace for replay. The task-board media set contains both videos and the setup image. Compare the replay and scripted traces by action number. Replay uses its own phase label.

## Build and reset

[Frame, tool dimensions and assembly](BUILD.md). Two moving finger extensions grip the chain's side plates. Collision-enabled frame tubes, chainstays, seatstays and a bottom-bracket shell constrain access. The chain drops onto the bottom-bracket area under gravity. The station has no front catcher. The rear disc guides and 58 mm service crank are retained simplifications. The rear wheel and tyre are omitted. The model uses parallel roller axes, simplified plates, and rigid tools. Fabrication and repair on hardware remain unverified.

With the chain rear-threaded and the crank mark at its reference angle, lay the front loop 11 mm inboard of the front tooth plane. Hold it there with a removable spacer, then remove the spacer. Let the chain settle onto the shell/stay. Confirm zero front pocket occupancy in simulation. On a physical fixture, slowly turn the crank once and check that the rear mark stays still. A repair demonstration must begin from the dropped condition.

## Judging the repair

Mark both sprockets. After the guiding jaws open and move away, turn the crank twice. The rear mark must make approximately 3½ turns. Release the pedal. The chain must remain wrapped around both sprockets. This is the physical check proposed for the build.

The native scorer detects rollers in tooth pockets using lateral, radial and tooth-phase errors. The drive proof requires at least four occupied front pockets, three rear pockets, and 10 mm clearance from the entire jaw to the chain envelope. If mesh or clearance is lost, or the rear angle differs from the 28:16 ratio by more than 0.040 revolution, the proof restarts. The final score requires at least two crank turns, 3½ rear turns, every roller identity passing the front ring, at least eight occupied front and five rear pockets, and a released pedal socket. No motor or kinematic constraint couples the shafts.

## Third-party assets

The dual-YAM scene comes from Dream Labs (`dreamlabs==1.0.23`, Apache-2.0). The stock YAM geometry is derived from I2RT (MIT). The two generated shoe cavities fit that geometry. The asset notice is included in `hardware/YAM-LICENSE.txt`. MuJoCo is Apache-2.0. The recording uses DejaVu Sans with the DejaVu/Bitstream Vera font license. The frame, chain, sprockets, extensions and socket are generated from the included dimensions.

[Dream Labs](https://github.com/dreamlabs-co/dreamlabs/tree/main/dream_library) · [I2RT](https://github.com/i2rt-robotics/i2rt) · [DejaVu notice](https://dejavu-fonts.github.io/License.html)
