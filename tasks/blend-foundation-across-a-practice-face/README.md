# Blend foundation across the cheeks and nose

Pick up a foam makeup sponge, load it from an open palette, and apply an even base layer to both cheeks and the nose. Keep the lips and eyelids clean. The scripted demo returns the sponge to its dock. The station uses two YAM arms, with the left arm performing the scripted application.

## Why it matters

In a 2023 YouGov survey of 1,000 U.S. adult citizens, 35% of women surveyed reported using foundation regularly. The survey measures foundation use across application methods. This task isolates the pressure-controlled sponge application around curved facial features.

[YouGov article](https://yougov.com/en-us/articles/48130-why-american-women-wear-makeup-poll) · [Survey results](https://d3nkl3psvxxpe9.cloudfront.net/documents/Makeup_poll_results.pdf)

## Physical setup and reset

| Item | Starting specification |
|---|---|
| Face | Reusable molded silicone makeup practice face, approximately 166 × 126 mm. Choose one with defined eyelids and lips and a raised nose. Record the supplier, dimensions and shade for the batch. |
| Base | Flat rigid board, about 310 × 390 mm. Secure the face face-up on a rigid 170 × 130 mm backing plate using perimeter clamps outside the application regions. Mark its footprint and orient the forehead toward one end of the board. |
| Tools | One roughly 40 mm foam makeup sponge, a shallow palette around 56 mm in diameter, and a 64 × 72 mm sponge dock. Mark each footprint. |
| Layout | Place the palette center 46 mm toward the chin and 104 mm to the side from the face center. Place the dock center 59 mm toward the forehead and 104 mm to the same side. Put the palette surface about 12 mm and the dock surface about 26 mm below the face mesh base plane. |
| Coating | Select one liquid foundation shade with visible contrast against the silicone. Start with 1.0 mL of the same product in the palette, measured with a graduated syringe. Fix the product lot, shade and supply amount after calibration. |

Clean the face according to the supplier instructions and compare the dried surface with its clean reference photo. Reposition it against the same stops. Place a fresh dry sponge of the same type on the dock, clean and refill the palette, and keep the camera and lighting fixed.

## Physical photo and force checks

1. Mount an overhead camera over the fixed board. Use diffuse lights on both sides. Lock camera position, focus, exposure and white balance, and keep a gray reference card in view.
2. Capture a clean image and a fully coated reference of the same face after manually applying one even coat over the labeled regions. Clean and reset before the robot trial. Photograph the trial after the same drying interval as the reference.
3. Draw an aligned grayscale PNG mask. Label the interiors of both cheeks 1 and 2, the nose 3, and the lips and eyelids 4. Leave the background and uncertain boundary pixels at 0. Save this mask for the entire batch.
4. Run the command below. It compares each trial pixel’s color shift with that of the clean-to-coated reference. The output includes usable coverage by target zone and maximum protected-region transfer. Test the photo procedure with a clean negative, a deliberately contaminated negative and a manually coated positive before using the thresholds to compare robot trials.
5. Mount the face backing on a tared load cell and sample normal force at 100 Hz or faster. Report peak load and cumulative time above 5.5 N. Compare that limit with measured manual sponge applications on the same fixture before adopting it for a physical pass criterion. Save photos and force logs for the calibration controls.

    python score_photos.py clean.png reference.png trial.png regions.png

The physical protocol is proposed. The supply amount, color thresholds and 5.5 N load limit still need bench calibration.

## Simulation score and limitations

- Each cheek and the nose must have at least 93% usable samples for full credit. A usable sample reaches 82% of the reference coating. The maximum transfer to a protected sample must be at most 20%, with less than 0.15 seconds over 5.5 N.
- With at most 35% protected-region transfer, combined usable coverage of 25% or 55% earns 0.25 or 0.50. Transfer above 35% gives zero credit.
- The sponge has a spring-mounted working pad. Native contact loads govern pickup and simulated pigment transfer, and local collision tiles approximate the curved face. The transfer model approximates visible coating. It omits film thickness, drying and absorbency.

## Reproduce

Use Python 3.12, FFmpeg with libx264, EGL and DejaVu Sans fonts.

    python -m pip install -r requirements.txt
    python -m pip install --no-deps dreamlabs==1.0.23 git+https://github.com/kairosiann/inspect-robots-dream.git@b5e936a201cec66c54c297e522c920d98445e339
    export MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1
    python run.py foundation --out runs/scripted --video
    python run.py foundation --out runs/replay --replay evidence/scripted/commands.json
    python run.py foundation --out runs/idle --control
    python run.py foundation --out runs/one-cheek --detail-control
    python evaluate.py foundation --out runs/dream --video

The Dream-1 command requires an authorized Dream API key. Both runs share the simulator and native scorer. The baseline uses three RGB views and measured joint and gripper positions. The episode cap is 1,200 actions at 20 Hz (60 seconds). The recorded fixture uses variant 0 and a fixed reset pose.

## Asset provenance

The face mesh and texture, palette, sponge and bench fixtures are generated by the task code. The station loads the I2RT Robotics YAM assets from the DreamLabs 1.0.23 MuJoCo example. The example's asset directory carries the MIT license and I2RT Robotics copyright notice. It supplies the base, links, gripper and fingertip STL meshes. The example also loads oak and glaze textures and the inherited apple and banana examples. Neither fruit appears in this scene. The SDK retains its Apache 2.0 license with the asset exceptions documented in the package. This folder contains no copies of SDK assets.

[DreamLabs 1.0.23 and licensing notes](https://pypi.org/project/dreamlabs/1.0.23/)

## Recorded validation

The first Dream-1 request interpreted the unqualified face as a person and was refused. The completed baseline explicitly described the fixed silicone practice face. The scene and scorer were unchanged.

| Trial | Score | Evidence |
|---|---:|---|
| Scripted | 1.00 | 836 actions, 95.8% usable coverage, 0.0% peak protected transfer, 5.11 N peak load. The final second held full score. |
| Fresh command replay | 1.00 | Reproduced the recorded coverage, transfer and peak force. |
| Idle control | 0.00 | No application. |
| One-cheek control | 0.25 | Partial coverage. |
| Dream-1 | 0.00 | 1,200 actions and 25 successful requests in one session. |

The photo checker passed synthetic checks for clean baseline, complete coverage, incomplete coverage, protected transfer and weak reference contrast. Actual foundation deposits and contact-force thresholds remain uncalibrated on the physical fixture.
