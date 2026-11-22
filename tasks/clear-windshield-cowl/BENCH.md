# Removable windshield-cowl fixture

The working section fits on its own table between two YAMs. A full vehicle, overhead bridge and cantilever arms are unnecessary. The cheapest option for a lab with an installed bimanual YAM station is to reuse its supported arm bases and build just the insert. Estimated consumables in USD: plywood and battens $15–30, ABS or polypropylene sheet and lip stock $10–20, clear polycarbonate offcut $10–20, tray and fasteners $10–20, leaf stock and ballast $5–10. Total **$50–100** is a planning allowance, excluding robots, existing station, tools, labour and taxes. A new pair of industrial pedestals is a separate capital cost.

## Cut and assemble

All coordinates are in millimetres. Stand facing the windshield. Positive x is to the right, positive y toward the screen. Mark x=0 at the gutter centre, y=0 along the rear edge of the hood strip. Take all heights relative to the **top of the gutter floor**. The nominal height of that surface in the simulation is 790 mm above the floor. Raise or lower the entire fixture to match installed arm bases.

| Part | Dimensions and position |
| --- | --- |
| Deck | 820 × 740 × 20 plywood, clamped to an independent bench. Set the top 100 mm below the gutter surface. Fit support blocks under the gutter and tray, clear of the drain openings. The deck only carries the fixture. |
| Gutter | Use smooth black ABS on a rigid backing. The rear floor spans x=±450, y=−30 to +124. Narrow the front floor to x=±390 from y=−165 to −30, leaving room for the supports. Add a 16 mm hood-edge strip, 760 × 126, centred at (0,−64), top +21. |
| Drains | Cut two 70 mm-wide slots centred at x=−340 and +302, open from y=124 to 153. Fit five 5.4 × 40 × 6 mm slats, centres offset −27, −15, −3, +9, +21 from each drain centre, top +6. Fit a 66 × 14 × 8 retaining lip centred at y=150 with its underside +16. Verify a 10 mm gap above the slats. Keep black backing below the openings, outside the measurement plane. |
| Screen and tray | Bolt a blunt-edged clear polycarbonate sheet, 886 × 260, to side cheeks at 43° above horizontal, rising toward +y. Centre it at (0,256), height +94. Fit a removable tray at (0,−246), floor +55, outer dimensions about 254 × 212 with a 27 mm raised rim. Mark the 230 × 176 scoring rectangle inside. |

A salvaged cowl can provide the visible shell. Retain the two specified slot inserts, lips and tray positions. Check the gutter surface height and measure both slot centres after fastening. Remove sharp exposed edges and use polycarbonate for the screen. The rear of a small car can differ by model year; a shell that cannot accept the marked inserts changes the trial.

## Arm support and limits of the validation

For an existing **I2RT YAM Standardized Teleoperation Station**, keep the OEM follower mounting plates, hardware and supporting frame in their commissioned configuration. Position the independent cowl fixture between the followers. The nominal base faces are 930 mm apart, at x=±465, y=−120, **82 mm above the gutter floor**. Left and right base yaw are 40° and 140° about +z from +x. Translate the fixture as a unit to match station height. Measure base-to-base spacing and yaw first. A station that cannot attain this layout with its approved mounting positions needs a separately verified setup. The manufacturer supplies the follower arms, cell and workspace table together. [I2RT station](https://i2rt.com/products/yam-teleoperation-stations) · [YAM installation](https://doc.i2rt.com/products/yam).

For a lab using separate, directly supported arm bases, a rated example is the **Vention ST-RB-033-0945__2** floor-mounted pedestal for each arm. The manufacturer specifies 2,250 N vertical, 1,300 N lateral, 1,700 N·m twist and 1,900 N·m bending under its mounting conditions, including emergency stops. Its 945 mm nominal height changes the absolute fixture elevation; set the gutter 82 mm below the measured base faces after assembly. **YAM is absent from Vention’s listed mounting patterns.** Obtain a compatible adapter and combined load approval for the specific YAM, plate, anchors and floor from the supplier or responsible integrator. Use their bolt schedule and substrate specification. Do not treat a tabletop mass rating or those column ratings as approval for a fabricated adapter. [Part numbers, ratings and installation instructions](https://vention.io/resources/515/cobot-pedestal-515/download.pdf).

Record the installed station or approved adapter drawing, anchorage, base coordinates, clearances and stop checks. Start with a slow empty dry run and confirm that links, pads and cables clear the real plates and fixture before running the leaf trial. The replay checks collisions against the **modelled support envelope** during the demonstration. It cannot establish real stopping loads, anchor capacity or structural safety. Neither mounting option was physically tested for this submission.

## Reusable leaf set and reset

1. Print leaf-templates.svg on A4 at actual size. Measure the 50 mm scale bar. Cut one blade per outline, within 1 mm, from the same sheet of 0.25–0.35 mm matte polypropylene report-cover stock. Use ochre/brown stock or mark each blade with its number on the underside. Mark the arrow as the windshield-facing tip. Fit a short flexible stem at the opposite end extending to 38 mm × the printed scale factor from the blade centre.
2. Form a shallow longitudinal fold with the centre ridge facing up. Attach narrow strips of removable putty along the central underside to reach 1.5 ± 0.1 g per piece. Shape the centre section to 6 ± 1 mm total thickness and target a 3–5 mm raised midrib with curled margins. Weigh each assembled leaf and check dimensions against the printed patterns. Reuse this labelled set across comparison runs. The standardized pieces represent curled dry leaves; no natural-leaf population is assumed.
3. Transfer the centres in the table to the gutter with removable dots. Place **midrib up**, stem toward the hood (−y), arrow toward the windshield (+y), yaw 0° ± 3° for all four. Centre tolerance is ±2 mm. Slide the tips of 1 and 4 under the lips by the overlap implied by the centre and outline; 2 and 3 remain free. Do not tuck any part underneath a slat. Release, wait at least 1.32 seconds, and check the reset before each attempt.

| ID | Blade length × maximum width | Scale factor | Centre x, y | Nominal tip beyond the front edge of the lip |
| --- | --- | ---: | --- | --- |
| 1 | 59.8 × 33.1 | 0.92 | −340, 126 | 14.3 mm (blade crosses the 14 mm lip) |
| 2 | 66.3 × 36.7 | 1.02 | −135, 83 | Free |
| 3 | 52.0 × 28.8 | 0.80 | 75, 104 | Free |
| 4 | 72.8 × 40.3 | 1.12 | 302, 124 | 19.1 mm (blade crosses the 14 mm lip) |

Photograph each drain from directly overhead and each trapped tip from the side. Start only with the tray empty, both tips captured, both grilles at least 5% covered and a total score of zero. Apply the normal tray and open-slot scoring rule after the robot retreats.
