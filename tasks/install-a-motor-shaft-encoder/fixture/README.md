# Build the motor encoder fixture

[parts.scad](parts.scad) contains the part geometry and an assembled view. For each printable piece, substitute `carrier`, `bracket`, `catch_carriage` or `catch_guide` for `bracket` in the command below.

```bash
openscad -D 'part="bracket"' -o bracket.stl parts.scad
```

## Mechanical assembly

Use PETG or acetal for the carrier, bracket and catch. Measure and deburr the printed bore before assembling. Retain the flat when fitting the carrier. Test it on the unpowered fixture before attaching the sensor.

| Assembly | Parts and mounting |
| --- | --- |
| Base and shaft | Fasten the unpowered motor to the base. Use a removable Ø8 mm steel training shaft with 15 mm exposed length. Machine the flat 3.00 mm from the centreline facing +X. At 5.00 mm below the tip, cut a Ø6.90 mm groove with a 1.40 mm floor and 0.70 mm ramps on both sides. Chamfer the tip over the last 1.50 mm as in the drawing. Use a spacer or mounting plate to set the heights in the layout. |
| Carrier | The bore locating land is Ø8.50 mm with the flat 3.25 mm from the centreline. Tap the side boss M4 × 0.7 and fit an adjustable stainless spring ball plunger with an M4 body and about 0.55 mm usable ball travel. The plunger points toward the shaft axis from −X and its centre meets the groove at the carrier stop. Seat the keyed bore to its end stop. Fit the Ø6 × 2.5 mm diametrically magnetized disc in its centred pocket, flush with the top. |
| Bracket | Use two vertical Ø8 mm pins on 76 mm centres. Set the Ø18 mm guide shoulders 88.5 mm above the table. The two 4 × 36 mm crossbar stops sit 100.5 mm above it at X = ±31 mm from the shaft axis, outside the PCB footprint. Fit the locating bores over the pin lead-ins. Fix the break-out PCB under the crossbar with its chip centred over the shaft. Use nonmagnetic shims to set the gap between the chip face and magnet to 1.5 mm. |
| Catch | Fix the slotted guide to a rigid spacer with its slots at the position shown in the layout. Fit two M3 shoulder screws 16 mm apart through the slots and into the carriage, with low-friction washers and enough slack for free travel. Use a light return spring along the slot direction and a spring-ball plunger or snap detent at the closed stop. Reset the carriage by hand. At the open stop the tongue clears the bracket tab's vertical path. At the closed stop it overlaps the tab by at least 3 mm and the vertical gap is at most 3 mm. |
| Key | Use a normally open momentary switch below a spring-return plunger with a Ø18 mm blue cap, 6 mm hard travel stop and switch actuation by 4.5 mm. The switch must open again when released. |

## Wiring

Use an Arduino Uno R3, an Adafruit AS5600 breakout, a normally open switch, two LEDs and two 330 Ω resistors. Power the Uno over USB and leave the motor disconnected.

| Uno | Connection |
| --- | --- |
| 5 V, GND | AS5600 VIN and GND |
| A4, A5 | AS5600 SDA and SCL, respectively |
| D2 | Switch to ground. The software enables the internal pull-up. |
| D8, D9 | Green and red LED anodes, respectively, each through 330 Ω. Connect both cathodes to ground. |

Flash [field_check.ino](field_check/field_check.ino) with Arduino IDE and the Arduino AVR Boards package. Hold the key for at least 300 ms. The green light means that the AS5600 responded at address `0x36` and its STATUS register reports a detected magnet with neither field limit flag set. A missing response, missing magnet or an out-of-range field lights red. Cover the breakout's power LED to distinguish it from the two result LEDs. [Breakout pinout](https://learn.adafruit.com/adafruit-as5600-magnetic-angle-sensor/pinouts) · [AS5600 datasheet](https://www.mouser.com/datasheet/3/5912/1/AS5600_DS000365_5_00.pdf)

## Bench checks and scoring

Before the first trial, back out the ball plunger and verify that the aligned carrier seats by hand and that a 90° misaligned push leaves the carrier at least 2 mm above its seat. Refit the plunger, set the installed axial breakaway force to 3–5 N using a spring scale and mark the adjustment for repeatable reset. Apply this setting only to the removable unpowered training shaft.

1. With the bracket seated, first test the circuit with the magnet in place, with the magnet removed and with the sensor unplugged. Require green, red and red, respectively. If the installed magnet fails, check alignment and adjust the gap using shims. Record the measured gap and the magnet and sensor used.
2. Mark the carrier stop and its +X datum. Score 0.25 if the released carrier is within 0.65 mm of the stop and the flats align within 11.5°. Add 0.20 if a 3 mm feeler gauge cannot enter under either bracket seat and both guide pins are engaged.
3. Add 0.20 if the closed catch stays in its detent after release and the tongue covers the tab by at least 3 mm. From the side, confirm the bracket cannot rise more than 3 mm before the tab meets the tongue.
4. After both grippers clear, hook a thin split fork under the carrier flange from the open side and pull the fork axially upward using a 0–10 N spring scale. Hold 2.0 N for 0.30 s and verify that the carrier rises no more than 0.50 mm with a dial indicator or a datum mark and feeler gauge. The fork must touch the carrier only. Full credit requires this proof check, a 0.30 s press lighting green, and a released key. Reset the bracket, carrier and catch before the next trial. Power cycle the Uno to clear the diagnostic LEDs.
