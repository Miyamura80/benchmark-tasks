// Pump lever in mm. Print flat or cut from 3 mm acetal.
// Two M3 adjuster/pad holes (local x=10 and x=40); pads sit on opposite faces.
$fn = 48;
difference() {
    hull() {
        for (x = [0, 38]) translate([x, 0, 0]) cylinder(h=3, r=6);
    }
    translate([0, 0, -1]) cylinder(h=5, d=3.2); // pivot
    for (x = [10, 40]) translate([x, 0, -1]) cylinder(h=5, d=2.5); // tap M3
}
