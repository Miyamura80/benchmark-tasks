// Millimetres. Set part to export an individual printable part.
part = "layout";
$fn = 72;

module block(size, position) {
    translate(position) cube(size, center=true);
}

module carrier() {
    difference() {
        union() {
            block([20,20,4], [0,0,-5]);
            translate([0,0,-21.8]) cylinder(d=19, h=15);
        }
        // Tune the sliding clearance to the measured shaft before use.
        intersection() {
            translate([0,0,-22]) cylinder(d=8.4, h=15);
            translate([-10,-10,-22]) cube([13.2,20,15]);
        }
        translate([0,0,-5.5]) cylinder(d=6.15, h=2.6);
    }
}

module bracket() {
    difference() {
        union() {
            block([67,53,8], [0,0,-18]);
            for (y = [-38,38]) {
                translate([0,y,-34]) cylinder(d=20, h=18);
                for (x = [-16,16])
                    block([15,39,8], [x,y*.83,-18]);
            }
            block([26,12,6], [34.5,-49,-19]);
            block([22,17,6], [49,-50,-19]);
            block([20,20,8], [0,0,-11]);
            block([26,25,17], [0,0,0]);
        }
        for (y = [-38,38]) {
            translate([0,y,-35]) cylinder(d=8.6, h=20);
            translate([0,y,-34.01]) cylinder(d1=11.8, d2=8.6, h=6);
        }
        // The chip centre goes on the shaft axis. Cut a small recess for its
        // package and use thin nonmagnetic adhesive or standoffs for the PCB.
        block([10,10,1.5], [0,0,-22]);
    }
}

module catch_carriage() {
    difference() {
        union() {
            block([25,21,8], [0,0,-3]);
            block([10,14,5], [-6,12,3.5]);
            block([10,12,8.5], [-6,9,-2]);
            block([30,9,16], [0,-6,-9]);
        }
        for (x = [-8,8])
            translate([x,0,-11])
                rotate([90,0,0]) cylinder(d=2.7, h=30, center=true);
    }
}

module catch_guide() {
    // Assembled at (77,-64,116) relative to the shaft's table projection.
    // Both shoulder screws travel along the same 18 mm by 21 mm incline.
    difference() {
        union() {
            block([56,6,40], [-9,-15,-10.5]);
            block([56,20,6], [-9,-15,-30]);
        }
        for (x = [-8,8]) {
            hull() {
                for (end = [[0,0],[-18,-21]])
                    translate([x+end[0],-15,end[1]])
                        rotate([90,0,0]) cylinder(d=3.4, h=10, center=true);
            }
        }
        for (x = [-32,12])
            translate([x,-15,-35]) cylinder(d=4, h=10);
    }
}

if (part == "carrier") translate([0,0,21.8]) carrier();
if (part == "bracket") translate([0,0,34]) bracket();
if (part == "catch_carriage") translate([0,0,17]) catch_carriage();
if (part == "catch_guide") translate([0,15,27]) catch_guide();

if (part == "layout") {
    // Layout coordinates are measured from the table directly under the shaft.
    color("slategray") block([265,205,12], [-1,-10,6]);
    color("steelblue") translate([0,0,13]) cylinder(d=80, h=64);
    color("silver") {
        translate([0,0,77]) cylinder(d=8, h=15);
        for (y = [-38,38]) {
            translate([0,y,52]) cylinder(d=18, h=36.5);
            translate([0,y,88.5]) cylinder(d=8, h=11);
            translate([0,y,99.5]) cylinder(d1=8, d2=2, h=14);
        }
        for (x = [-31,31]) block([4,36,22], [x,0,89.5]);
    }
    color("orange") translate([0,0,99]) carrier();
    color("lightgray") translate([0,0,122.5]) bracket();
    color("red") translate([77,-64,127]) catch_carriage();
    color("gray") translate([77,-64,116]) catch_guide();
}
