// ==============================================
// Rydful-Beacon Case v6 — JLC3DP MJF
// - No ring ledge (holder too close to PCB edge)
// - 3 tapered nubs at 90°/210°/330° for PCB support
// - 0.5mm snap bumps + 0.05mm interference skirt fit
// - Fingernail pry notches at ±X (away from tabs)
// - Orient PCB: holder long axis along ±X
// ==============================================

pcb_dia       = 30;
pcb_thk       = 1.0;
comp_h        = 2.0;     // Max component height above PCB
batt_h        = 6.0;     // BS-8-1 + CR2032 below PCB (0.8mm margin)

wall          = 1.5;
gap           = 0.4;     // Radial PCB clearance
floor_h       = 1.2;
ceil_h        = 1.0;
chamfer       = 0.6;

// Snap-fit — firmer engagement
snap_bump     = 0.5;     // Bump protrusion
snap_h        = 1.5;     // Bump height
snap_chamf    = 0.6;     // Entry chamfer

skirt_d       = 3.0;     // Lid skirt depth (was 2.5)
lid_gap       = 0.05;    // Near-zero for interference fit (MJF ±0.2mm)

tab_w         = 8.0;
tab_ext       = 7.0;
tab_h         = 2.5;
screw_d       = 3.2;
tab_r         = 1.5;

engrave_depth = 0.4;   // Text engraving depth (mm)
engrave_w     = 16;    // Text width (mm)

$fn = 100;

// Derived
ir = pcb_dia/2 + gap;
or = ir + wall;
cav_h = batt_h + pcb_thk + comp_h;
base_h = floor_h + cav_h;
pcb_z = floor_h + batt_h;

echo(str("Outer Ø=", or*2, "  Base H=", base_h, "  Total=", base_h + ceil_h));
echo(str("PCB Z=", pcb_z, "  Inner Ø=", ir*2));

// ==============================================
// CHAMFERED CYLINDER
// ==============================================
module cham_cyl_bot(r, h, c) {
    hull() {
        translate([0, 0, c])
            cylinder(h = h - c, r = r);
        cylinder(h = 0.01, r = r - c);
    }
}

module cham_cyl_top(r, h, c) {
    hull() {
        cylinder(h = h - c, r = r);
        translate([0, 0, h - 0.01])
            cylinder(h = 0.01, r = r - c);
    }
}

// ==============================================
// MOUNT TAB
// ==============================================
module mount_tab() {
    overlap = 3;
    total_l = tab_ext + overlap;
    difference() {
        hull() {
            translate([-tab_w/2, -overlap, 0])
                cube([tab_w, 0.01, tab_h]);
            translate([-tab_w/2 + tab_r, total_l - overlap - tab_r, 0])
                cylinder(h = tab_h, r = tab_r);
            translate([tab_w/2 - tab_r, total_l - overlap - tab_r, 0])
                cylinder(h = tab_h, r = tab_r);
        }
        translate([0, tab_ext - tab_r - screw_d/2, -0.1])
            cylinder(h = tab_h + 0.2, d = screw_d);
    }
}

// ==============================================
// PCB SUPPORT NUBS
// ==============================================
// Tapered nubs on interior wall at pcb_z height
// Positioned in equilateral triangle at 90°/210°/330°
// All 30°+ away from ±X where holder pads reach PCB edge
module pcb_support_nubs() {
    nub_w = 5;       // Width along wall (mm)
    nub_depth = 2.0; // How far nub sticks inward (inner edge at 13.4mm from center)
    nub_h = 1.2;     // Height (supports PCB from below)

    // Nub positions: 90° (+Y, top), 210°, 330° — equilateral triangle
    // ALL positions are 30°+ away from ±X (holder long axis)
    // Holder short axis only reaches 7.85mm from center, so
    // at these angles the holder edge is well inside the nub zone.
    for (a = [90, 210, 330]) {
        rotate([0, 0, a])
        translate([ir - nub_depth, -nub_w/2, pcb_z - nub_h])
        hull() {
            // Full shelf at wall
            cube([0.1, nub_w, nub_h]);
            // Tapered inner edge for easy PCB drop-in
            translate([nub_depth, 0, nub_h - 0.4])
                cube([0.1, nub_w, 0.4]);
        }
    }
}

// ==============================================
// BASE
// ==============================================
module base() {
    difference() {
        union() {
            // Main shell
            cham_cyl_bot(or, base_h, chamfer);

            // Tabs
            translate([0, or, floor_h])
                mount_tab();
            rotate([0, 0, 180])
            translate([0, or, floor_h])
                mount_tab();

            // Snap bumps — 4x on inner wall near top
            // (added after hollow cut below)
        }

        // Hollow interior
        translate([0, 0, floor_h])
            cylinder(h = cav_h + 1, r = ir);

        // Fingernail pry notches — scoop at lid/base seam
        // At 0° and 180° (±X) — away from tabs (±Y) and nubs
        for (a = [0, 180]) {
            rotate([0, 0, a])
            translate([or - 1.2, 0, base_h - 1.0])
            scale([1, 2.5, 1])
            hull() {
                translate([0, 0, 0]) cylinder(d = 4, h = 0.01, $fn = 24);
                translate([1.5, 0, 2.0]) cylinder(d = 3, h = 0.01, $fn = 24);
            }
        }
    }

    // PCB support nubs (added after hollow)
    pcb_support_nubs();

    // Snap bumps — protrude inward from wall into cavity
    // Lid skirt slides past these; grooves in skirt catch them
    for (i = [0:3]) {
        a = i * 90 + 45;
        rotate([0, 0, a])
        translate([ir - snap_bump, -2, base_h - skirt_d + 0.5])
        hull() {
            // Full bump at wall
            translate([snap_bump - 0.1, 0, 0])
                cube([0.1, 4, snap_h - snap_chamf]);
            // Tapered tip toward center (entry chamfer)
            cube([0.1, 4, snap_h - snap_chamf]);
            // Chamfer top for smooth lid insertion
            translate([snap_bump - 0.1, 0, snap_h - snap_chamf])
                cube([0.1, 4, snap_chamf]);
        }
    }
}

// ==============================================
// LID
// ==============================================
module lid() {
    skirt_or = ir - lid_gap;         // Skirt outer = fits inside base wall
    skirt_ir = skirt_or - wall;      // Skirt inner

    difference() {
        union() {
            // Ceiling
            cham_cyl_top(or, ceil_h, chamfer);

            // Skirt (internal, nests inside base)
            translate([0, 0, -skirt_d])
            difference() {
                cylinder(h = skirt_d, r = skirt_or);
                translate([0, 0, -0.01])
                    cylinder(h = skirt_d + 0.02, r = skirt_ir);
            }
        }

        // Snap grooves — channels in skirt outer surface
        // When lid is inserted, bumps deflect the skirt inward,
        // then pop into these grooves for a click-lock
        for (i = [0:3]) {
            a = i * 90 + 45;
            rotate([0, 0, a])
            translate([skirt_or - snap_bump - 0.2, -2.3, -skirt_d + 0.3])
                cube([snap_bump + 0.4, 4.6, snap_h + 0.3]);
        }

        // Engraved "Rydful" text centered on lid top
        translate([0, 0, ceil_h - engrave_depth])
            linear_extrude(engrave_depth + 0.01)
                resize([engrave_w, 0], auto=true)
                    text("Rydful", halign="center", valign="center",
                         font="Liberation Sans:style=Bold");
    }
}

// ==============================================
// RENDER — comment out for export
// ==============================================
// base();
// rotate([180,0,0]) lid();

color("DimGray") base();
color("SlateBlue", 0.7) translate([0, 0, base_h]) lid();
%translate([0, 0, pcb_z]) cylinder(h = pcb_thk, r = pcb_dia/2);
%translate([0, 0, floor_h + 0.5]) cylinder(h = 3.2, d = 20);
