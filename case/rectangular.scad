// =============================================
// Rydful-Beacon Rectangular Case v1 — 2×AA
// - 2×AA side by side with spring contacts
// - PCB 17.8×38.1mm beside batteries
// - Snap-fit lid, rounded corners
// - Orient: batteries along X, PCB beside in Y
// =============================================

// AA Battery
aa_dia        = 14.5;   // Battery diameter
aa_len        = 50.5;   // Battery length (incl. positive nub)
spring_cl     = 3.5;    // Spring contact clearance per end
batt_gap      = 1.0;    // Gap between two batteries
batt_cl       = 0.5;    // Battery clearance to walls
cradle_depth  = 3.0;    // Cradle channel depth

// PCB
pcb_l         = 38.1;   // Length (X)
pcb_w         = 17.8;   // Width (Y)
pcb_thk       = 1.6;
comp_h        = 2.0;    // Max component height (top side)
pcb_gap       = 0.3;    // Clearance around PCB

// Case
wall          = 1.5;
floor_h       = 1.2;
ceil_h        = 1.0;
corner_r      = 2.0;
chamfer       = 0.6;
div_wall      = 1.0;    // Divider between battery/PCB sections

// Snap-fit
snap_bump     = 0.4;
snap_h        = 1.2;
snap_chamf    = 0.3;

// Lid
skirt_d       = 2.5;
lid_gap       = 0.1;
sk_wall       = 1.2;    // Skirt wall thickness

// PCB support
standoff_h    = 2.0;
standoff_d    = 3.0;

// Text engraving
engrave_depth = 0.4;
engrave_w     = 30;     // Text width (mm)

// Spring contact pockets
contact_w     = 12.0;   // Contact plate width
contact_h     = 10.0;   // Contact plate height
contact_d     = 1.0;    // Pocket depth into wall

$fn = 60;

// ---- Derived ----
batt_sec_x = aa_len + 2 * spring_cl;
batt_sec_y = 2 * aa_dia + batt_gap + 2 * batt_cl;
pcb_sec_y  = pcb_w + 2 * pcb_gap;

cav_x = batt_sec_x;
cav_y = batt_sec_y + div_wall + pcb_sec_y;
cav_z = aa_dia + batt_cl;

ext_x = cav_x + 2 * wall;
ext_y = cav_y + 2 * wall;
base_h = floor_h + cav_z;

// PCB origin inside cavity
pcb_ox = wall + (cav_x - pcb_l) / 2;
pcb_oy = wall + batt_sec_y + div_wall + pcb_gap;

echo(str("Case: ", ext_x, " × ", ext_y, " × ", base_h + ceil_h, " mm"));
echo(str("Battery section: ", batt_sec_x, " × ", batt_sec_y, " mm"));
echo(str("PCB section: ", cav_x, " × ", pcb_sec_y, " mm"));

// =============================================
// ROUNDED RECTANGLE (2D)
// =============================================
module rrect(x, y, r) {
    offset(r) offset(-r)
        square([x, y]);
}

// =============================================
// CHAMFERED ROUNDED RECTANGLE
// =============================================
module cham_rrect_bot(x, y, h, r, c) {
    hull() {
        linear_extrude(0.01)
            translate([c, c])
                rrect(x - 2*c, y - 2*c, max(r - c, 0.1));
        translate([0, 0, c])
            linear_extrude(h - c)
                rrect(x, y, r);
    }
}

module cham_rrect_top(x, y, h, r, c) {
    hull() {
        linear_extrude(h - c)
            rrect(x, y, r);
        translate([c, c, h - 0.01])
            linear_extrude(0.01)
                rrect(x - 2*c, y - 2*c, max(r - c, 0.1));
    }
}

// =============================================
// BASE
// =============================================
module base() {
    difference() {
        // Outer shell (chamfered bottom edge)
        cham_rrect_bot(ext_x, ext_y, base_h, corner_r, chamfer);

        // Internal cavity
        translate([wall, wall, floor_h])
            cube([cav_x, cav_y, cav_z + 1]);

        // Spring contact pockets — recesses in end walls (glue contacts in)
        for (i = [0, 1]) {
            batt_cy = wall + batt_cl + aa_dia/2 + i * (aa_dia + batt_gap);
            pocket_z = floor_h + cradle_depth - 1;

            // Negative end (X=0 wall)
            translate([wall - contact_d, batt_cy - contact_w/2, pocket_z])
                cube([contact_d + 0.1, contact_w, contact_h]);
            // Positive end (X=max wall)
            translate([wall + cav_x - 0.1, batt_cy - contact_w/2, pocket_z])
                cube([contact_d + 0.1, contact_w, contact_h]);
        }

        // Fingernail pry notches — centered on long sides
        for (dy = [0, ext_y]) {
            translate([ext_x/2, dy, base_h - 0.8])
            scale([3, 1, 1])
            hull() {
                cylinder(d = 4, h = 0.01, $fn = 24);
                translate([0, dy == 0 ? -1.2 : 1.2, 1.5])
                    cylinder(d = 3, h = 0.01, $fn = 24);
            }
        }
    }

    // ---- Internal features (added after hollow) ----

    // Battery cradle — raised floor with cylindrical channels
    difference() {
        translate([wall, wall, floor_h])
            cube([batt_sec_x, batt_sec_y, cradle_depth]);

        for (i = [0, 1]) {
            by = wall + batt_cl + aa_dia/2 + i * (aa_dia + batt_gap);
            translate([wall - 0.1, by, floor_h + cradle_depth])
                rotate([0, 90, 0])
                    cylinder(d = aa_dia + 0.6, h = batt_sec_x + 0.2);
        }
    }

    // Divider wall (70% height — open top for wire routing)
    translate([wall, wall + batt_sec_y, floor_h])
        cube([cav_x, div_wall, cav_z * 0.7]);

    // PCB standoffs — 4 posts near PCB corners
    for (dx = [1.5, pcb_l - 1.5], dy = [1.5, pcb_w - 1.5])
        translate([pcb_ox + dx, pcb_oy + dy, floor_h])
            cylinder(d = standoff_d, h = standoff_h, $fn = 20);

    // PCB slide-in rails (Y sides) — L-shaped channels that capture PCB edges
    rail_lip   = 0.6;    // Lip overhang over PCB edge
    rail_wall  = 0.8;    // Rail outer wall thickness
    rail_h     = standoff_h + pcb_thk + 0.3;  // Full height incl. lip
    rail_slot  = pcb_thk + 0.2;  // Slot height (PCB + clearance)

    for (side = [0, 1]) {
        ry = side == 0
            ? pcb_oy - rail_wall          // Near side
            : pcb_oy + pcb_w;             // Far side
        translate([pcb_ox, ry, floor_h]) {
            // Base wall (full height below PCB lip)
            cube([pcb_l, rail_wall, standoff_h]);
            // Upper wall (below PCB)
            cube([pcb_l, rail_wall + rail_lip, standoff_h]);
            // Lip overhang (above PCB slot)
            translate([0, 0, standoff_h + rail_slot])
                cube([pcb_l, rail_wall + rail_lip, rail_h - standoff_h - rail_slot]);
        }
    }

    // PCB end stops (X sides) — short walls at both ends
    stop_h = standoff_h + pcb_thk + 0.3;
    stop_w = 0.8;
    // Near X end
    translate([pcb_ox - stop_w, pcb_oy, floor_h])
        cube([stop_w, pcb_w, stop_h]);
    // Far X end
    translate([pcb_ox + pcb_l, pcb_oy, floor_h])
        cube([stop_w, pcb_w, stop_h]);

    // Snap bumps — protrude from inner walls into cavity
    bump_z = base_h - skirt_d + 0.5;
    bump_w = 6;

    // On Y walls (long sides)
    for (sx = [wall + cav_x * 0.3, wall + cav_x * 0.7]) {
        // Near Y wall
        translate([sx - bump_w/2, wall, bump_z])
        hull() {
            cube([bump_w, snap_bump, snap_h - snap_chamf]);
            translate([0, 0, snap_h - snap_chamf])
                cube([bump_w, 0.1, snap_chamf]);
        }
        // Far Y wall
        translate([sx - bump_w/2, wall + cav_y - snap_bump, bump_z])
        hull() {
            cube([bump_w, snap_bump, snap_h - snap_chamf]);
            translate([0, snap_bump - 0.1, snap_h - snap_chamf])
                cube([bump_w, 0.1, snap_chamf]);
        }
    }

    // On X walls (short sides)
    for (sy = [wall + cav_y * 0.35, wall + cav_y * 0.65]) {
        // Near X wall
        translate([wall, sy - bump_w/2, bump_z])
        hull() {
            cube([snap_bump, bump_w, snap_h - snap_chamf]);
            translate([0, 0, snap_h - snap_chamf])
                cube([0.1, bump_w, snap_chamf]);
        }
        // Far X wall
        translate([wall + cav_x - snap_bump, sy - bump_w/2, bump_z])
        hull() {
            cube([snap_bump, bump_w, snap_h - snap_chamf]);
            translate([snap_bump - 0.1, 0, snap_h - snap_chamf])
                cube([0.1, bump_w, snap_chamf]);
        }
    }
}

// =============================================
// LID
// =============================================
module lid() {
    skirt_x = cav_x - 2 * lid_gap;
    skirt_y = cav_y - 2 * lid_gap;
    bump_w  = 6;

    difference() {
        union() {
            // Top plate (chamfered top edge)
            cham_rrect_top(ext_x, ext_y, ceil_h, corner_r, chamfer);

            // Skirt (nests inside base cavity)
            translate([wall + lid_gap, wall + lid_gap, -skirt_d])
            difference() {
                cube([skirt_x, skirt_y, skirt_d]);
                translate([sk_wall, sk_wall, -0.01])
                    cube([skirt_x - 2*sk_wall, skirt_y - 2*sk_wall,
                          skirt_d + 0.02]);
            }
        }

        // Snap grooves — channels on skirt outer surface
        // On Y walls (long sides)
        for (sx = [wall + cav_x * 0.3, wall + cav_x * 0.7]) {
            // Near Y side
            translate([sx - bump_w/2 - 0.3,
                       wall + lid_gap - 0.1,
                       -skirt_d + 0.3])
                cube([bump_w + 0.6, snap_bump + 0.3, snap_h + 0.2]);
            // Far Y side
            translate([sx - bump_w/2 - 0.3,
                       wall + lid_gap + skirt_y - snap_bump - 0.2,
                       -skirt_d + 0.3])
                cube([bump_w + 0.6, snap_bump + 0.3, snap_h + 0.2]);
        }

        // On X walls (short sides)
        for (sy = [wall + cav_y * 0.35, wall + cav_y * 0.65]) {
            // Near X side
            translate([wall + lid_gap - 0.1,
                       sy - bump_w/2 - 0.3,
                       -skirt_d + 0.3])
                cube([snap_bump + 0.3, bump_w + 0.6, snap_h + 0.2]);
            // Far X side
            translate([wall + lid_gap + skirt_x - snap_bump - 0.2,
                       sy - bump_w/2 - 0.3,
                       -skirt_d + 0.3])
                cube([snap_bump + 0.3, bump_w + 0.6, snap_h + 0.2]);
        }

        // Engraved "Rydful" text centered on lid top
        translate([ext_x/2, ext_y/2, ceil_h - engrave_depth])
            linear_extrude(engrave_depth + 0.01)
                resize([engrave_w, 0], auto=true)
                    text("Rydful", halign="center", valign="center",
                         font="Liberation Sans:style=Bold");
    }
}

// =============================================
// RENDER — comment out for export
// =============================================
// base();
// rotate([180,0,0]) lid();

base();
// rotate([180,0,0]) lid();

// Phantom batteries
%for (i = [0, 1]) {
    by = wall + batt_cl + aa_dia/2 + i * (aa_dia + batt_gap);
    translate([wall + spring_cl, by, floor_h + cradle_depth])
        rotate([0, 90, 0])
            cylinder(d = aa_dia, h = aa_len, $fn = 30);
}

// Phantom PCB
%translate([pcb_ox, pcb_oy, floor_h + standoff_h])
    cube([pcb_l, pcb_w, pcb_thk]);
