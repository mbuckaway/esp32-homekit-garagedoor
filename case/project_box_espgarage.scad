$fn=50;
//Procedural Project Box Screws

box_enable = 1;
top_enable = 1;
fillets_enable = 1;
fillets2_enable = 1;

text_enable_box = 1;
//40mm x 60 board size

inside_width = 60;
inside_length = 130;
inside_height = 35;
//Wall thickness
thickness = 2.5;                  
//Fillet radius. This should not be larger than thickness.
radius = 2;                     
//Diameter of the holes that screws thread into. 
screw_dia = 2.5;                  
//Diameter of the holes on the lid (should be larger than the diameter of your screws)
screw_loose_dia = 3.2;
//Only use this if the lip on your lid is detached from the lid! This is a hack to work around odd union() behaviour.
extra_lid_thickness = 0;        //Extra lid thickness above thickness. 
                                //You may want to tweak this to account for large chamfer radius.

// The lid can have a lip on it for attached to another surface
// This is the size of that lip
lid_screw_lip_size = 15;
lid_screw_lip_screw_size = 5;
// The side of the box has a hole for the wifi antenna.
// Set to zero to disable.
antenna_holesize=0;
antenna_offset=7;

led_holesize=9.8;
led_offset=25;
led_spacing=20;
led_count = 2;

top_holesize=15;
top_holeoffset = 5;

bottom_holesize=10;

screwmount_height = 12;
screwmount_length = 65;
screwmount_width = 40;
screwmount_offset_x = 12;
screwmount_offset_y = 13;
screwmount_screw_dia = 1.25;

screwmount2_height = 8;
screwmount2_length = 21;
screwmount2_width = 29;
screwmount2_offset_x = 18;
screwmount2_offset_y = 95;


outside_width = inside_width + thickness * 2;
outside_length = inside_length + thickness * 2;
od = screw_dia * 2.5;

module screwmount()
{
    difference() {
        cylinder(r=3.0, h=screwmount_height-2);
        translate([0,0,-1]) cylinder(r=screwmount_screw_dia, h=screwmount_height);
    }
    rotate_extrude(convexity = 10)
        translate([2.5,0,0]) {
            intersection()
            {
                square(5);
                difference() {
                    square(5, center=true);
                    translate([2.5,2,5]) circle(2.0);
                }
            }
        }
}

module filletposts()
{
    postoffset_length = screwmount_length;
    postoffset_width = screwmount_width;
    post1 = [0, 0, 0];
    post2 = [0, postoffset_length, 0];
    post3 = [postoffset_width, 0, 0];
    post4 = [postoffset_width, postoffset_length, 0];
    translate(post1) screwmount();
    translate(post2) screwmount();
    translate(post3) screwmount();
    translate(post4) screwmount();
}

module filletposts2()
{
    postoffset_length = screwmount2_length;
    postoffset_width = screwmount2_width;
    post1 = [0, 0, 0];
    post2 = [0, postoffset_length, 0];
    post3 = [postoffset_width, 0, 0];
    post4 = [postoffset_width, postoffset_length, 0];
    translate(post1) screwmount();
    translate(post2) screwmount();
    translate(post3) screwmount();
    translate(post4) screwmount();
}

module box_screw(id, od, height){
    difference(){
        union(){
            cylinder(d=od, h=height, $fs=0.2);
            translate([-od/2, -od/2, 0])
                cube([od/2,od,height], false);
            translate([-od/2, -od/2, 0])
                cube([od,od/2,height], false);
        }
        cylinder(d=id, h=height, $fn=6);
    }
}

module rounded_box(x,y,z,r){
    translate([r,r,r])
    minkowski(){
        cube([x-r*2,y-r*2,z-r*2]);
        sphere(r=r, $fs=0.1);
    }
}

module main_box(){
    difference(){
        //cube([outside_width, outside_length, inside_height + thickness * 2]);
        difference(){
            rounded_box(outside_width, outside_length, inside_height + thickness + 2, radius);
            translate([0,0,inside_height + thickness])
            cube([outside_width, outside_length, inside_height + thickness * 2]);
        }
        translate([thickness, thickness, thickness])
        cube([inside_width, inside_length, inside_height + thickness]);
        if (antenna_holesize>0)
        {
            translate([thickness/2,inside_length/2,inside_height-antenna_offset]) {
                rotate([0,90,0])
                cylinder(h = thickness*2, d = antenna_holesize, center=true, $fs=0.2);    
            }
        }
        if (led_holesize>0)
        {
            translate([inside_width/2+thickness,led_offset,thickness/2])
                cylinder(h = thickness*2, d = led_holesize+.15, center=true, $fs=0.2);    
            translate([inside_width/2+thickness,led_offset+led_spacing,thickness/2])
                cylinder(h = thickness*2, d = led_holesize, center=true, $fs=0.2);
            if (led_count>2)
            {    
            translate([inside_width/2+thickness,led_offset+led_spacing*2,thickness/2])
                cylinder(h = thickness*2, d = led_holesize, center=true, $fs=0.2);    
            }
            if (led_count>3)
            {
                translate([inside_width/2+thickness,led_offset+led_spacing*3,thickness/2])
                cylinder(h = thickness*2, d = led_holesize, center=true, $fs=0.2);
            }
        }
        if (top_holesize>0)
        {
            translate([inside_width/2+thickness,thickness/2,inside_height/2+thickness+top_holeoffset]) {
                rotate([90,0,0])
                cylinder(h = thickness*2, d = top_holesize, center=true, $fs=0.2);    
            }
        }
        if (bottom_holesize>0)
        {
            translate([inside_width/2+thickness,inside_length+thickness+thickness/2,inside_height/2+thickness]) {
                rotate([90,0,0])
                cylinder(h = thickness*2, d = bottom_holesize, center=true, $fs=0.2);    
            }
        }
        if (text_enable_box)
        {
            translate ([2,inside_length-25,thickness/3]) 
                rotate([180,0,0])
                linear_extrude(thickness/3)
                text("Garage Door", size=8);
            translate ([14,inside_length-10,thickness/3]) 
                rotate([180,0,0])
                linear_extrude(thickness/3)
                text("Opener", size=8);
        }
    }

    od = screw_dia * 2.5;
    
    translate([od/2+thickness,od/2+thickness, 0])
        box_screw(screw_dia, od, inside_height);
    
    translate([thickness+inside_width-od/2, od/2+thickness, 0])
        rotate([0,0,90])
            box_screw(screw_dia, od, inside_height);
    
    translate([thickness+inside_width-od/2, -od/2+thickness+inside_length, 0])
        rotate([0,0,180])
            box_screw(screw_dia, od, inside_height);
    
    translate([od/2 + thickness, -od/2+thickness+inside_length, 0])
        rotate([0,0,270])
            box_screw(screw_dia, od, inside_height);
}

module lid(){
    difference(){
        union(){
        //Lid.
        difference(){
            translate([0,-lid_screw_lip_size,0])
                rounded_box(outside_width, outside_length+lid_screw_lip_size*2, thickness * 4, radius);
            translate([0,-lid_screw_lip_size, thickness + extra_lid_thickness])
                cube([outside_width, outside_length+lid_screw_lip_size*2, inside_height + thickness * 4]);
        }
        //Lip
        lip_tol = 0.5;
        lip_width = inside_width - lip_tol;
        lip_length = inside_length - lip_tol;
        translate([(outside_width - lip_width)/2,(outside_length - lip_length)/2, thickness * 0.99])
            difference(){
                cube([lip_width, lip_length, thickness]);
                translate([thickness, thickness, 0])
                    cube([lip_width-thickness*2, lip_length-thickness*2, thickness]);
        }
        
        intersection(){
            union(){
            translate([od/2 + thickness, od/2 + thickness, thickness])
                box_screw(screw_dia, od, thickness);
            translate([inside_width - od/2 + thickness, od/2 + thickness, thickness])
                rotate([0,0,90])
                    box_screw(screw_dia, od, thickness);
            translate([inside_width - od/2 + thickness, inside_length - od/2 + thickness, thickness])
                rotate([0,0,180])
                    box_screw(screw_dia, od, thickness);
            translate([od/2 + thickness, inside_length - od/2 + thickness, thickness])
                rotate([0,0,270])
                    box_screw(screw_dia, od, thickness);
            }
            translate([thickness + lip_tol, thickness + lip_tol, 0])
            cube([lip_width-lip_tol,lip_length-lip_tol, 200]);
        }

        }
        
        union(){
            translate([od/2 + thickness, od/2 + thickness, thickness])
                cylinder(h = thickness * 4, d = screw_loose_dia, center=true, $fs=0.2);
            translate([inside_width - od/2 + thickness, od/2 + thickness, thickness])
                cylinder(h = thickness * 4, d = screw_loose_dia, center=true, $fs=0.2);
            translate([inside_width - od/2 + thickness, inside_length - od/2 + thickness, thickness])
                cylinder(h = thickness * 4, d = screw_loose_dia, center=true, $fs=0.2);
            translate([od/2 + thickness, inside_length - od/2 + thickness, thickness])
                cylinder(h = thickness * 4, d = screw_loose_dia, center=true, $fs=0.2);
        }
        // Draw the lip screw holes
        if (lid_screw_lip_size>0)
        {
            union() {
                translate([10, lid_screw_lip_size-22, 0])
                    cylinder(h = thickness * 4, d = lid_screw_lip_screw_size, center=true, $fs=0.2);            
                translate([inside_width-od/2, lid_screw_lip_size-22, 0])
                    cylinder(h = thickness * 4, d = lid_screw_lip_screw_size, center=true, $fs=0.2);            
                translate([inside_width-od/2, inside_length+13, 0])
                    cylinder(h = thickness * 4, d = lid_screw_lip_screw_size, center=true, $fs=0.2);            
                translate([10, inside_length+13, 0])
                    cylinder(h = thickness * 4, d = lid_screw_lip_screw_size, center=true, $fs=0.2);            
            }
        }

    }
}

if (box_enable) main_box();
if (top_enable) translate([-outside_width-5,0,0]) lid();
if (fillets_enable) translate([screwmount_offset_x,screwmount_offset_y, thickness]) filletposts();
if (fillets2_enable) translate([screwmount2_offset_x,screwmount2_offset_y, thickness]) filletposts2();
