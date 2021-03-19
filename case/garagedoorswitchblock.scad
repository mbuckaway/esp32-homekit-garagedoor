$fn = 20;


width = 50;
length = 100;
height = 40;
top_height = 5;

module rounded_box(x,y,z,r){
    translate([r,r,r])
    minkowski(){
        cube([x-r*2,y-r*2,z-r*2]);
        sphere(r=r, $fs=0.1);
    }
}

difference()
{
    rounded_box(length, width, height, 3);
    cube([length/4, width, height-top_height]);
    translate([length-70, 0, height]) rotate([0, 30, 0]) cube([length, width, height+10]);
    translate([length-90, 0, 0]) rotate([0, 30, 0]) cube([length/2+10, width, height-10]);
    translate([15, width/3, 25]) cylinder(h=20, d=5);
    translate([15, width/3*2, 25]) cylinder(h=20, d=5);
    translate([15, width/3, 46]) rotate([180, 0, 0])cylinder(10, 10);
    translate([15, width/3*2, 46]) rotate([180, 0, 0])cylinder(10, 10);
}
