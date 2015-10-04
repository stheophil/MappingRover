nut_height = 6;
nut_radius = 5.95; // 6 was slightly too big
edge_length = tan(30) * nut_radius;

hole_radius = 2.3;
hole_cutoff = 1.3; // 1.5 fits on one axis, too tight on others. 

difference() {
	union() {
		linear_extrude( height=nut_height ) {	
			for(i = [0:6]) {
			 	rotate([0,0, i*60]) polygon([[0,0], [nut_radius, -edge_length], [nut_radius, edge_length]]);
			}
		}
		cylinder(h=nut_height*1.5, r=nut_radius*0.8, $fn=100);
	}
	difference() {
		translate([0,0,-1]) cylinder(h=nut_height*2, r=hole_radius, $fn=100);
		translate([-hole_radius, hole_radius-hole_cutoff, 0]) cube([2*hole_radius, 2*hole_radius, nut_height*2]);
	}
}