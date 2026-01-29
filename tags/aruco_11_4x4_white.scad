union()
{
  translate([-15.0, 15.0, 0.35]) cube([9.99, 9.99, 0.7], center=true);
  translate([-15.0, 5.0, 0.35]) cube([9.99, 9.99, 0.7], center=true);
  translate([15.0, -5.0, 0.35]) cube([9.99, 9.99, 0.7], center=true);
  translate([-5.0, -5.0, 0.35]) cube([9.99, 9.99, 0.7], center=true);
  translate([5.0, -15.0, 0.35]) cube([9.99, 9.99, 0.7], center=true);
  translate([-5.0, -15.0, 0.35]) cube([9.99, 9.99, 0.7], center=true);
  translate([-15.0, -15.0, 0.35]) cube([9.99, 9.99, 0.7], center=true);
}
