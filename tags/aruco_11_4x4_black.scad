difference()
{
  translate([0, 0, 0.35]) cube([60, 60, 0.7], center=true);
  translate([-15.0, 15.0, 0]) cube([10.01, 10.01, 2.0999999999999996], center=true);
  translate([-15.0, 5.0, 0]) cube([10.01, 10.01, 2.0999999999999996], center=true);
  translate([15.0, -5.0, 0]) cube([10.01, 10.01, 2.0999999999999996], center=true);
  translate([-5.0, -5.0, 0]) cube([10.01, 10.01, 2.0999999999999996], center=true);
  translate([5.0, -15.0, 0]) cube([10.01, 10.01, 2.0999999999999996], center=true);
  translate([-5.0, -15.0, 0]) cube([10.01, 10.01, 2.0999999999999996], center=true);
  translate([-15.0, -15.0, 0]) cube([10.01, 10.01, 2.0999999999999996], center=true);
}
