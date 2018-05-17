
// scale( [0.001, 0.001, 0.001] )
//  barhocker();
//eps = 1;
//fn = 40;

module barhocker( foot_diameter=500, foot_height=50,
                  foot_rest_diameter = 400, foot_rest_height = 300, foot_rest_thickness = 20,
                  pillar_diameter=70, pillar_height=800,
                  seat_diameter=400, seat_height=130 ) 
{
  color( [0.9, 0.9, 0.9] ) 
    cylinder( d=foot_diameter, h=foot_height, center=false, $fn=fn );
  color( [0.9, 0.9, 0.9] ) 
    translate( [0,0,foot_height-eps] )
      cylinder( d=pillar_diameter, h=pillar_height, center=false, $fn=fn );
  color( [0.9, 0.9, 0.9] ) 
    difference() {
      translate( [0,0,foot_rest_height] )
        cylinder( d=foot_rest_diameter, h=foot_rest_thickness, center=false, $fn=fn );
      translate( [0,0,foot_rest_height-eps] )
        cylinder( d=foot_rest_diameter-2*foot_rest_thickness, 
                  h=foot_rest_thickness+4*eps, center=false, $fn=fn );
      }
  color( [0.6, 0.2, 0.2] ) 
    translate( [0,0,foot_height+pillar_height-eps] )
      cylinder( d=seat_diameter, h=seat_height, center=false, $fn=fn );
}