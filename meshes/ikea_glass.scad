/** IKEA glass, dimensions in millimeters */

eps = 0.1;


// ROS wants meters
scale( [0.001, 0.001, 0.001] )  ikea_glass();


module ikea_glass() {
  difference() {
    cylinder( d1=56, d2=86, h=120, $fn=50, center=false ); 
    translate( [0,0,6] ) // bottom thickness
      cylinder( d1=56, d2=81, h=(120-6+eps), $fn=50, center=false );
  }
}