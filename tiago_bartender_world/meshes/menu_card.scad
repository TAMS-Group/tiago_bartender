/* menu_card.scad 
 * 
 * just an A4-paper sized cube, so that we can make
 * a Collada and put a texture on it... Gazebo 7+
 * doesn't support textures on STLs anymore.
 *
 * Dimensions are in millimeters, scale 0.001 for ROS.
 * Origin is at the center.
 */
 
 
card_A4();


module card_A4() { menu_card(); }

module card_A6() { menu_card( w=10.5, h=14.9, d=0.1 ); }

 
module menu_card(
  w = 21.0,
  h = 29.7,
  d = 0.1
  )
{
  cube( size=[w,h,d], center=true );
}