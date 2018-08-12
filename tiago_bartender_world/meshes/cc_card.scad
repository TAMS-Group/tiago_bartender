/* menu_card.scad 
 * 
 * just an A4-paper sized cube, so that we can make
 * a Collada and put a texture on it... Gazebo 7+
 * doesn't support textures on STLs anymore.
 *
 * Dimensions are in millimeters, scale 0.001 for ROS.
 * Origin is at the center.
 */
 
 
command_card();



module command_card() { card( w=90, h=130, d=0.5 ); }


module card_A4() { card(); }

module card_A6() { card( w=105, h=149, d=0.5 ); }

 
module card(
  w = 210,
  h = 297,
  d = 0.5
  )
{
  cube( size=[w,h,d], center=true );
}