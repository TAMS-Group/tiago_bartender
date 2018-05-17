/** easy_grasp_bottle.scad - 3D model of our "bartender robot" bottles.
 * Includes extra "grasping collar" for use in Gazebo.
 *
 * Note: units in millimeters, scale for ROS.
 * Note: total height with cap 287 mm, without 285 mm,
 * cap diameter 30 mm, screw diameter 28,
 * collar diameter 40 mm, collar thickness 2 mm
 * 
 * (c) 2018, fnh, hendrich@informatik.uni-hamburg.de
 */
 
 
fn  = 50; // use 10 for raspi 3...
eps = 0.1;
 
// scale and change origin for use with ROS:
// scale( [0.001, 0.001, 0.001] ) 
  //translate( [0,0,-284] ) 
    bottle();
 
 
 
 /**
  * The bottle consists of four parts:
  * - the "pouring opening" with the cap screw (but without threads!)
  * - a cone
  * - the main part
  * - a rounded bottom
  * Origin is at the bottom center.
  */
 module bottle( 
   main_diameter = 80.0,
   screw_diameter = 28.0,
   screw_height = 19.0,
   collar_diameter = 40.0,
   collar_height = 4.0,
   cone1_diameter = 70.0,
   cone1_height = 30.0,
   cone2_height = 53.0,
   main_height = 158.0,
   bottom_diameter = 60.0,
   bottom_height = 20.0,
   grasp_collar_diameter = 100,
   grasp_collar_height = 10,
   ) 
 {
   // bottom part
   // cylinder( d1=bottom_diameter, d2=main_diameter, h=bottom_height+eps, $fn=fn, center=false );   
   hh = bottom_height;
   ww = main_diameter - bottom_diameter;
   translate( [0,0,hh] ) 
   minkowski() { 
     cylinder( d=bottom_diameter, h=1, $fn=fn, center=false ); 
     scale( [ww,ww,2*hh] ) sphere( d=1, $fn=fn );
   }
   
   // main part
   translate( [0,0,bottom_height] )
     cylinder( d=main_diameter, h=main_height+eps, $fn=fn, center=false );

   // extra grasping collor (for Gazebo+robot gripper)
   translate( [0,0,bottom_height+main_height-grasp_collar_height] )
     cylinder( d=grasp_collar_diameter, h=grasp_collar_height, $fn=fn, center=false );
   
   // cone part
//   translate( [0,0,bottom_height+main_height] )
//     cylinder( d1=main_diameter, d2=cone1_diameter, h=cone1_height+eps, $fn=fn, center=false );
   translate( [0,0,bottom_height+main_height+cone1_height] )
     cylinder( d1=cone1_diameter, d2=screw_diameter, h=cone2_height+eps, $fn=fn, center=false );

   // hhh = 2.23;
   hhh = 1.6;
   translate( [0,0,bottom_height+main_height-eps] )
     scale( [1,1,hhh] ) sphere( d=main_diameter, $fn=fn );   
   
   
   // original grasping collar (at the cap)
   cone_height = cone1_height + cone2_height;
   translate( [0,0,bottom_height+main_height+cone_height] )
     cylinder( d=collar_diameter, h=collar_height, $fn=fn, center=false );
   
   // simplified screw/cap part
   translate( [0,0,bottom_height+main_height+cone_height+collar_height] )
     cylinder( d=screw_diameter, h=screw_height, $fn=fn, center=false );
     
   echo( "total height: " );
   echo( bottom_height+main_height+cone_height+collar_height+screw_height );
}
