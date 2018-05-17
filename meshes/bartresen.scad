fn = 50;
eps = 0.1;

include <barhocker.scad>;



// scale( [0.001, 0.001, 0.001] ) 
//   bartresen();


translate( [-2.0,0,0] )
  // scale( [0.001, 0.001, 0.001] ) 
    cupboard();

/*

//translate( [-1.0, -4.0,,0] )
//  scale( [0.001, 0.001, 0.001] ) side_barrier();
translate( [-1.0, +2.5,,0] )
  scale( [0.001, 0.001, 0.001] ) side_barrier();

translate( [1.0, -1.0, 0.0] )
  scale( [0.001, 0.001, 0.001] ) barhocker();
translate( [1.0, 0.0, 0.0] )
  scale( [0.001, 0.001, 0.001] ) barhocker();
translate( [1.0, 1.0, 0.0] )
  scale( [0.001, 0.001, 0.001] ) barhocker();
*/



module bartresen(
  tresen_length = 3000,
  tresen_width  =  800,
  tresen_height = 1200,
  base_height = 50,        // height of bottom/footer part of the Tresen
  counter_height = 100,    // height of upper table-plate of the Tresen
  inset_height = 500, // clear height of one cupboard on robot side
)
{
  dx = 300; 
  difference() {
    union() {
      color( [0.6, 0.4, 0.2] ) 
        translate( [0,0,tresen_height/2] ) 
          cube( size=[tresen_width, tresen_length, tresen_height], center=true );
      color( [0.6, 0.4, 0.2] ) 
        translate( [0,tresen_length/2,tresen_height/2] ) 
          cylinder( d=tresen_width, h=tresen_height-eps, center=true, $fn = fn );
      color( [0.6, 0.4, 0.2] ) 
        translate( [0,-tresen_length/2,tresen_height/2] ) 
          cylinder( d=tresen_width, h=tresen_height-eps, center=true, $fn = fn );
    }
    translate( [tresen_width/2-dx/2+eps,0,tresen_height/2-base_height/2] )
      cube( size=[dx+10*eps, tresen_length+tresen_width+eps, tresen_height-counter_height-base_height], center=true );

    translate( [-tresen_width/2+dx/2-eps,0,base_height+inset_height/2] )
      cube( size=[dx, tresen_length+tresen_width+eps, inset_height], center=true );
    translate( [-tresen_width/2+dx/2-eps,0,base_height+inset_height+base_height+inset_height/2] )
      cube( size=[dx, tresen_length+tresen_width+eps, inset_height], center=true );
     
  }
}


module cupboard(
  cupboard_length= 4000,
  cupboard_width = 500,
  cupboard_height= 2500,
  base_height    = 50,
  inset_height   = 500,
  wall_thickness = 30,
)
{
  color( [0.6, 0.4, 0.2] ) 
    for( dx=[0:inset_height:cupboard_height] )
      translate( [0,0,base_height/2+dx] )
        cube( size=[cupboard_width,cupboard_length,base_height], center=true );
  // rear wall
  color( [0.6, 0.4, 0.2] ) 
    translate( [-cupboard_width/2+wall_thickness/2-eps,0,cupboard_height/2] )
      cube( size=[wall_thickness, cupboard_length, cupboard_height], center=true );  
  // side walls
  color( [0.6, 0.4, 0.2] ) 
    translate( [0,cupboard_length/2-wall_thickness/2+eps,cupboard_height/2] )
      cube( size=[cupboard_width, wall_thickness, cupboard_height], center=true );  
  color( [0.6, 0.4, 0.2] ) 
    translate( [0,-cupboard_length/2+wall_thickness/2-eps,cupboard_height/2] )
      cube( size=[cupboard_width, wall_thickness, cupboard_height], center=true );  
}


module side_barrier(     
  dx = 2000,
  dy = 200,
  dz = 800,
) 
{

  color( [0.8,0.8,0.8] ) 
    translate( [0,0,dz/2] )
      cube( size=[dx,dy,dz], center=true );

}

  
