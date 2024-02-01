// NED_axes

use <axes.scad>


module NED_axes(){
   rotate([180,0,0]){
      RHR_axes(){
         arrow_n_rotation_arrow_rhs(); 
         arrow_n_rotation_arrow_rhs();
         arrow_n_rotation_arrow_rhs();
      }
   }
}

NED_axes();