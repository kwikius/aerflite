use <axes.scad>

module arrow_axes(){
   RHR_axes(){arrow();arrow(); arrow();}
}

module ic(){
   cube([20,20,5]);
   translate([4,4,5]){
      sphere( d = 3);
   }
}


ic();
translate([10,10,10]){
arrow_axes();
}

