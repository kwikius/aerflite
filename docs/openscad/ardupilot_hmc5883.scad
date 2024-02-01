use <axes.scad>



module arrow_axes(){
   RHR_axes(){arrow();arrow(); arrow();}
}

module ic(){
   translate([-10,-10,0]){
    
   cube([20,20,5]);

       
   translate([4,4,5]){
       color("black"){
      sphere( d = 3);
   }
         }
}
}


ic();
translate([0,0,25]){
rotate([0,180,0]){
arrow_axes();
}
}
translate([-10,15]){
text(" Ardupilot HMC5883");
}
