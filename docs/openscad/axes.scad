
module arrow(){
   body_h = 10;
   d = 1;
   arrow_base = 2;
   arrow_head_length = 4;
   
   translate([0,0,body_h]){
   cylinder ( d1 = arrow_base, d2 = 0.01, h =arrow_head_length, $fn = 20);
   }
   cylinder( d= 1, h = body_h,$fn = 20);
}

module rotation_arrow_rhs(){
rotate_extrude(angle = 120){
   translate([4,0,0]){
      circle ( d= 1, $fn = 20);
   }
}
rotate([0,0,120]){ 

rotate([90,0,0]){
      translate([4,0,-2]){
      cylinder ( d1 = 0.01, d2 = 1.75, h =2.5, $fn = 20);
   }
}
}
}
module rotation_arrow_lhs(){
   mirror([0,1,0]){
      rotation_arrow_rhs();
   }
}




module arrow_n_rotation_arrow_rhs(){
   arrow();
translate([0,0,9]){
rotation_arrow_rhs();
}
}

module arrow_n_rotation_arrow_lhs(){
   arrow();
translate([0,0,9]){
rotation_arrow_lhs();
}
}

//arrow();
module RHR_axes(){
   color("red"){
      translate([15,-2,-2]){
         rotate([0,-90,0]){
            text("X", size = 4);
         }
      }
      rotate([0,90,0]){
         children(0);
      }
   }  
   color("green"){
      translate([-2,15,-2]){
         rotate([90,0,0]){
            text("Y", size = 4);
         }
      }
      rotate([-90,0,0]){
       children(1);
      }
   }   
   
   color("blue"){
   translate([-2,-2,15]){
       rotate([0,0,0]){
          text("Z", size = 4);
       }
   }
   rotate([0,0,0]){
      children(2);
      }
   }   
   
}


RHR_axes(){
   arrow_n_rotation_arrow_rhs(); 
   arrow_n_rotation_arrow_rhs();
   arrow_n_rotation_arrow_rhs();
}




//RHR_axes();