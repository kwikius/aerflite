
use <axes.scad>

module arrow_axes(){
   RHR_axes(){arrow();arrow(); arrow();}
}
module BMI160_arrow_n_rotation_axes(){
   RHR_axes(){
   arrow_n_rotation_arrow_rhs(); 
   arrow_n_rotation_arrow_rhs();
   arrow_n_rotation_arrow_rhs();
}
}

module board_arrow_n_rotation_axes(){
   rotate([180,0,0]){
   RHR_axes(){
   arrow_n_rotation_arrow_rhs(); 
   arrow_n_rotation_arrow_rhs();
   arrow_n_rotation_arrow_rhs();
}
}
}

translate([70,0,5]){
   translate([3,3,0]){
      text("board_orient",size = 2);
   }
   rotate([0,0,0]){
      board_arrow_n_rotation_axes();
   }
}


translate([0,50,5]){
   translate([3,3,0]){
      text("compass_orient",size = 2);
   }
   rotate([0,0,270]){
     arrow_axes();
   }
}


translate([-70,0,5]){
   translate([3,3,0]){
      text("BMI160_orient",size = 2);
   }
   rotate([0,0,90]){
      BMI160_arrow_n_rotation_axes();
   }
}

text_size = 2;
show_raspberry_pi = false;
module JST_SH1_horz(n_pins,name_text)
{
  translate([-4,-(n_pins + 2)/2,0]){
  	  cube ([4,n_pins + 2,3]);
  }
  translate([-2,-(n_pins + 3)/2,0]){
	  cube ([2,n_pins + 3,0.5]);
  }
  for ( i= [-(n_pins-1)/2:1: (n_pins-1)/2]){
	 translate([-5.5, i -0.25,0]){
		cube([1.5,0.5,0.25]);
	 }
  }
  translate([1,0,1]){
      text(name_text,size = text_size);
  }
}

module raspberry_pi_zero()
{
     difference(){
	 cube([65,30,5], center = true);
      translate([27,-1,-0]){
        rotate([0,180,0]){
        linear_extrude(height = 3) {
           text("Raspberry Pi Zero", size =  5);
        }
     }
      }
  }
}

module JST_GH_horz( n_pins,name_text)
{
  for ( i= [-(n_pins-1)/2:1: (n_pins-1)/2]){
	 translate([-5.4, i *1.25 -0.3,0]){
		cube([1.4,0.6,0.1]);
	 }
  }
  for ( i = [-1:2:1]){
		translate([-2.7,((n_pins-1)/2*1.25 +1.85)*i -0.5,0]){
			cube([2.7,1,0.1]);
		}
  }
  width = (n_pins-1) + 4.5;
  thickness = 0.5;
  difference(){
  translate([-4.05,-width/2,0]){
	cube([4.05,width,4.25]);
  }

  translate([-4.05 + thickness,-(width/2)+thickness,thickness]){
	cube([5,width - 2*thickness,4.25 - 2*thickness]);
  }
  }
   translate([1,0,1]){
      text(name_text,size = text_size);
  }
  
}

module JST_GH_vert( n_pins,name_text)
{
  for ( i= [-(n_pins-1)/2:1: (n_pins-1)/2]){
	 translate([-5.6, i *1.25 -0.3,0]){
		cube([1.7,0.6,0.1]);
	 }
  }
  for ( i = [-1:2:1]){
		translate([-2.8,((n_pins-1)/2*1.25 +1.85)*i -0.5,0]){
			cube([2.8,1,0.1]);
		}
  }
  width = (n_pins-1) + 4.5;
  thickness = 0.5;
  difference(){
  translate([-4.25,-width/2,0]){
	cube([4.25,width,4.05]);
  }

  translate([-4.25 + thickness,-(width/2)+thickness,thickness]){
	cube([4.25 - 2 * thickness,width - 2*thickness,5]);
  }
  }
  translate([1,0,1]){
      text(name_text,size = text_size);
  }
  
}


module hdr_pin()
{
	translate([-3,-0.32, -0.32]){
		cube([11.5,0.64,0.64]);
	}
   translate([0,-1.27,-1.27]){
		cube([2.54,2.54,2.54]);
	}
}
module rc_in_hdr()
{
	for (col = [-2.5:1:2.5]){
		for (row = [-1:1:1]){
         translate([0,col * 2.54,row * 2.54 +1.27]){
				hdr_pin();
		   }
		}
	}
}
module sd_socket()
{
   x = 12;
   y = 11.45;
   z = 1.6;
   
   translate([-x,-y/2,0]){
		cube([x,y,z]);
	}
    
   translate([1,0,1]){
      text("uSD Card",size = text_size);
  }
}


pcb_size = [77,35,1.5];
hole_spacing = [58, 23];
hole_dia = 3;

module pcb()
{
   difference(){
		translate(-pcb_size/2){
	     cube(pcb_size);
		}
		for (x_iter = [-1:2:1]){
			for (y_iter = [-1:2:1]){
	         translate([x_iter*hole_spacing[0]/2,
						y_iter*hole_spacing[1]/2,-(pcb_size[2]+1)/2]){
	             cylinder(d = hole_dia, h = pcb_size[2]+1, $fn = 20);
				}
			}
		}
   }
}

module mcu()
{
   color([0.3,0.3,0.3]){
		translate([-5,-5,0]){
		     cube([10,10,1]);
	   }
   }
    color([0.7,0.7,0.7]){
   translate([-7,-7,0]){
       cube([14,14,0.2]);
	}
   }
}

module aerflite_fc() {
 // headers
  rotate([0,0,180]){
 	 translate([pcb_size[0]/2,0,0]){
			rc_in_hdr();
	  }
  }

  translate([-7,0,-pcb_size[2]/2]){
    rotate([0,180,0]){
      mcu();
    }
  }

  color([0.5,0.1,0.5]){
    pcb();
  }
if (show_raspberry_pi){
  translate([0,0,-5]){
     raspberry_pi_zero();
  }
}
  
  translate([pcb_size[0]/2 -1,0.5,pcb_size[2]/2]){
	 sd_socket();
  }
//-----------------
// rear side with RC out
   // stbd top
  //P1 CAN
  translate([-pcb_size[0]/2,-12,pcb_size[2]/2]){
   rotate([0,0,180]){
   JST_GH_vert(4,"CAN");
	}
  }
   // stbd bottom 
  //P2 CAN
   translate([-pcb_size[0]/2,-12,-pcb_size[2]/2]){
   rotate([180,0,180]){
   JST_GH_horz(4,"CAN");
	}
  }
  //P7  Usart3
  translate([-pcb_size[0]/2,12,pcb_size[2]/2]){
   rotate([0,0,180]){
   JST_GH_vert(4,"UART3(GPS)");
	}
  }
  //P8 Usart4
  translate([-pcb_size[0]/2,12,-pcb_size[2]/2]){
   rotate([180,0,180]){
   JST_GH_horz(4,"UART_(unused)");
	}
  }
//----------------------------------
//  front side with uSD_Card
  //top left
  //P4 power horz
  translate([pcb_size[0]/2,12.5,pcb_size[2]/2]){
   JST_GH_horz(2,"VIN 8V~16V");
  }
//bottom left
  //P5 power horz
  translate([pcb_size[0]/2,12.5,-pcb_size[2]/2]){
   rotate([180,0,0]){
   JST_GH_horz(2,"VIN 8V~16V");
	}
  }
//top right
 //P3 video out
  translate([pcb_size[0]/2,-11.75,pcb_size[2]/2]){
   JST_GH_horz(4,"VIDEO OUT");
  }
//bottom right
  // P22 camera Video in
   translate([pcb_size[0]/2,-12.5,-pcb_size[2]/2]){
   rotate([180,0,0]){
   JST_GH_horz(3,"VIDEO IN");
	}
  }
//------------------------------
// port side front to back
// P19 5V out
  translate([22.5,pcb_size[1]/2,pcb_size[2]/2]){
	 rotate([0,0,90]){
    	JST_GH_vert(3,"5V OUT 1A");
	 }
  }

  //P20 analog_in
  translate([13,pcb_size[1]/2,pcb_size[2]/2]){
	 rotate([0,0,90]){
    	JST_GH_vert(3,"ANALOG IN");
	 }
  }

  // prog
  translate([-4,pcb_size[1]/2,pcb_size[2]/2]){
	 rotate([0,0,90]){
    	JST_SH1_horz(2,"PROG");
	 }
  }
   // jtag
  translate([4,pcb_size[1]/2,pcb_size[2]/2]){
	 rotate([0,0,90]){
    	JST_SH1_horz(4,"JTAG");
	 }
  }

  //P23 Batt current
  translate([-12,pcb_size[1]/2,pcb_size[2]/2]){
	 rotate([0,0,90]){
    	JST_GH_vert(3,"BATTI");
	 }
  }
 //P9 usart6
 translate([-21,pcb_size[1]/2,pcb_size[2]/2]){
	 rotate([0,0,90]){
    	JST_GH_vert(4,"UART6(tracker)");
	 }
  }

//----------------------
//stbd side front to back
//P24 Aux video in
   translate([22,-pcb_size[1]/2,pcb_size[2]/2]){
	 rotate([0,0,-90]){
    	JST_GH_vert(3,"AUX VIDEO IN");
	 }
  }
  //P11 RC In
  translate([12,-pcb_size[1]/2,pcb_size[2]/2]){
	 rotate([0,0,-90]){
    	JST_GH_vert(5,"RC IN");
	 }
  }

   //P21 Airspeed
   translate([0.5,-pcb_size[1]/2,pcb_size[2]/2]){
	 rotate([0,0,-90]){
    	JST_GH_vert(3,"AIRSPEED IN");
	 }
  }

  //P6 I2C
  translate([-10.5,-pcb_size[1]/2,pcb_size[2]/2]){
	 rotate([0,0,-90]){
    	JST_GH_vert(5,"I2C");
	 }
  }

  //P10 usart6
   translate([-21,-pcb_size[1]/2,pcb_size[2]/2]){
	 rotate([0,0,-90]){
    	JST_GH_vert(4,"UART1(console)");
	 }
  }
  
  translate ([100,0,20]){
     // text ("Hello world");
  }
}

aerflite_fc();

  