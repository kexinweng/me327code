import processing.serial.*;

Serial myPort;        // The serial port

//initialize all variables
float inByte = 0; //current value of the first variable in the string
float lastByte = 0; //previous value of the first variable in the string
float inByte2 = 0; //current value of the second variable in the string
float lastByte2 = 0; //previous value of the second variable in the string

void setup () {
  // set the window size:
  size(600, 400);        

  // List all the available serial ports
//  println(Serial.list());
  // Check the listed serial ports in your machine
  // and use the correct index number in Serial.list()[].

  //note you may need to change port number, it is 9 for me
//  myPort = new Serial(this, Serial.list()[0], 38400);  // also make sure baud rate matches Arduino
  

  // A serialEvent() is generated when a newline character is received :
//  myPort.bufferUntil('\n');
  background(0);      // set inital background:
}
void draw () {
  // everything happens in the serialEvent()
  background(0); //uncomment if you want to control a ball
  stroke(255);     //stroke color
  strokeWeight(3);        //stroke wider
  
  //float wallmap1 = map(?,-0.0689, 0.06119,0,600);
  //float wallmap2 = map(?,-0.0689, 0.06119,0,600);
  //float wallmap3 = map(?,-0.0689, 0.06119,0,600);
  //float wallmap4 = map(?,-0.0689, 0.06119,0,600);
  //float wallmap5 = map(?,-0.0689, 0.06119,0,600);
  
  line(5, 5 , 595, 5);
  line(595, 5, 595, 300);
  line(5, 5, 5, 300);
  line(5, 300, 200, 300);
  line(400, 300,  595, 300);

  fill(255);
  circle(300, 285, 50);
  noFill();
  arc(300, 285, 200, 200, atan(0.15), PI - atan(0.15));
  noFill();
  
  fill(255);
  rect(75, 75, 185, 160, 0 , 0, 40, 0);
  fill(255);
  rect(340, 75, 185, 160, 0, 0, 0, 40);
  
  stroke(180, 110, 30);
  fill(180, 110, 30);
  rect(15, 200, 15, 25, 3, 3, 3, 3);
  //circle(inByte, 200, 20);
  //circle(inByte2, 200, 20);
  //START EDITING HERE
  //stroke(r,g,b);     //stroke color
  //strokeWeight(num);        //stroke wider
  
  stroke(255);
  strokeWeight(0.5);
  line(40, 40, 560, 40);
  line(40, 40, 40,270);
  line(40, 270,220, 270);
  line(380, 270, 560, 270);
  line(560, 270, 560, 40);
  line(300, 75, 300, 195);
  // Mass Spring Damper
  
  //draw the wall
  //draw a line from the wall to the xMass
  //draw an ellipse to represent the mass of the spring-mass-damper
  //draw an ellipse to represent the user
  

}
