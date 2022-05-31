import processing.serial.*;

Serial myPort;        // The serial port

//initialize all variables
float in_x = 70; //current value of the first variable in the string
float last_x = 0; //previous value of the first variable in the string
float in_y = 70; //current value of the second variable in the string
float last_y = 0; //previous value of the second variable in the string
float in_theta = PI/2;
float last_theta = 0;

void setup () {
  // set the window size:
  size(600, 400);        

  // List all the available serial ports
  println(Serial.list());
  // Check the listed serial ports in your machine
  // and use the correct index number in Serial.list()[].

  //note you may need to change port number, it is 9 for me
  myPort = new Serial(this, Serial.list()[1], 115200);  // also make sure baud rate matches Arduino
  

  // A serialEvent() is generated when a newline character is received :
  myPort.bufferUntil('\n');
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
  
  // outer boundary
  fill(255);
  rect(50,50,500,300);
  
  //inner obstacle
  fill(0);
  rect(200, 150, 200, 100);

  //// lower half circle, roundabout
  //fill(255);
  //circle(300, 285, 50);
  //noFill();
  //arc(300, 285, 200, 200, atan(0.15), PI - atan(0.15));
  //noFill();
  
  //// two rectangle, left and right
  //fill(255);
  //rect(75, 75, 185, 160, 0 , 0, 40, 0);
  //fill(255);
  //rect(340, 75, 185, 160, 0, 0, 0, 40);
  
  //// central lines
  //stroke(255);
  //strokeWeight(0.5);
  //line(40, 40, 560, 40);
  //line(40, 40, 40,270);
  //line(40, 270,220, 270);
  //line(380, 270, 560, 270);
  //line(560, 270, 560, 40);
  //line(300, 75, 300, 195);
  
  // location of the car 
  stroke(180, 110, 30);
  fill(180, 110, 30);
  ellipse(in_x, in_y, 10, 10);
  // representing direction
  strokeWeight(3);
  line(in_x, in_y, in_x + 15 * cos(in_theta), in_y + 15 * sin(in_theta));
  
  //fill(180, 110, 30);
  //rect(15, 200, 15, 25, 3, 3, 3, 3); 
}

void serialEvent (Serial myPort) {
  // get the ASCII string:
  // read the first part of the input string
  // HINT: use myPort.readStringUntil() with the appropriate argument
  // trim and convert string to a number
  String read_x = myPort.readStringUntil(',');
  String read_y = myPort.readStringUntil(',');
  String read_theta = myPort.readStringUntil('\n'); // 32 is the ASCII of "\n"
  if (read_x ==null || read_y == null || read_theta == null){
    return;
  }
  String text = trim(read_x);
  float x = float(text.substring(0, text.length()-1));
  text = trim(read_y);
  float y = float(text.substring(0, text.length()-1));
  text = trim(read_theta);
  float theta = float(text);
  // if: the number is NaN, set current value to previous value
  // otherwise: map the new value to the screen width
  //           & update previous value variable
    if (Float.isNaN(theta)){
    in_theta = last_theta;
  } else {
    in_theta = theta;
    last_theta = in_theta;
  }
  
    if (Float.isNaN(x)){
    in_x = last_x;
  } else {
    in_x = x;
    last_x = in_x;
  }
  
    if (Float.isNaN(y)){
    in_y = last_y;
  } else {
    in_y = y;
    last_y = in_y;
  }
}
