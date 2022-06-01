//--------------------------------------------------------------------------
// Code to test basic Hapkit functionality (sensing and force output)
// Updated by Allison Okamura 1.17.2018
//--------------------------------------------------------------------------

// Includes
#include <math.h>

// Pin declares
int pwmPin = 5; // PWM output pin for motor 1
int dirPin = 8; // direction output pin for motor 1
int sensorPosPin = A2; // input pin for MR sensor
int fsrPin = A3; // input pin for FSR sensor

// Position tracking variables
int updatedPos = 0;     // keeps track of the latest updated value of the MR sensor reading
int rawPos = 0;         // current raw reading from MR sensor
int lastRawPos = 0;     // last raw reading from MR sensor
int lastLastRawPos = 0; // last last raw reading from MR sensor
int flipNumber = 0;     // keeps track of the number of flips over the 180deg mark
int tempOffset = 0;
int rawDiff = 0;
int lastRawDiff = 0;
int rawOffset = 0;
int lastRawOffset = 0;
const int flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flipped = false;
double OFFSET = 980;
double OFFSET_NEG = 15;

// Kinematics variables
double xh = 0;           // position of the handle [m]

// Force output variables
double force = 0;           // force at the handle
double Tp = 0;              // torque of the motor pulley
double duty = 0;            // duty cylce (between 0 and 255)
unsigned int output = 0;    // output command to the motor

// 2D maze
double world_height = 400;
double world_width = 600;
double resolution = 0.02;
int array2D[8][12]; // 8 <= 400*0.02; 12 <= 600*0.02


// world location using x, y; same as processing
double world_x = 70;
double world_y = 70;
double world_theta =  0; //pi/2
double steer_angle = 0; //steer wheel angle, to change world_theta
double steer_ratio = 0.005; // TOTUNE
bool collide = false;
double carSpeed = 0.08;



// --------------------------------------------------------------
// Setup function -- NO NEED TO EDIT
// --------------------------------------------------------------
void setup()
{
  // Set up serial communication
  Serial.begin(115200);

  // Set PWM frequency
  setPwmFrequency(pwmPin, 1);

  // Input pins
  pinMode(sensorPosPin, INPUT); // set MR sensor pin to be an input
  pinMode(fsrPin, INPUT);       // set FSR sensor pin to be an input

  // Output pins
  pinMode(pwmPin, OUTPUT);  // PWM pin for motor A
  pinMode(dirPin, OUTPUT);  // dir pin for motor A

  // Initialize motor
  analogWrite(pwmPin, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin, LOW);  // set direction

  // Initialize position valiables
  lastLastRawPos = analogRead(sensorPosPin);
  lastRawPos = analogRead(sensorPosPin);
  flipNumber = 0;

  // Build 2D Maze
  build_maze();

  for (int i = 0; i <= 7; i++) {
    for (int j = 0; j <= 11; j++) {
      Serial.print(array2D[i][j]);
    }
    Serial.println();
  }

}


// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------
void loop()
{

  //*************************************************************
  //*** Section 1. Compute position in counts (do not change) ***
  //*************************************************************

  // Get voltage output by MR sensor
  rawPos = analogRead(sensorPosPin);  //current raw position from MR sensor

  // Calculate differences between subsequent MR sensor readings
  rawDiff = rawPos - lastRawPos;          //difference btwn current raw position and last raw position
  lastRawDiff = rawPos - lastLastRawPos;  //difference btwn current raw position and last last raw position
  rawOffset = abs(rawDiff);
  lastRawOffset = abs(lastRawDiff);

  // Update position record-keeping vairables
  lastLastRawPos = lastRawPos;
  lastRawPos = rawPos;

  // Keep track of flips over 180 degrees
  if ((lastRawOffset > flipThresh) && (!flipped)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if (lastRawDiff > 0) {       // check to see which direction the drive wheel was turning
      flipNumber--;              // cw rotation
    } else {                     // if(rawDiff < 0)
      flipNumber++;              // ccw rotation
    }
    flipped = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    flipped = false;
  }
  updatedPos = rawPos + flipNumber * OFFSET; // need to update pos based on what most recent offset is


  //*************************************************************
  //*** Section 2. Compute position in meters *******************
  //*************************************************************

  // Define kinematic parameters you may need
  //  double rh = 0.09;   //[m]
  double rh = 1;
  // Step B.1:
  steer_angle = (updatedPos - 900) * (-0.01324) - 1.857 ; // in degree
  world_theta = steer_angle / 180 * PI * steer_ratio + world_theta;
  double x_temp = world_x + carSpeed * cos(world_theta);
  double y_temp = world_y + carSpeed * sin(world_theta);

  //check if collides
  //collide = collasion(x_temp, y_temp , world_theta, maize, resolution, ep);
  //collide = false;

  bool collide = array2D[int(y_temp * resolution)][int(x_temp * resolution)];
  if (!collide) {
    world_x = x_temp;
    world_y = y_temp;
    force = 0;
  } // if collide, x, y won't change

  else {
    x_temp = world_x + carSpeed * cos(world_theta + PI / 2);
    y_temp = world_y + carSpeed * sin(world_theta + PI / 2);

    int x_map = x_temp * resolution;
    int y_map = y_temp * resolution;
    bool new_collide = array2D[y_map][x_map] == 1;
    if (new_collide) {
      force = -0.02;
    } else {
      force = 0.02;
    }

  } // if collide, x, y won't change
  Serial.print(world_x, 5);
  Serial.print(',');
  Serial.print(world_y, 5);
  Serial.print(',');
  Serial.println(world_theta, 5);

  // Step B.6:
  double ts = 0.01199 * updatedPos - 2.447 ; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  // Step B.7:
  // xh = rh * ts * 3.1415926 / 180;      // Compute the position of the handle (in meters) based on ts (in radians)
  // Step B.8: print xh via serial monitor
  // Serial.println(xh, 5);

  //*************************************************************
  //*** Section 3. Assign a motor output force in Newtons *******
  //*************************************************************

  // ADD YOUR CODE HERE
  // Define kinematic parameters you may need
  double rp = 0.00476;   //[m]
  double rs = 0.073;   //[m]
  // Step C.1:
  //  force = 0; // You can  generate a force by assigning this to a constant number (in Newtons) or use a haptic rendering / virtual environment
  // Step C.2:
  double k = 300;
  //     force = -k * xh;
  Tp = rh * rp * force / rs;    // Compute the require motor pulley torque (Tp) to generate that force using kinematics

  //*************************************************************
  //*** Section 4. Force output (do not change) *****************
  //*************************************************************

  // Determine correct direction for motor torque
  if (force > 0) {
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  duty = sqrt(abs(Tp) / 0.0183);

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {
    duty = 1;
  } else if (duty < 0) {
    duty = 0;
  }
  output = (int)(duty * 255);  // convert duty cycle to output signal
  analogWrite(pwmPin, output); // output the signal
}

// --------------------------------------------------------------
// Function to set PWM Freq -- DO NOT EDIT
// --------------------------------------------------------------
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if (pin == 3 || pin == 11) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}




// --------------------------------------------------------------
// 2D Maze Helper Functions
// --------------------------------------------------------------
void build_maze() {
  int maze_height = resolution * world_height;
  int maze_width = resolution * world_width;

  for (int i = 0; i < 8; i++) {
    array2D[i][0] = 1;
    array2D[i][11] = 1;
  }
  for (int j = 0; j < 12; j++) {
    array2D[0][j] = 1;
    array2D[7][j] = 1;
  }

  for (int i = 3; i <= 4; i++) {
    for (int j = 4; j <= 7; j++) {
      array2D[i][j] = 1;
    }
  }

  for (int i = 2; i <= 5; i++) {
    array2D[i][1] = 1;
  }

  array2D[5][9] = 1;

  for (int i = 1; i <= 2; i++) {
    array2D[i][9] = 1;
    array2D[i][10] = 1;
  }
  array2D[1][4] = 1;
  array2D[1][5] = 1;
  array2D[6][4] = 1;
  array2D[6][5] = 1;
  array2D[6][6] = 1;
}
