//--------------------------------------------------------------------------
// Code to test basic Hapkit functionality (sensing and force output)
// Code updated by Cara Nunez 4.17.19
//--------------------------------------------------------------------------
// Parameters that define what environment to render
// #define ENABLE_VIRTUAL_WALL
// #define ENABLE_LINEAR_DAMPING
//#define ENABLE_NONLINEAR_FRICTION
//#define ENABLE_HARD_SURFACE
// #define ENABLE_BUMP_VALLEY
//#define ENABLE_TEXTURE
# define ENABLE_SPRING_DAMPING_Assignment5

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

// Kinematics variables
double xh = 0;           // position of the handle [m]
double theta_s = 0;      // Angle of the sector pulley in deg
double xh_prev;          // Distance of the handle at previous time step
double xh_prev2;
double dxh;              // Velocity of the handle
double dxh_prev;
double dxh_prev2;
double dxh_filt;         // Filtered velocity of the handle
double dxh_filt_prev;
double dxh_filt_prev2;

// Force output variables
double force = 0;           // force at the handle
double Tp = 0;              // torque of the motor pulley
double duty = 0;            // duty cylce (between 0 and 255)
unsigned int output = 0;    // output command to the motor

// Assignment5 Problem2 Additional Variables
double mass = 2;
double b = 1;
double kspring = 300;
double kuser = 1000;
double xmass = 0.005;

double dxmass = 0;      // Velocity of the mass
double dxmass_prev = 0;
double force_mass = 0;
double acc_mass = 0;


// definition for final project
double resolution = 0.02;
// world height and width
unsigned world_height = 400;
unsigned world_width = 600;
int array2D[8][12];


// world location using x, y; same as processing
double world_x = 580;
double world_y = 220;
double world_theta = 1.57; //pi/2
int * map_ptr;
int map_ij[2];
double world_xy[2];
double stir_angle = 0; //stirring wheel angle, to change world_theta
double stirring_ratio = 0.2; // TOTUNE
bool collide = false;
unsigned ep = 10;
double carSpeed = 10;



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
  // build the maize
  build_maize(resolution, world_height, world_width);
  // car location initialize
  world_xy[0] = world_x;
  world_xy[1] = world_y;
  map_ptr = world2map(resolution, world_xy);
  map_ij[0] = map_ptr[0];
  map_ij[1] = map_ptr[1];
  //int * world_ptr = map2world(resolution, map_ij);
  Serial.println("end of setup");
  Serial.println(map_ij[0]);
  Serial.println(map_ij[1]);
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
    if (rawOffset > flipThresh) { // check to see if the data was good and the most current offset is above the threshold
      updatedPos = rawPos + flipNumber * rawOffset; // update the pos value to account for flips over 180deg using the most current offset
      tempOffset = rawOffset;
    } else {                     // in this case there was a blip in the data and we want to use lastactualOffset instead
      updatedPos = rawPos + flipNumber * lastRawOffset; // update the pos value to account for any flips over 180deg using the LAST offset
      tempOffset = lastRawOffset;
    }
    flipped = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    updatedPos = rawPos + flipNumber * tempOffset; // need to update pos based on what most recent offset is
    flipped = false;
  }

  //*************************************************************
  //*** Section 2. Compute position in meters *******************
  //*************************************************************

  // ADD YOUR CODE HERE (Use your code from Problems 1 and 2)
  // Define kinematic parameters you may need
  //double rh = ?;   //[m]
  double rh = 0.09;   //[m]
  // Step B.1: print updatedPos via serial monitor
  // Serial.println(updatedPos);
  // Step B.6: double ts = ?; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  double ts = updatedPos * (-0.01169) + 7.53957;
  // Step B.7: xh = ?;       // Compute the position of the handle (in meters) based on ts (in radians)
  xh = rh * ts / 180 * 3.1415926;
  // Step B.8: print xh via serial monitor
  // Serial.println(xh,5);
  // Calculate velocity with loop time estimation
  dxh = (double)(xh - xh_prev) / 0.001;

  // Calculate the filtered velocity of the handle using an infinite impulse response filter
  dxh_filt = .7 * dxh + .3 * dxh_prev; // having more filtering

  // Record the position and velocity
  xh_prev2 = xh_prev;
  xh_prev = xh;

  dxh_prev2 = dxh_prev;
  dxh_prev = dxh;

  dxh_filt_prev2 = dxh_filt_prev;
  dxh_filt_prev = dxh_filt;

  // final project codes
  // calibration
  stir_angle = updatedPos * (-0.01324) - 1.857; // TOTUNE
  map_ptr = world2map(resolution, world_xy);
  map_ij[0] = map_ptr[0];
  map_ij[1] = map_ptr[1];

  // change of location and direction
  world_theta = stir_angle * stirring_ratio + world_theta;
  double x_temp = world_x + carSpeed * cos(world_theta);
  double y_temp = world_y + carSpeed * sin(world_theta);
  Serial.println("check collision");
  //check if collides
  collide = collasion(x_temp, y_temp , world_theta, resolution, ep);

  if (!collide) {
    world_x = x_temp;
    world_y = y_temp;
  } // if collide, x, y won't change


  //*************************************************************
  //*** Section 3. Assign a motor output force in Newtons *******
  //*************************************************************
  //*************************************************************
  //******************* Rendering Algorithms ********************
  //*************************************************************

#ifdef ENABLE_VIRTUAL_WALL
  if (xh > 0.005) { // if penetrating wall
    force = -800 * (xh - 0.005); // max stiffness set as 800
  }
  else {
    force = 0;
  }
#endif


#ifdef ENABLE_SPRING_DAMPING_Assignment5
  force = - kuser * (max(0, xh - xmass)); // force output for hapkit
  force_mass = kuser * max(0, xh - xmass) + kspring * (0.005 - xmass) - b * dxmass;
  acc_mass = force_mass / mass;
  dxmass = dxmass + 0.001 * acc_mass; // Calculate velocity with loop time estimation
  xmass = xmass + 0.5 * (dxmass + dxmass_prev) * 0.001; // Calculate position with loop time estimation
  dxmass_prev = dxmass; // store the velocity
#endif

#ifdef ENABLE_LINEAR_DAMPING
  force = 10 * dxh_filt; // set damping coefficient as 10.
  //Serial.println(force);
#endif


  // ADD YOUR CODE HERE (Use ypur code from Problems 1 and 2)
  // Define kinematic parameters you may need
  //double rp = ?;   //[m]
  //double rs = ?;   //[m]
  // Step C.1: force = ?; // You can  generate a force by assigning this to a constant number (in Newtons) or use a haptic rendering / virtual environment
  // Step C.2: Tp = ?;    // Compute the require motor pulley torque (Tp) to generate that force using kinematics

  double rp = 0.00476;
  double rs = 0.073;

  double Tp = rh * rp / rs * force;

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
  duty = sqrt(abs(Tp) / 0.03);

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {
    duty = 1;
  } else if (duty < 0) {
    duty = 0;
  }
  output = (int)(duty * 255);  // convert duty cycle to output signal
  analogWrite(pwmPin, output); // output the signal
}

// When calling://TO BE CHANGED
//int **maize;
//array = new int *[10];
//for(int i = 0; i <10; i++)
//   array[i] = new int[10];
//processArr(array);


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
// Function fo map2world and world2map: the world is processing pos, the map is maize, resolution = 2 default.
// (maize matrix is larger than world, to be more accurate)
// --------------------------------------------------------------


double * map2world(double resolution, int * map_ij) {
  static double xy[2];
  xy[0] = map_ij[1] / resolution;
  xy[1] = map_ij[0] / resolution;
  return xy;
}

int * world2map(double resolution, double * world_xy) {
  static int ij[2];
  ij[0] = int(round(world_xy[1] * resolution));
  ij[1] = int(round(world_xy[0] * resolution));
  return ij;
}

bool collasion(double x, double y, double world_theta, double resolution, unsigned ep) {
  double xy[2] = {x, y};
  int * ij_ptr = world2map(resolution, xy);
  int i = ij_ptr[0];
  int j = ij_ptr[1];
  if (array2D[i + ep][j] == 1 || array2D[i - ep][j] == 1 || array2D[i][j - ep] == 1 || array2D[i][j + ep] == 1) {
    return true;
  } else {
    int newep = int(ceil(ep / 1.414));
    if (array2D[i + newep][j + newep] == 1 || array2D[i + newep][j - newep] == 1 || array2D[i - newep][j - newep] == 1 || array2D[i - newep][j + newep] == 1) {
      return true;
    }
  }
  return false;
}

void build_maize(double resolution, unsigned height, unsigned width) { // maize size is height * width
  Serial.println("maze dim");
  Serial.println(height);
  Serial.println(width);
  int maize_height = height * resolution;
  int maize_width = width * resolution;
  
  Serial.println(maize_height);
  Serial.println(maize_width);
  // Serial.println(maize_width);
  for (int h = 0; h < maize_height; h++) {
    for (int w = 0; w < maize_width; w++) {
      // fill in some initial values
      // (filling in zeros would be more logic, but this is just for the example)
      // array2D[h][w] = 0;
      Serial.println("h and w");
      Serial.print(h);
      Serial.print(" ");
      Serial.print(w);
      Serial.println(" ");
      if (h >= 0 && h <= 5 * resolution) {
        if (w >= 0 && w <= 600 * resolution) {
          Serial.println("first condition");
          array2D[h][w] = 1;
        }
      } 
      
      else if (h > 5 * resolution && h <= 75 * resolution) {
        if ((w >= 0 && w <= 5 * resolution) || (w >= 595 * resolution && w <= 600 * resolution)) {
          Serial.println("second condition");
          array2D[h][w] = 1;
        }
      } 
      
      else if (h > 75 * resolution && h <= 195 * resolution) {
        if ((w >= 0 && w <= 5 * resolution) || (w >= 595 * resolution && w <= 600 * resolution) || (w >= 75 * resolution && w <= 260 * resolution) || (w >= 390 * resolution || w <= 525 * resolution)) {
          Serial.println("third condition");
          array2D[h][w] = 1;
        }
      } 
      
      else if (h > 195 * resolution && h <= 235 * resolution) {
        Serial.println("fourth condition");
        if ((w >= 0 && w <= 5 * resolution) || (w >= 595 * resolution && w <= 600 * resolution) || (w >= 75 * resolution && w <= 220 * resolution) || (w >= 380 * resolution && w <= 525 * resolution)) {
          array2D[h][w] = 1;
        }
        if ((w >= 220 * resolution && w <= 260 * resolution) && ((sq(h - 195 * resolution) + sq(w - 220 * resolution) <= sq(40 * resolution)))) {
          array2D[h][w] = 1;
        }
        if ((w >= 340 * resolution && w <= 380 * resolution) && ((sq(h - 195 * resolution) + sq(w - 380 * resolution) <= sq(40 * resolution)))) {
          array2D[h][w] = 1;
        }
      } 

      else if (h > 235 * resolution && h <= 260 * resolution) {
        Serial.println("fourth condition");
        if ((w >= 0 && w <= 5 * resolution) || (w >= 595 * resolution && w <= 600 * resolution)) {
          array2D[h][w] = 1;
        }
      } 

      else if (h > 260 * resolution && h <= 310 * resolution) {
        Serial.println("fifth condition");
        if ((w >= 0 && w <= 5 * resolution) || (w >= 595 * resolution && w <= 600 * resolution)) {
          array2D[h][w] = 1;
        }
        if (sq(h - 285 * resolution) + sq(w - 300 * resolution) <= sq(25 * resolution)) {
          array2D[h][w] = 1;
        }
      } 
//        else {
//        if (sq(h - 285 * resolution) + sq(w - 300 * resolution) >= sq(50 * resolution)) {
//          array2D[h][w] = 1;
//        }
//      }
    }
  }
}
