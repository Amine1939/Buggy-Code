#include <Wire.h>
#include <Adafruit_PWMServoDrive.h>
#include <Adafruit_TCS34725.h>
#include <HCSR04.h>
#include <SoftwareSerial.h>
//------------------------------------------------------------------------------------------
#define SERVOMIN  150 // Minimum pulse length count for servo (0 degrees)
#define SERVOMAX  600 // Maximum pulse length count for servo (180 degrees)

// ULTRASONIC SENSOR PINS
#define TRIGPIN 7     // Trigger pin for ultrasonic sensor
#define ECHOPIN 8     // Echo pin for ultrasonic sensor

// MOTOR CONTROL PINS
#define MOTOR_LEFT_FORWARD 6     // Left motor forward direction
#define MOTOR_LEFT_BACKWARD 5    // Left motor backward direction
#define MOTOR_RIGHT_FORWARD 10   // Right motor forward direction
#define MOTOR_RIGHT_BACKWARD 9   // Right motor backward direction

// IR SENSOR PINS (used for line following)
#define SENSOR_LEFT A0     // Left IR sensor
#define SENSOR_CENTER A1   // Center IR sensor
#define SENSOR_RIGHT A2    // Right IR sensor

// WHEEL MOTOR SPEEDS
#define BASE_SPEED 150     // Default speed for straight motion
#define LEFT_SPEED 185     // Slightly higher speed for left motor correction
#define TURN_SPEED 160     // Speed for turning movements

#define END_STOP_SWITCH 4  // End-stop switch pin for gripper confirmation

// IR SENSOR THRESHOLDS
#define IR_THRESHOLD_LOW 550    // Threshold to detect black line
#define IR_THRESHOLD_HIGH 1023  // Max reading for IR sensors

// Bluetooth Module Pin Initialisation
SoftwareSerial bts (12, 13); 
//------------------------------------------------------------------------------------------
// Initialise Colour Sensor with specific integration time and gain
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_16X);

// Colour name mapping for identification
const char* colorNames[] = {"Purple", "Red", "Floor", "Green", "Black", "Yellow"};

String colour;  // Stores identified colour

const int numColors = 6;  // Total number of colours being tracked

// Pre-calibrated RGB values for each colour in same order as colorNames[]
const int colorR[] = {790, 2000, 1630, 540, 400, 2520};
const int colorG[] = {580, 440, 1080, 800, 300, 1550};
const int colorB[] = {860, 460, 810, 630, 270, 770};

uint16_t r, g, b, c;  // Variables to store RGB and clear values from sensor

float distance = 1000.0;  // Used for colour match confidence

// Function to find the closest matching colour using Euclidean distance
const char* findClosestColor(int r, int g, int b) {
    unsigned long minDistance = 9999999; // Set initial distance to a high value
    const char* bestMatch = "Unknown";

    for (int i = 0; i < numColors; i++) {
        long dr = r - colorR[i];
        long dg = g - colorG[i];
        long db = b - colorB[i];

        unsigned long distance = dr * dr + dg * dg + db * db; // Squared Euclidean distance
        if (distance < minDistance) {
            minDistance = distance;
            bestMatch = colorNames[i];
        }
    }

    return bestMatch;
}
//--------------------------------------------------------------------------------------------------------------------------------------
const int DETECTION_DISTANCE = 13; // Minimum distance (in cm) for object detection

// Initialise Ultrasonic Distance Sensor using defined pins
UltraSonicDistanceSensor distanceSensor(TRIGPIN, ECHOPIN);

// Function to get current distance from sensor
float getDistance() {
    return distanceSensor.measureDistanceCm();
}
//--------------------------------------------------------------------------------------------------------------------------------------
int lastDetected = 0; // Stores last known line detection: -1 = left, 1 = right, set as 0 initially

int wheelCorrection = 12; // Correction value for balancing motors

// Tracks last used speeds to avoid redundant commands
int lastMotorSpeedL = 0;
int lastMotorSpeedR = 0;

// Initialise PWM servo driver instance
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); 

// Set direction and speed for individual motors using PWM
void set_motor_pwm(int pwm, int IN1_PIN, int IN2_PIN) {
    if (pwm >= 0) {
        digitalWrite(IN1_PIN, LOW);
        analogWrite(IN2_PIN, pwm);
    } else {
        analogWrite(IN1_PIN, -pwm);
        digitalWrite(IN2_PIN, LOW);
    }
}

// Wrapper function to set both motors’ PWM signals
void set_motor_currents(int pwm_A, int pwm_B) {
    set_motor_pwm(pwm_A, MOTOR_LEFT_BACKWARD, MOTOR_LEFT_FORWARD);
    set_motor_pwm(pwm_B, MOTOR_RIGHT_BACKWARD, MOTOR_RIGHT_FORWARD);
}


// Function to handle line following behavior using three IR sensors
void lineFollow() {
    // Read analog values from left, center, and right sensors
    int left = analogRead(SENSOR_LEFT);
    int center = analogRead(SENSOR_CENTER);
    int right = analogRead(SENSOR_RIGHT);

    // Determine if each sensor detects the line (based on threshold)
    bool leftDetected = (left >= IR_THRESHOLD_LOW);
    bool centerDetected = (center >= IR_THRESHOLD_LOW);
    bool rightDetected = (right >= IR_THRESHOLD_LOW);

    // If the center sensor detects the line, go straight
    if (centerDetected) {
        moveForward(BASE_SPEED, BASE_SPEED);
    } 
    // If both left and center sensors detect the line, veer slightly left
    else if (leftDetected && centerDetected) {
        moveForward(100, BASE_SPEED);
        lastDetected = -1;  // Track last known direction (left)
    } 
    // If both right and center sensors detect the line, veer slightly right
    else if (rightDetected && centerDetected) {
        moveForward(BASE_SPEED, 100);
        lastDetected = 1;  // Track last known direction (right)
    } 
    // If only the left sensor detects the line, correct by turning left
    else if (leftDetected) {
        moveForward(50, BASE_SPEED);
        lastDetected = -1;
    } 
    // If only the right sensor detects the line, correct by turning right
    else if (rightDetected) {
        moveForward(BASE_SPEED, 50);
        lastDetected = 1;
    } 
    else {
        // If no sensors detect the line, turn in the last known direction
        if (lastDetected == -1) {
            turnLeft();  // Assume line was last on the left
        } else if (lastDetected == 1) {
            turnRight(); // Assume line was last on the right
        } else {
            stopMotors(); // No direction history, stop as fallback
        }
    }
}

// Drives forward at specified speeds for left and right motors
void moveForward(int leftSpeed, int rightSpeed) {
  if(leftSpeed != lastMotorSpeedL || rightSpeed != lastMotorSpeedR){
    analogWrite(MOTOR_LEFT_FORWARD, leftSpeed+wheelCorrection);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_FORWARD, rightSpeed-wheelCorrection);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
  }
}

// Turns the buggy to the left
void turnLeft() {
    analogWrite(MOTOR_LEFT_FORWARD, 0);
    analogWrite(MOTOR_LEFT_BACKWARD, TURN_SPEED);
    analogWrite(MOTOR_RIGHT_FORWARD, TURN_SPEED);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
}

// Turns the buggy to the right
void turnRight() {
    analogWrite(MOTOR_LEFT_FORWARD, TURN_SPEED);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_FORWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, TURN_SPEED);
}

// Immediately stop all motor movement
void stopMotors() {
    analogWrite(MOTOR_LEFT_FORWARD, 0);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_FORWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
}
//--------------------------------------------------------------------------------------------------------------------------------------
int armCurrent = 180; // Starting position of arm (fully raised)
int armTarget = 180;  // Desired arm position
const int armSweepingRate = 100; // Servo speed (steps per second)

// Converts angle to PWM pulse and sends to servo
void angleToServo(int servoNum, int targetAngle) {
  int pulseLen = map(targetAngle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servoNum, 0, pulseLen);
}

// Converts degrees to PWM pulse length
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// Sets both arm servos to complementary angles
void armAngle(int angle){
  angleToServo(1, angle);
  angleToServo(2, 180-angle);
}

// Smoothly sweep arm from current to target angle
void sweepArm(int target){
  armTarget = target;
  int stepDelay = 1000 / armSweepingRate;
  int step = (armCurrent < armTarget) ? 1 : -1;

  for (int pwm = armCurrent; pwm != armTarget; pwm += step) {
    armAngle(pwm);
    delay(stepDelay);
  }
  armCurrent = armTarget;
  armAngle(armCurrent);
}

//--------------------------------------------------------------------------------------------------------------------------------------
// Set gripper servo to specific angle
void gripperAngle(int angle){
  angleToServo(0,angle);
}

// Open gripper fully
void openGripper(){
  angleToServo(0,0);
}

// Close gripper to holding position
void closeGripper(){
  angleToServo(0,80);
}

// Return true if gripper successfully detects item via end-stop switch
bool isObjectGripped() {
  return digitalRead(END_STOP_SWITCH) == LOW;
}
//--------------------------------------------------------------------------------------------------------------------------------------
int chuteCurrent = 145;     // Current chute position
int chuteTarget = 145;      // Target chute position
const int chuteSweepingRate = 70; // Chute servo sweep rate

// Move the container chute to a given angle gradually
void sweepChute(int target){
  chuteTarget = target;
  int stepDelay = 1000 / chuteSweepingRate;
  int step = (chuteCurrent < chuteTarget) ? 1 : -1;

  for (int pwm = chuteCurrent; pwm != chuteTarget; pwm += step) {
    angleToServo(3,pwm);
    delay(stepDelay);
  }
  chuteCurrent = chuteTarget;
  angleToServo(3,chuteCurrent);
}

// Container control functions
void bucketOpen(){
  sweepChute(140); // Open trapdoor
}
void bucketClosed(){
  sweepChute(75);  // Lock trapdoor
}
void bucketShoot(){
  sweepChute(0);   // Fully eject item
}
//--------------------------------------------------------------------------------------------------------------------------------------
// Complete object pickup routine with retries and buzzer confirmation
void itemPickup() {
  set_motor_currents(0, 0); // Stop buggy
  delay(1000);

  // First pickup attempt
  sweepArm(0);
  delay(2000);
  closeGripper();
  delay(2000);

  // Retry to left
  if (!isObjectGripped()) {
    openGripper(); delay(1000);
    sweepArm(45); delay(1000);
    turnLeft(); delay(400);
    stopMotors(); delay(500);
    sweepArm(0); delay(1000);
    closeGripper(); delay(2000);
  }

  // Retry to right
  if (!isObjectGripped()) {
    openGripper(); delay(1000);
    sweepArm(45); delay(1000);
    turnRight(); delay(500);
    stopMotors(); delay(500);
    sweepArm(0); delay(1000);
    closeGripper(); delay(2000);
  }

  // Final verification and confirmation
  if (isObjectGripped()) {
    sweepArm(180); delay(2000);
    openGripper(); delay(2000);
    bucketClosed(); delay(2000);

    // Buzzer signal for successful pickup
    digitalWrite(2, HIGH); delay(100);
    digitalWrite(2, LOW); delay(50);
    digitalWrite(2, HIGH); delay(100);
    digitalWrite(2, LOW); delay(500);

  } else {
    bts.println("Failed to grip object."); // Print to bluetooth module connected to phone application
    lineFollow(); // Retry navigation if pickup fails
  }
}

//--------------------------------------------------------------------------------------------------------------------------------------
int hasObject; // Tracks whether object has been picked up

// Method for unloading object at base
void dropOff() {
  stopMotors(); delay(1000);
  turnRight(); delay(1900);
  lastDetected = 1;
  stopMotors();
  bucketShoot(); delay(1000);
  hasObject = 0; 
  bucketOpen();
}

//--------------------------------------------------------------------------------------------------------------------------------------
// Boolean variables to validate each colour-coded item picked up
bool red = false;
bool green = false;
bool purple = false;

// Set colour to 'u' intially and tracks last recognised colour
char lastColour = 'u';

// Count variable used to validate each action/method completed for memory
int count = 0;

// Fail-Safe varibales used for error handling 
bool accident1 = false;
bool accident2 = false;
bool accident3 = false;
bool accident4 = false;
--------------------------------------------------------------------------------------------------------------------------------------

// Setup
void setup(void)
{
  //SETUP SERIAL
  Serial.begin(9600);
  bts.begin(9600); // Establish Bluetooth connection from bluetooth module to phone application
  //SETUP COLOUR SENSOR
  if (!(tcs.begin())) {
    bts.println("No TCS34725 found ... check your connections");
    while (1);
  }
  
  pinMode(END_STOP_SWITCH, INPUT_PULLUP); // Configure the switch as input with pull-up
  pinMode(2, OUTPUT);

  //SETUP PWM DRIVER
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  pinMode(2,OUTPUT);
  //SET INITIAL ARM AND GRIPPER POSITION
  openGripper();
  delay(1000);
  bucketOpen();
  delay(1000);
  sweepArm(180);

  Serial.println(4);
  delay(4000);
  Serial.println(5);
  delay(4000);
  Serial.println(6);
  delay(1000);
  Serial.println(7);
  delay(1000);

  
  bts.println("let's go");
  delay(1000);
}

// Buggy Loop
void loop() {
    // COLOR SENSING
    tcs.getRawData(&r, &g, &b, &c);
    colour = findClosestColor(r,g,b);
    bts.println(colour);

    int distance = distanceSensor.measureDistanceCm();

    // If item is detected within detection distance, pick it up
    if (distance <= DETECTION_DISTANCE && hasObject == 0) {
        if (colour == "Red") {
            itemPickup();
            hasObject = 1;
            count = 2;
            bts.print("Count is: ");
            bts.print(count);
            red = true;
            bts.print("Red is ");
            bts.println(red);
            Serial.println(1);
        } else if (colour == "Green") {
            itemPickup();
            hasObject = 1;
            count = 7;
            bts.print("Count is: ");
            bts.print(count);
            green = true;
            bts.println("Green is ");
            bts.println(green);
            Serial.println(1);
        } else if (colour == "Purple") {
            itemPickup(); 
            hasObject = 1;
            purple = true;
            bts.println("Purple is ");
            bts.println(purple);
            Serial.println(1);
        } else { 
            // If no color is detected but an obstacle is present
            set_motor_currents(0, 0);
            bts.println("Obstacle detected. Sounding buzzer.");
            digitalWrite(2, HIGH);
            delay(400);
            digitalWrite(2, LOW);
            delay(100);
            digitalWrite(2, HIGH);
            delay(400);
            digitalWrite(2, LOW);
            delay(100);
            digitalWrite(2, HIGH);
            delay(400);
            digitalWrite(2, LOW);
            delay(100);
        }
    } else {
        lineFollow();
    }

    if (colour == "Red" && !red && !green && count == 0) {
      count = 1;
      bts.print("COUNT IS: ");
      bts.println(count);
      delay(1000);
    }

    if (colour != "Red" && red && count == 2) {
      count = 3;
      bts.print("COUNT IS: ");
      bts.println(count);
    }

    if (colour == "Red" && distance <= DETECTION_DISTANCE && count == 3 && hasObject == 1) {
      dropOff();
      bts.println("Red Dropped off successfully! ");
      Serial.println(2);
      count = 4;
      hasObject = 0;
    }

    if (colour == "Yellow" && red && !green && !purple && count == 4) {
      bts.print("COUNT IS: ");
      bts.println(count);
      Serial.println(3);
      delay(1000);
      accident1 = true;
      accident2 = true;
      count = 5;
    }

    if (colour == "Green" && red && !green && accident1 == true && count == 5) {
      set_motor_currents(0, 0);
      bts.println("Turning right from red to purple");
      turnRight();delay(200);
      accident1 = false;
      lineFollow();
    }

    if (colour == "Purple" && red && !green && accident2 == true) {
      set_motor_currents(0, 0);
      bts.println("Turning right from red to purple");
      turnLeft();delay(400);
      accident2 = false;
      lineFollow();
    }

    // Detect if back at the start (after a pickup cycle)
    if (colour == "Red" && (red && !green && count == 5)) { 
        // Red was done, back at start, need to turn left to green
        set_motor_currents(0, 0);
        Serial.println("Turning left from red to green");
        turnLeft();
        delay(500);
        accident1 = false;
        accident2 = false;
        count = 6;
        bts.print("COUNT IS: ");
        bts.println(count);
        lineFollow();
    } 

    if (colour != "Green" && green && count == 7) {
      bts.print("COUNT IS: ");
      bts.println(count);
      delay(1000);
      count = 8;
    }

    if (colour == "Green" && distance <= DETECTION_DISTANCE && count ==8 && hasObject == 1) {
      dropOff();
      Serial.println(2);
      bts.println("Red Dropped off successfully! ");
      hasObject = 0;
      count = 9;
    }

    if (colour == "Yellow" && red && green && !purple && count == 9) {
      count = 10;
      bts.print("COUNT IS: ");
      bts.println(count);
      accident3 = true;
      accident4= true;

      Serial.println(3);
      delay(1000);
    }

    if (colour == "Green" && green && red && !purple && accident3 == true) {
      set_motor_currents(0, 0);
      turnRight();
      delay(800);
      accident3 = false;
      lineFollow();
    }

    if (colour == "Red" && red && green && !purple && count == 10 && accident4 == true) { 
        // Purple was done, back at start, need to turn right
        delay(200);
        set_motor_currents(0, 0);
        Serial.println("Turning right from red to purple");
        turnRight();
        delay(400);
        count = 11;
        accident4 = false;
        lineFollow();
    }

    if (colour != "Purple" && purple && count == 11) {
      delay(1000);
      count = 12;
      bts.print("COUNT IS: ");
      bts.println(count);
      delay(1000);
    }

    if (colour == "Purple" && distance <= DETECTION_DISTANCE && purple && hasObject == 1 && count == 12) {
      delay(200);
      dropOff();
      Serial.println(2);
      bts.println("Red Dropped off successfully! ");
      delay(1000);
      count = 13;
      hasObject = 0;
      purple = true;
    }

    if (colour == "Yellow" && red && green && purple && count == 13 && hasObject == 0) {
      bts.print("COUNT IS: ");
      bts.println(count);
      set_motor_currents(0, 0);
      delay(1000);

      digitalWrite(2, HIGH);
      delay(100);
      digitalWrite(2, LOW);
      delay(50);
      digitalWrite(2, HIGH);
      delay(100);
      digitalWrite(2, LOW);
      delay(500);
    }
    
    bts.print("Count is: ");
    bts.println(count);
}
