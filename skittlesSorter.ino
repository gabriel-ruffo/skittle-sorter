
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "Adafruit_TCS34725.h"


// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Servo skittlesServo;
int ledPin = 5;
int servoPin = 3;
// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4)
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 1);
//array for skittles classifications
#define NUM_COLORS  5

// Skittle colours to indices
#define COL_RED     0
#define COL_GREEN   3
#define COL_ORANGE  1
#define COL_YELLOW  2
#define COL_PURPLE  4

// Names for colours
#define COLNAME_RED     "RED"
#define COLNAME_GREEN   "GREEN"
#define COLNAME_ORANGE  "ORANGE"
#define COLNAME_YELLOW  "YELLOW"
#define COLNAME_PURPLE  "PURPLE"

// RGB channels in the array
#define CHANNEL_R   0
#define CHANNEL_G   1
#define CHANNEL_B   2

// Training colours (populate these manually, but these vectors must be of unit length (i.e. length 1))
float trainingColors[3][NUM_COLORS];    // 3(rgb) x NUM_COLORS.

// Last read colour
float rNorm = 0.0f;
float gNorm = 0.0f;
float bNorm = 0.0f;
float hue = 0.0f;
float saturation = 0.0f;
float brightness = 0.0f;

// Last classified class
int lastClass = -1;
float lastCosine = 0;

/*
 * Colour sensing
 */


void initializeTrainingColors() {
  // Skittle: red
  trainingColors[CHANNEL_R][COL_RED] = 0.879;
  trainingColors[CHANNEL_G][COL_RED] = 0.3134;
  trainingColors[CHANNEL_B][COL_RED] = 0.3309;

  // Skittle: green
  trainingColors[CHANNEL_R][COL_YELLOW] = 0.4684;
  trainingColors[CHANNEL_G][COL_YELLOW] = 0.8015;
  trainingColors[CHANNEL_B][COL_YELLOW] = 0.3627;

  // Skittle: orange
  trainingColors[CHANNEL_R][COL_GREEN] = 0.9158;
  trainingColors[CHANNEL_G][COL_GREEN] = 0.3108;
  trainingColors[CHANNEL_B][COL_GREEN] = 0.2309;

  // Skittle: yellow
  trainingColors[CHANNEL_R][COL_ORANGE] = 0.765;
  trainingColors[CHANNEL_G][COL_ORANGE] = 0.5894;
  trainingColors[CHANNEL_B][COL_ORANGE] = 0.2424;

  // Skittle: purple
  trainingColors[CHANNEL_R][COL_PURPLE] = 0.7143;
  trainingColors[CHANNEL_G][COL_PURPLE] = 0.4913;
  trainingColors[CHANNEL_B][COL_PURPLE] = 0.4699;
//
//  // Nothing
//  trainingColors[CHANNEL_R][COL_NOTHING] = 0.71;
//  trainingColors[CHANNEL_G][COL_NOTHING] = 0.55;
//  trainingColors[CHANNEL_B][COL_NOTHING] = 0.43;
}


void getNormalizedColor() {
  uint16_t r, g, b, c, colorTemp, lux;  
  tcs.getRawData(&r, &g, &b, &c);

  float lenVec = sqrt((float)r*(float)r + (float)g*(float)g + (float)b*(float)b);

  // Note: the Arduino only has 2k of RAM, so rNorm/gNorm/bNorm are global variables. 
  rNorm = (float)r/lenVec;
  gNorm = (float)g/lenVec;
  bNorm = (float)b/lenVec;
}


int getColorClass() {
  float distances[NUM_COLORS] = {0.0f};

  // Step 1: Compute the cosine similarity between the query vector and all the training colours. 
  for (int i=0; i<NUM_COLORS; i++) {
    // For normalized (unit length) vectors, the cosine similarity is the same as the dot product of the two vectors.
    float cosineSimilarity = rNorm*trainingColors[CHANNEL_R][i] + gNorm*trainingColors[CHANNEL_G][i] + bNorm*trainingColors[CHANNEL_B][i];
    distances[i] = cosineSimilarity;

    // DEBUG: Output cosines
    Serial.print("   C"); Serial.print(i); Serial.print(": "); Serial.println(cosineSimilarity, 3);
  }

  // Step 2: Find the vector with the highest cosine (meaning, the closest to the training color)

  float maxVal;// = distances[0];
  int maxIdx = 0;
  for (int i=0; i<NUM_COLORS; i++) {
    if (distances[i] > maxVal) {
      maxVal = distances[i];
      maxIdx = i;
    }
  }
//  maxVal = max(distances[0], distances[3]);
//  maxVal = max(maxVal,distances[1]);
//  maxVal = max(maxVal,distances[4]);
//  maxVal = max(maxVal,distances[5]);
//  maxVal = max(maxVal,distances[2]);
//
//if(maxVal == distances[0])
//  maxIdx = 0;
//  else if(maxVal == distances[1])
//  maxIdx = 1;
//  else if(maxVal == distances[2])
//  maxIdx = 2;
//  else if(maxVal == distances[3])
//  maxIdx = 3;
//  else if(maxVal == distances[4])
//  maxIdx = 4;
//  else if(maxVal == distances[4] && (abs(distances[5]) - abs(distances[6]) <= .02))
//  maxIdx = 6;
//  else if(maxVal == distances[4])
//  maxIdx = 5;
  // Step 3: Return the index of the minimum color
  lastCosine = maxVal;
  lastClass = maxIdx;
  return maxIdx;
}


void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  pinMode(ledPin,OUTPUT);
  skittlesServo.attach(servoPin);
  initializeTrainingColors();
  AFMS.begin();  // create with the default frequency 1.6KHz
  tcs.begin();
  
  myMotor->setSpeed(150);  // 10 rpm   
}


void loop() {
  digitalWrite(ledPin,HIGH);
  getNormalizedColor();
  int color = getColorClass();  
  moveServo(color);
  myMotor->step(50,FORWARD,DOUBLE);
  }

  void moveServo(int color){
  if(color == 0){
     Serial.println("MOVING TO RED");
  }
  else if(color == 1){
     Serial.println("MOVING TO YELLOW");
  }
  else if(color == 2){
     Serial.println("MOVING TO GREEN");
  }
  else if(color == 3){
    Serial.println("MOVING TO ORANGE");
  }
  else if(color == 4){
    Serial.println("MOVING TO PURPLE");
  }
  }
  

