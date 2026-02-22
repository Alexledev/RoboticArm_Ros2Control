#include <Arduino.h>
#include "DisplayControl.h"
#include "MotorControls.h"
#include "ExternalInput.h"

#define EXT_BUTTON 2
#define GRIP_SENSOR 3
#define BASE_Z_MOTOR 15
#define BASE_Y_MOTOR 14
#define ELBOW_Y_MOTOR 13
#define WRIST_Y_MOTOR 10
#define WRIST_X_MOTOR 11
#define GRIPPER_MOTOR 9

enum TransmitStates : uint8_t
{
    Awaken = 11,
    AwakenComplete = 12,

    Rest = 13,
    RestComplete = 14,

    ArmMoving = 15,
    
    ArmMovingEnd = 16,
    ArmMovingComplete = 17,

    GripperOpen = 21,
    GripperOpenComplete= 22,

    GripperClose = 23,
    GripperCloseComplete = 24,

    GripperGrasp = 25,
    GripperGraspComplete = 26
};

Adafruit_PWMServoDriver pwm_servo = Adafruit_PWMServoDriver(0x40);

FullJointAngles initialAngles = {90, 15, 30, 125, 90, 90}; //138  27
MotorControls motorControls(&pwm_servo, BASE_Z_MOTOR, BASE_Y_MOTOR, ELBOW_Y_MOTOR, WRIST_Y_MOTOR, WRIST_X_MOTOR, GRIPPER_MOTOR);
ExternalInput extInput;
// DisplayControl display(0x27, 16, 2);
TransmitStates transmissionType;

bool awake = false;
bool armMoving = false;


volatile unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 1000;
void handleButtonPress()
{
  unsigned long currentTime = millis();
  if ((currentTime - lastDebounceTime) < debounceDelay) {
      return;
  }   

  lastDebounceTime = currentTime;

  if (armMoving) 
    return;

  if (!awake)
  {
    Serial.println(TransmitStates::Awaken);  
    awake = true;
    // display.displayLine(1, "Awaken", false);
  }
  else
  {
    Serial.println(TransmitStates::Rest);
    awake = false;
    // display.displayLine(1, "Rest  ", false);
  }
}


void gripperClosed()
{
  // if (transmissionType == TransmitStates::GripperGrasp)
  // {
  Serial.println(TransmitStates::GripperGraspComplete);
  // } 
}

void setup() {
  // display.init();

  Serial.begin(115200);  

  pinMode(EXT_BUTTON, INPUT);  
  pinMode(GRIP_SENSOR, INPUT);
  attachInterrupt(digitalPinToInterrupt(EXT_BUTTON), handleButtonPress, RISING);
  attachInterrupt(digitalPinToInterrupt(GRIP_SENSOR), gripperClosed, RISING);

  motorControls.setUp(initialAngles);
}

String inputString = "";
FullJointAngles currAngles = initialAngles;
    

static float radToDeg(float radians)
{
    return radians*180.0f/M_PI;
}

const int VECTOR_SIZE = 7;
float vec[VECTOR_SIZE];   // match float size = 4 bytes

// void displayInfo()
// {
//   String disp = String(static_cast<int>(vec[0])) + "|" + String(vec[1]) + "|" + String(vec[2]);
//   display.displayLine(0, disp, true);

//   String disp2 = String(vec[3]) + "|" + String(vec[4])+ "|" + String(vec[5]);
//   display.displayLine(1, disp2, false);
  
// }

bool readVectorFromSerial(TransmitStates& transmit) {

  const int numBytes = VECTOR_SIZE * sizeof(float);
  static uint8_t* ptr = reinterpret_cast<uint8_t*>(vec);
  static int received = 0;

  while (received < numBytes) {
    if (Serial.available() )
        ptr[received++] = Serial.read();
  }

  if (received == numBytes) {
    received = 0;

    transmit = static_cast<TransmitStates>( static_cast<int>(vec[0]) );
    // displayInfo();
    return true;
  }

  return false; 
}

void handleArmAngles()
{ 
  for (int i = 1; i < VECTOR_SIZE; i++) { // skip header value
    int value = radToDeg(vec[i]) + SERIAL_ANGLE_OFFSET;
    if (i == 3)
    {
      value = -1*radToDeg(vec[i]) + SERIAL_ANGLE_OFFSET;
    }

    currAngles.joints[i-1] = value;   
  }

  motorControls.setEntireArmAngles(currAngles);    
}

void loop() {
 
  if (!readVectorFromSerial(transmissionType))
    return;
    
  handleArmAngles();

  if (transmissionType == TransmitStates::ArmMovingEnd)
  {
    Serial.println(TransmitStates::ArmMovingComplete);
  }
  else if (transmissionType == TransmitStates::Awaken)
  {
    Serial.println(TransmitStates::AwakenComplete);
  }
  else if (transmissionType == TransmitStates::Rest)
  {
    Serial.println(TransmitStates::RestComplete);
  }
  else if (transmissionType == TransmitStates::GripperOpen)
  { 
    Serial.println(TransmitStates::GripperOpenComplete);
  }
  else if (transmissionType == TransmitStates::GripperClose)
  { 
    Serial.println(TransmitStates::GripperCloseComplete);        
  }
  // else if (transmissionType == TransmitStates::GripperGrasp)
  // { 
  //   Serial.println(TransmitStates::GripperGraspComplete);
  // }
    
}
