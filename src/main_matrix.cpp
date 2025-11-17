// #include <Arduino.h>
// #include "DisplayControl.h"
// #include "MotorControls.h"
// #include "ExternalInput.h"

// #define EXT_BUTTON 2
// #define BASE_Z_MOTOR 15
// #define BASE_Y_MOTOR 14
// #define ELBOW_Y_MOTOR 13
// #define WRIST_Y_MOTOR 10
// #define GRIPPER_MOTOR 9

// Adafruit_PWMServoDriver pwm_servo = Adafruit_PWMServoDriver(0x40);

// FullJointAngles initialAngles = {90, 27, 27, 132, 90};
// MotorControls motorControls(&pwm_servo, BASE_Z_MOTOR, BASE_Y_MOTOR, ELBOW_Y_MOTOR, WRIST_Y_MOTOR, GRIPPER_MOTOR);
// ExternalInput extInput;
// DisplayControl display(0x27, 16, 2);

// bool awake = false;
// bool armMoving = false;

// enum TransmitStates : uint8_t
// {
//     Awaken = 11,
//     AwakenComplete = 12,

//     Rest = 13,
//     RestComplete = 14,

//     ArmMoving = 15,
//     ArmMovingComplete = 16,

//     GripperOpen = 21,
//     GripperOpenComplete= 22,

//     GripperClose = 23,
//     GripperCloseComplete = 24,

//     GripperGrasp = 25,
//     GripperGraspComplete = 26
// };

// volatile unsigned long lastDebounceTime = 0;
// const unsigned long debounceDelay = 1000;
// void handleButtonPress()
// {
//   unsigned long currentTime = millis();
//   if ((currentTime - lastDebounceTime) < debounceDelay) {
//       return;
//   }   

//   lastDebounceTime = currentTime;

//   if (armMoving) 
//     return;

//   if (!awake)
//   {
//     Serial.println(TransmitStates::Awaken);  
//     awake = true;
//     // display.displayLine(1, "Awaken", false);
//   }
//   else
//   {
//     Serial.println(TransmitStates::Rest);
//     awake = false;
//     // display.displayLine(1, "Rest  ", false);
//   }
// }


// void setup() {
//   display.init();

//   Serial.begin(115200);  

//   pinMode(EXT_BUTTON, INPUT);  
//   attachInterrupt(digitalPinToInterrupt(EXT_BUTTON), handleButtonPress, RISING);

//   motorControls.setUp(initialAngles);
// }

// String inputString = "";
// FullJointAngles currAngles = initialAngles;
    

// static float radToDeg(float radians)
// {
//     return radians*180.0f/M_PI;
// }

// const int VECTOR_SIZE = 5;
// float vec[VECTOR_SIZE];   // match float size = 4 bytes

// bool readVectorFromSerial() {

//   const int numBytes = VECTOR_SIZE * sizeof(float);
//   static uint8_t* ptr = reinterpret_cast<uint8_t*>(vec);
//   static int received = 0;

//   while (Serial.available() && received < numBytes) {
//     ptr[received++] = Serial.read();
//   }

//   if (received == numBytes) {
//     received = 0;
//     return true;
//   }

//   return false; 
// }
// void handleArmAngles()
// {
//   String str;
//   for (int i = 0; i < VECTOR_SIZE; i++) {
//     int value = radToDeg(vec[i]) + SERIAL_ANGLE_OFFSET;
//     if (i == 2)
//     {
//       value = -1*radToDeg(vec[i]) + SERIAL_ANGLE_OFFSET;
//     }

//     currAngles.joints[i] = value;
//     str.concat(value);
//     str.concat(" ");
//   }

//   currAngles.j5_gripper_angle = 90;
//   display.displayLine(0, str, true);
//   motorControls.setEntireArmAngles(currAngles);    
// }

// // static bool finished = false;

// void reconstructArmMatrixFromBuffer(FullJointAngles* matrix, const uint8_t* bodyBuf, size_t rows, size_t cols)
// {
//     const float* float_buf = reinterpret_cast<const float*>(bodyBuf);
//     size_t float_idx = 0;
//     for (size_t i = 0; i < rows; ++i)
//     {
//         for (size_t j = 0; j < cols; ++j)
//         {
//             float val = float_buf[float_idx++];
//             int16_t deg = static_cast<int16_t>(radToDeg(val));

//             if (j == 2) // elbow joint
//             {
//                 matrix[i].joints[j] = -deg + SERIAL_ANGLE_OFFSET;
//             }
//             else
//             {
//                 matrix[i].joints[j] = deg + SERIAL_ANGLE_OFFSET;
//             }
//         }
//         matrix[i].j5_gripper_angle = motorControls.getEntireArmAngles().j5_gripper_angle;
//     }
// }


// void reconstructGripperMatrixFromBuffer(FullJointAngles* matrix, const uint8_t* bodyBuf, size_t rows, size_t cols)
// {
//     const float* float_buf = reinterpret_cast<const float*>(bodyBuf);
//     size_t float_idx = 0;
//     for (size_t i = 0; i < rows; ++i)
//     {      
//       float val = float_buf[float_idx++];
//       int16_t deg = static_cast<int16_t>(radToDeg(val));

//       matrix[i] = motorControls.getEntireArmAngles();
//       matrix[i].j5_gripper_angle = deg + SERIAL_ANGLE_OFFSET;
//     }
// }

// FullJointAngles* readMatrixFromSerial(int& outRows, TransmitStates& outTransmissionType)
// {  
//   const size_t headerBytes = 4 * sizeof(float);
//   uint8_t headerBuf[headerBytes]; 
//   size_t headerBytesRead = 0;
  
//   while (headerBytesRead < headerBytes)
//   {   
//       if (Serial.available())
//         headerBytesRead += Serial.readBytes(headerBuf + headerBytesRead, headerBytes - headerBytesRead);
//   }

//   if (headerBytesRead == headerBytes)
//   {   
//     float* headerFloats = reinterpret_cast<float*>(headerBuf);

//     const unsigned int rows = static_cast<int>(headerFloats[0]);
//     const unsigned int cols = static_cast<int>(headerFloats[1]); 
//     const unsigned int part = static_cast<int>(headerFloats[2]); // Arm: 1 | Gripper: 2
  
//     FullJointAngles* matrix = new FullJointAngles[rows];
   
//     const size_t bodyBytes = rows * cols * sizeof(float);
//     uint8_t bodyBuf[bodyBytes];
//     size_t bodyBytesRead = 0;

//     while (bodyBytesRead < bodyBytes)
//     {   
//       if (Serial.available())
//         bodyBytesRead += Serial.readBytes(bodyBuf + bodyBytesRead, bodyBytes - bodyBytesRead);
//     }

//     if (bodyBytesRead == bodyBytes)
//     {
//       // Reconstruct floats
//       if (part > 10 && part <= 19)
//       {        
//         reconstructArmMatrixFromBuffer(matrix, bodyBuf, rows, cols);
//       }
//       else if (part > 20 && part <= 29)
//       {        
//         reconstructGripperMatrixFromBuffer(matrix, bodyBuf, rows, cols);
//       }
//     }    

//     outRows = rows;
//     outTransmissionType = static_cast<TransmitStates>(part);
//     return matrix;
//   } 

//   return nullptr; 
// }

// FullJointAngles* matrix  = nullptr;  

// void loop() {

//   if (armMoving == true) return;

//   int rows;
//   TransmitStates transmissionType;

//   if (matrix == nullptr)
//   {
//       matrix = readMatrixFromSerial(rows, transmissionType);
//   }

//   if (matrix != nullptr)
//   {   
//     armMoving = true;
//     for (int i = 0; i < rows; i++)
//     {
//         motorControls.setEntireArmAngles(matrix[i]);
//         delay(95);
//     }  

//     delete[] matrix;   // free memory
//     matrix = nullptr;  // reset pointer

//     if (transmissionType == TransmitStates::ArmMoving)
//     {
//       Serial.println(TransmitStates::ArmMovingComplete);
//     }
//     else if (transmissionType == TransmitStates::Awaken)
//     {
//       Serial.println(TransmitStates::AwakenComplete);
//     }
//     else if (transmissionType == TransmitStates::Rest)
//     {
//       Serial.println(TransmitStates::RestComplete);
//     }
//     else if (transmissionType == TransmitStates::GripperOpen)
//     { 
//       Serial.println(TransmitStates::GripperOpenComplete);
//     }
//     else if (transmissionType == TransmitStates::GripperClose)
//     { 
//       Serial.println(TransmitStates::GripperCloseComplete);
//     }
//     else if (transmissionType == TransmitStates::GripperGrasp)
//     { 
//       Serial.println(TransmitStates::GripperGraspComplete);
//     }
   
//     armMoving = false;
//   } 
// }
