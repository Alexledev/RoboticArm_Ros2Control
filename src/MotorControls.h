#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
// #include "MathUtils/EulerAngles.h"
// #include "MathUtils/Vector3.h"
// #include "ArmCalculations.h"

#define SERVO_MIN 82
#define SERVO_MAX 553

#define LOWERDEG 1
#define UPPERDEG 180

#ifndef MOTORCTRL_H
#define MOTORCTRL_H

struct FullJointAngles
{
  union {
    struct {
      int16_t j1_base_angle;
      int16_t j2_shoulder_angle;
      int16_t j3_elbow_angle;
      int16_t j4_wrist_angle;
      int16_t j5_gripper_angle;
    };
    int16_t joints[5];  // access by index
  };
};


class MotorControls
{
private:
  Adafruit_PWMServoDriver *pwm;

  const int8_t baseZMotor = 15;
  const int8_t baseYMotor = 14;
  const int8_t elbowYMotor = 13;
  const int8_t wristYMotor = 10;
  const int8_t gripperMotor = 9;

  // const int8_t closedClawAngle = 90;
  // const int8_t fullOpenClawAngle = 40;
  FullJointAngles currentRefAngles; 

  const float delayFactor = 2.4f;
  float _j1Psi = 0.0f;
  float _j1Theta = 0.0f;
  float _j2Theta = 0.0f;
  float _jwTheta = 0.0f;
public:

  MotorControls(
      Adafruit_PWMServoDriver *servo,
      int8_t baseZPin,
      int8_t baseYPin,
      int8_t elbowYPin,
      int8_t wristYPin,
      int8_t gripperPin)
      : pwm(servo),
        baseZMotor(baseZPin),
        baseYMotor(baseYPin),
        elbowYMotor(elbowYPin),
        wristYMotor(wristYPin),
        gripperMotor(gripperPin)
  {
  }

  void setSingleJointDeg(int deg, int8_t pin);
  void setUp(const FullJointAngles initialAngles);
  void setEntireArmAngles(const FullJointAngles angles);

  FullJointAngles getEntireArmAngles()
  {
    return currentRefAngles;
  }

  float calculateDelayMSPerDegree(float deg)
  {
      return deg * 2.4f;
  }

  void delayWith(const float& angle, float& store)
  {
    if (angle != store) 
    {
        float p = fabs(angle - store);
        delay(calculateDelayMSPerDegree(p));
        store = angle;
    }
  }

};

#endif