#include "MotorControls.h"
// #include "MotorConfig.h"

void MotorControls::setSingleJointDeg(int deg, int8_t pin)
{
    if (deg < LOWERDEG) // degrees in range
    {
        deg = LOWERDEG;
    }
    else if (deg > UPPERDEG)
    {
        deg = UPPERDEG;
    }

    int pos = map(deg, LOWERDEG, UPPERDEG, SERVO_MIN, SERVO_MAX);
    pwm->setPWM(pin, 0, pos);
}

void MotorControls::setUp(const FullJointAngles initialAngles)
{
    pwm->begin();
    pwm->setPWMFreq(50); // Set frequency to 50 Hz for servos
    setEntireArmAngles(initialAngles);
}

void MotorControls::setEntireArmAngles(const FullJointAngles angles)
{

    if (angles.j1_base_angle > 10 && angles.j1_base_angle < 170)
    {        
        setSingleJointDeg(angles.j1_base_angle, baseZMotor);   // base motor M1
        delayWith(angles.j1_base_angle, _j1Psi);
    }

    if (angles.j2_shoulder_angle > 10 && angles.j2_shoulder_angle < 170)
    { 
        setSingleJointDeg(angles.j2_shoulder_angle, baseYMotor); // Lower Y motor
        delayWith(angles.j2_shoulder_angle, _j1Theta);
    }

    if (angles.j3_elbow_angle > 10 && angles.j3_elbow_angle < 170)
    { 
        setSingleJointDeg(angles.j3_elbow_angle, elbowYMotor);              // Back/Middle Y motor
        delayWith(angles.j3_elbow_angle, _j2Theta);
    }

    if (angles.j4_wrist_angle > 10 && angles.j4_wrist_angle < 170)
    { 
        setSingleJointDeg(angles.j4_wrist_angle, wristYMotor);
        delayWith(angles.j4_wrist_angle, _jwTheta);
    }
    
    if (angles.j5_gripper_angle > 10 && angles.j5_gripper_angle <= 90)
    { 
        setSingleJointDeg(angles.j5_gripper_angle, gripperMotor);
    }

    currentRefAngles = angles;
}
