#include <Arduino.h>
#include "MotorControls.h"

#ifndef EXTIN_H
#define EXTIN_H

#define SERIAL_ANGLE_OFFSET 90

class ExternalInput
{
private:    
    
    bool parseMessage(const String &msg, FullJointAngles &fja) {
         //b000s000e000w000g000

        int8_t baseIndex = msg.indexOf('b');
        int8_t shoulderIndex = msg.indexOf('s');
        int8_t elbowIndex = msg.indexOf('e');
        int8_t wristIndex = msg.indexOf('w');
        int8_t gripperIndex = msg.indexOf('g');

        if (baseIndex == -1 || shoulderIndex == -1 || elbowIndex == -1|| wristIndex == -1 || gripperIndex == -1) return false;

        fja.j1_base_angle = msg.substring(baseIndex + 1, shoulderIndex).toInt() + SERIAL_ANGLE_OFFSET;
        fja.j2_shoulder_angle = msg.substring(shoulderIndex + 1, elbowIndex).toInt() + SERIAL_ANGLE_OFFSET;
        fja.j3_elbow_angle = -1*msg.substring(elbowIndex + 1, wristIndex).toInt() + SERIAL_ANGLE_OFFSET;
        fja.j4_wrist_angle = msg.substring(wristIndex + 1, gripperIndex).toInt() + SERIAL_ANGLE_OFFSET;
        fja.j5_gripper_angle = msg.substring(gripperIndex + 1).toInt() + SERIAL_ANGLE_OFFSET;

        return true;
    }

public:
    bool getSerial( FullJointAngles& fja, DisplayControl& display)
    {        
        if (Serial.available() > 0)
        {
            String input = Serial.readStringUntil('\n');
            display.displayLine(0, input, true);
            return parseMessage(input, fja);
        }

        return false;
    }
};

#endif