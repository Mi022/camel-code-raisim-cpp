#ifndef RAISIM_MOTORCAN_H
#define RAISIM_MOTORCAN_H

#include <iostream>
#include <unistd.h>
#include <cmath>
#include <net/if.h>
#include <cstring>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include "SharedMemory.h"

/*
 * two RMD-X8-pro-V2 are used in the robot.
 */
class MotorCAN {
public:
    MotorCAN(std::string canName)
    {
        mCanName = canName;
        mMotorId = 0x142;
        enc2rad = 2.0 * 3.141592 / 65535;
        torque2int = 24.0385;
        mEncoder = 0;
        mEncoderMultiturnNum = 0;
        mEncoderTemp = 35000;
        mEncoderPast = 35000;
        mEncoderRaw = 0;
        mEncoderOffset = 0;
        mSock = 0;
        mSendedCommand = 0;
        mGearRatio = 9;
        mMotorTemperature = 0;
        mMotorErrorCode = 0;

        mAngularPosition = 0;
        mAngularVelocity = 0;
        mCurrentTorque = 0;
        mMotorVoltage = 0;
    }
    double enc2rad;
    double torque2int;

private:
    std::string mCanName;
    struct can_frame mFrame;
    int mMotorId;
    int mEncoder;
    int mEncoderMultiturnNum;
    int mEncoderTemp;
    int mEncoderPast;
    int mEncoderRaw;
    int mEncoderOffset;
    int mSock;
    int mSendedCommand;
    int mGearRatio;
    int mMotorTemperature;
    int mMotorErrorCode;
    double mAngularPosition;
    double mAngularVelocity;
    double mCurrentTorque;
    double mMotorVoltage;

public:
    void canInit();
    void canSend(const u_int8_t *data);
    void canRead();
    void readEncoder();
    void readMotorErrorStatus();
    void stopMotor();
    void turnOnMotor();
    void turnOffMotor();

    void setTorque(double desiredTorque);
    void setVelocity(double desiredVelocity);
    void setPosition(double desiredPosition);

    can_frame getFrame() { return mFrame; }
    int getSock() { return mSock; }
    int getEncoder() { return mEncoder; }
    int getEncoderRaw() { return mEncoderRaw; }
    int getEncoderOffset() { return mEncoderOffset; }
};


#endif //RAISIM_MOTORCAN_H
