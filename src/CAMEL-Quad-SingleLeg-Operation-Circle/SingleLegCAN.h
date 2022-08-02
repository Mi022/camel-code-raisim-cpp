#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <sstream>
#include <cmath>

// author@ Jaehyeong Park & Jaehoon An
/*
    0x142 : knee
    0x141 : hip
*/

class SingleLegCAN {
public:
    SingleLegCAN(char *canName, std::string canName_temp, std::string bitRate) {
        std::string command3 =
                "sudo ip link set " + canName_temp + " up type can bitrate " + bitRate; // TODO: should be modified.
        const char *c3 = command3.c_str();
        system(c3);
        initCanInterface(canName);
    }

    double rad2deg = 180.0 / 3.141592;
    double deg2rad = 3.141592 / 180.0;
    double enc2rad = 2 * 3.141592 / 65535;

    void initCanInterface(const char *CanName);
    void canSend(const u_int8_t *data, int motorID);
    void canRead();

    void optionSetMotorOffsetToCurrentPosition(int motorID);

    void readEncoder(int motorID);
    void readMultiturnAngularPosition(int motorID);
    void readMotorStatus2(int motorID);

    void stopMotor(int motorID);
    void turnOnMotor(int motorID);
    void turnOffMotor(int motorID);

    void setTorque(int motorID, double torque);
    void setVelocity(int motorID, int Velocity);
    void setPosition1(int motorID, double position);

    int getSock() { return mSock; }
    int getEncoder1() { return mEncoder1; }
    int getEncoder2() { return mEncoder2; }
    int getEncoderRaw1() { return mEncoderRaw1; }
    int getEncoderRaw2() { return mEncoderRaw2; }
    int getEncoderOffset1() { return mEncoderOffset1; }
    int getEncoderOffset2() { return mEncoderOffset2; }
    double getAngularPosition1() { return -mAngularPosition_rad1; }
    double getAngularPosition2() { return -mAngularPosition_rad2; }
    double getAngularVelocity1() { return -mAngularVelocity_rad1; }
    double getAngularVelocity2() { return -mAngularVelocity_rad2; }
    can_frame getFrame() { return mFrame; }

private:
    struct can_frame mFrame;
    const char *mCanName;
    double mAngularPosition_rad1 = 0;
    double mAngularVelocity_rad1 = 0;
    double mAngularPosition_rad2 = 0;
    double mAngularVelocity_rad2 = 0;
    int mMotorHipID = 0x141;
    int mMotorKneeID = 0x142;
    int mEncoder1 = 0;
    int mEncoder2 = 0;
    int mEncoderMultiturnNum1 = 0;
    int mEncoderMultiturnNum2 = 0;
    int mEncoder_temp1 = 35000;
    int mEncoder_temp2 = 35000;
    int mEncoderPast1 = 35000;
    int mEncoderPast2 = 35000;
    int mEncoderRaw1;
    int mEncoderRaw2;
    int mEncoderOffset1;
    int mEncoderOffset2;
    int mSock;
    int mSendedCommand;
    int mGearRatio = 9;
    u_int32_t mCanID;
};


