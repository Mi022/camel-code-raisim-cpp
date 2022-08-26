//
// Created by jaehoon on 22. 5. 26.
//
#include "SingleLegCAN.h"

void SingleLegCAN::initCanInterface(const char *ifname) {
    //CAN socket 생성
    mCanName = ifname;
    mSock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (mSock == -1) {
        printf("Fail to create can socket for %s - %m \n", mCanName);
        return;
    }
    printf("Success to create can socket for %s\n", mCanName);

    //CAN 식별번호 획득
    struct ifreq ifr;
    strcpy(ifr.ifr_name, mCanName);
    int ret = ioctl(mSock, SIOCGIFINDEX, &ifr);
    if (ret == -1) {
        perror("Fail to get can interface index -");
        return;
    }
    printf("Success to get can interface index: %d\n", ifr.ifr_ifindex);

    //CAN socket 바인딩
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    ret = bind(mSock, (struct sockaddr *) &addr, sizeof(addr));
    if (ret == -1) {
        perror("Fail to bind can socket -");
        return;
    }
    printf("Success to bind can socket\n");
}

void SingleLegCAN::canSend(const u_int8_t *data, int motorID) // TODO : motorId should be specified.
{
    u_int32_t tempid = motorID & 0x1fffffff;
    mFrame.can_id = tempid;
    memcpy(mFrame.data, data, sizeof(data));
    mFrame.can_dlc = sizeof(data);

    int tx_bytes = write(mSock, &mFrame, sizeof(mFrame));
    if (tx_bytes == -1) {
        perror("Fail to transmit can frame -");
        return;
    }
    mSendedCommand = mFrame.data[0];
}

void SingleLegCAN::canRead() {
    int rx_bytes = read(mSock, &mFrame, sizeof(mFrame));
    // TODO : Should be changed to better method.
    int iteration = 0;
    while (mFrame.data[0] != mSendedCommand) {
        iteration++;
        rx_bytes = read(mSock, &mFrame, sizeof(mFrame));
        if (iteration > 100) {
            std::cout << "failed to read data in while loop" << std::endl;
            break;
        }
    }
}

void SingleLegCAN::optionSetMotorOffsetToCurrentPosition(int motorID) {
    u_int8_t data[8] = {0X19, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00};
    canSend(data, motorID);
}

// 1. Read Position loop KP data command (0x30)
// 2. Read Position lop Ki data command (0x31)
// 3. Read Speed loop KP data command (0x32)
// 4. Read Speed loop Ki data command (0x33)
// 5. Read Current loop KP data command (0x34)
// 6. Read Current loop Ki data command (0x35)
// 7. Write Position loop KP data to RAM command (0x36)
// 8. Write Position loop Ki data to RAM command (0x37)
// 9. Write Speed loop KP data to RAM command (0x38)
// 10. Write Speed loop Ki data to RAM command (0x39)
// 11. Write Current loop KP data to RAM command (0x3A)
// 12. Write Current loop Ki data to RAM command (0x3B)
// 13. Write Position loop KP data to ROM command (0x3C)
// 14. Write Position loop Ki data to ROM command (0x3D)
// 15. Write Speed loop KP data to ROM command (0x3E)
// 16. Write Speed loop Ki data to ROM command (0x3F)
// 17. Write Current loop KP data to ROM command (0x40)
// 18. Write Current loop Ki data to ROM command (0x41)
// 19. Read acceleration data command (0x42)
// 20. Write acceleration data to RAM command (0x43)
// 21. Read multiturn encoder position command (0x60)
// 22. Read multiturn encoder original position command (0x61)
// 23. Read multiturn encoder offset command (0x62)
// 24. Write multiturn encoder values to ROM as motor zero command (0x63)
// 25. Write multiturn encoder current position to ROM as motor zero command (0x64)

// 26. Read encoder data command (0x90)
void SingleLegCAN::readEncoder(int motorID) {
    u_int8_t data[8] = {0X90, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00};
    canSend(data, motorID);
    canRead();
    if (motorID == mMotorKneeID) {
        mEncoderPast1 = mEncoder_temp1;
        mEncoder_temp1 = mFrame.data[2] + mFrame.data[3] * 256;
        mEncoderRaw1 = mFrame.data[4] + mFrame.data[5] * 256;
        mEncoderOffset1 = mFrame.data[6] + mFrame.data[7] * 256;
        if ((mEncoder_temp1 < 10000) && (mEncoderPast1 > 50000)) {
            mEncoderMultiturnNum1 += 1;
        } else if ((mEncoder_temp1 > 50000) && (mEncoderPast1 < 10000)) {
            mEncoderMultiturnNum1 -= 1;
        } else {
            mEncoder1 = mEncoder_temp1 + 65535 * mEncoderMultiturnNum1;
            mAngularPosition_rad1 = mEncoder1 * enc2rad / mGearRatio;
        }
    } else if (motorID == mMotorHipID) {
        mEncoderPast2 = mEncoder_temp2;
        mEncoder_temp2 = mFrame.data[2] + mFrame.data[3] * 256;
        mEncoderRaw2 = mFrame.data[4] + mFrame.data[5] * 256;
        mEncoderOffset2 = mFrame.data[6] + mFrame.data[7] * 256;
        if ((mEncoder_temp2 < 10000) && (mEncoderPast2 > 50000)) {
            mEncoderMultiturnNum2 += 1;
        } else if ((mEncoder_temp2 > 50000) && (mEncoderPast2 < 10000)) {
            mEncoderMultiturnNum2 -= 1;
        } else {
            mEncoder2 = mEncoder_temp2 + 65535 * mEncoderMultiturnNum2;
            mAngularPosition_rad2 = mEncoder2 * enc2rad / mGearRatio;
        }
    }


}

// 27. Write encoder values to ROM as motor zero command (0x91)
// 28. Write current position to ROM as motor zero command (0x19)

// 29. Read multiturn turns angle command (0x92)
void SingleLegCAN::readMultiturnAngularPosition(int motorID) {
    u_int8_t data[8] = {0X92, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00};
    canSend(data, motorID);
    canRead();
    double degree = (mFrame.data[1] + mFrame.data[2] * 256.0 + mFrame.data[3] * 256.0 * 256.0 +
                     mFrame.data[4] * 256.0 * 256.0 * 256.0 +
                     mFrame.data[5] * 256.0 * 256.0 * 256.0 * 256.0 +
                     mFrame.data[6] * 256.0 * 256.0 * 256.0 * 256.0 * 256.0) * 0.01 / mGearRatio;
    if (mFrame.data[7] == 1) {
        degree = -1.0 * degree;
    }
    if (motorID == mMotorKneeID) {
        mAngularPosition_rad1 = degree * deg2rad;
    } else if (motorID == mMotorHipID) {
        mAngularPosition_rad2 = degree * deg2rad;
    }


}

// 30. Read single circle ange command (0x94)
// 31. Read motor status 1 and error flag commands (0x9A)

// 32. Read motor status 2 (0x9C)
void SingleLegCAN::readMotorStatus2(int motorID) {
    u_int8_t data[8] = {0X9c, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00};
    canSend(data, motorID);
    canRead();
    if (motorID == mMotorKneeID) {
        mAngularPosition_rad1 = (mFrame.data[6] + mFrame.data[7] * 256) * enc2rad;
        mAngularVelocity_rad1 = mFrame.data[4] + mFrame.data[5] * 256;
        if (mFrame.data[5] > 127) {
            mAngularVelocity_rad1 -= 256 * 256;
        }
        mAngularVelocity_rad1 = mAngularVelocity_rad1 * deg2rad / mGearRatio;
    } else if (motorID == mMotorHipID) {
        mAngularPosition_rad2 = (mFrame.data[6] + mFrame.data[7] * 256) * enc2rad;
        mAngularVelocity_rad2 = mFrame.data[4] + mFrame.data[5] * 256;
        if (mFrame.data[5] > 127) {
            mAngularVelocity_rad2 -= 256 * 256;
        }
        mAngularVelocity_rad2 = mAngularVelocity_rad2 * deg2rad / mGearRatio;
    }

}
// 33. Read motor status 3 (0x9D)

// 34. Motor off command (0x80)
void SingleLegCAN::turnOffMotor(int motorID) {
    u_int8_t data[8] = {0x80, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00};
    canSend(data, motorID);
}

// 35. Motor stop command (0x81)
void SingleLegCAN::stopMotor(int motorID) {
    u_int8_t data[8] = {0x81, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00};
    canSend(data, motorID);
}

// 36. Motor running command (0x88)
void SingleLegCAN::turnOnMotor(int motorID) {
    u_int8_t data[8] = {0x88, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00};
    canSend(data, motorID);
    sleep(5);
    std::cout << "motor " << motorID << " is turned on" << std::endl;
}

// 37. Torque closed-loop command (0xA1)
// torque_int : 0 ~ 4096 which matches to (-32A ~ 32A)
void SingleLegCAN::setTorque(int motorID, double torque) {
    torque = -torque;
    int torque_int;
    if(motorID == mMotorHipID)
    {
        torque_int = round(30.62201 * torque);
    }
    else if(motorID == mMotorKneeID)
    {
        torque_int = round(24.5737034 * torque);
    }

    if (torque_int < 0) {
        torque_int += +2 * pow(2, 15);
    }

    u_int8_t torqueLowerData = torque_int % 256;
    torque_int = torque_int / 256;
    u_int8_t torqueUpperData = torque_int % 256;
    u_int8_t data[8] = {0Xa1, 0X00, 0X00, 0X00, torqueLowerData, torqueUpperData, 0X00, 0X00};
    canSend(data, motorID);
    canRead();

    if (motorID == mMotorKneeID) {
        mAngularVelocity_rad1 = mFrame.data[4] + mFrame.data[5] * 256;

        if (mFrame.data[5] > 127) {
            mAngularVelocity_rad1 -= 256 * 256;
        }
        mAngularVelocity_rad1 = mAngularVelocity_rad1 * deg2rad / mGearRatio;

        mEncoderPast1 = mEncoder_temp1;
        mEncoder_temp1 = mFrame.data[6] + mFrame.data[7] * 256;
        if ((mEncoder_temp1 < 10000) && (mEncoderPast1 > 50000)) {
            mEncoderMultiturnNum1 += 1;
        } else if ((mEncoder_temp1 > 50000) && (mEncoderPast1 < 10000)) {
            mEncoderMultiturnNum1 -= 1;
        } else {
            mEncoder1 = mEncoder_temp1 + 65535 * mEncoderMultiturnNum1;
            mAngularPosition_rad1 = mEncoder1 * enc2rad / mGearRatio;
        }
    } else if (motorID == mMotorHipID) {
        mAngularVelocity_rad2 = mFrame.data[4] + mFrame.data[5] * 256;

        if (mFrame.data[5] > 127) {
            mAngularVelocity_rad2 -= 256 * 256;
        }
        mAngularVelocity_rad2 = mAngularVelocity_rad2 * deg2rad / mGearRatio;

        mEncoderPast2 = mEncoder_temp2;
        mEncoder_temp2 = mFrame.data[6] + mFrame.data[7] * 256;
        if ((mEncoder_temp2 < 10000) && (mEncoderPast2 > 50000)) {
            mEncoderMultiturnNum2 += 1;
        } else if ((mEncoder_temp2 > 50000) && (mEncoderPast2 < 10000)) {
            mEncoderMultiturnNum2 -= 1;
        } else {
            mEncoder2 = mEncoder_temp2 + 65535 * mEncoderMultiturnNum2;
            mAngularPosition_rad2 = mEncoder2 * enc2rad / mGearRatio;
        }
    }
}

// 38. Speed closed-loop command (0xA2)
void SingleLegCAN::setVelocity(int motorID, int velocity) {
    //0.01dps/LSB
    u_int8_t data[8] = {0Xa2, 0X00, 0X00, 0X00, 0x00, velocity, 0X00, 0X00};
    canSend(data, motorID);
}

// 39. Position closed-loop command 1 (0xA3)
void SingleLegCAN::setPosition1(int motorID, double desiredPosition_rad) {
    double position_Deg = desiredPosition_rad * rad2deg * 100 * mGearRatio;
    int position_int = (int) round(position_Deg);

    u_int8_t pos0;
    u_int8_t pos1;
    u_int8_t pos2;
    u_int8_t pos3;

    int temp = position_int;
    pos0 = temp % 256;
    temp = temp / 256;
    pos1 = temp % 256;
    temp = temp / 256;
    pos2 = temp % 256;
    temp = temp / 256;
    pos3 = temp % 256;

    u_int8_t data[8] = {0Xa3, 0X00, 0X00, 0X00, pos0, pos1, pos2, pos3};
    canSend(data, motorID);
    canRead();
}
// 40. Position closed-loop command 2 (0xA4)
// 41. Position closed-loop command 3 (0xA5)
// 42. Position closed-loop command 4 (0xA6)
// 43. Multiturn incremental position control command (0xA7)
// 44. Multiturn incremental position control command (0xA8)
// 45. read running mode (0x70)
// 46. read power value (0x71)
// 47. read Battery voltage (0x72)
// 48. TF command (0x73)
// 49. System reset command (0x76)
// 50. Brake opening command (0x77)
// 51. Brake close command (0x78)
// 52. CAN ID setting and reading (0x79)
