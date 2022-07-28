//
// Created by jaehoon on 22. 4. 20.
//
# include "include/CAN/CanMotorX8ProV2.h"

std::string canName_temp = "can8";
std::string bitRate = "1000000";
char *canName = "can8";
int motor1ID = 0x141;
CanMotorX8ProV2 canX8pro(canName, canName_temp, bitRate);


int main() {
    if (canX8pro.getSock() < 0) { return -1; }
    canX8pro.turnOnMotor(motor1ID);

    double torque = -7.8;
    canX8pro.setTorque(motor1ID, torque);

    sleep(5);


    canX8pro.turnOffMotor(motor1ID);
//    canX8pro.stopMotor(motor1ID);

    return 0;
}