#ifndef RAISIM_COMMAND_H
#define RAISIM_COMMAND_H

#include "../Robot/RobotDescription.h"
#include "../Utils/SharedMemory.h"
#include "../Utils/MotorCAN.h"

class Command {
public:
    Command(MotorCAN *can)
    {
        mCan = can;
    }
    void commandFunction();

private:
    MotorCAN *mCan;
    void visualOn();
};


#endif //RAISIM_COMMAND_H
