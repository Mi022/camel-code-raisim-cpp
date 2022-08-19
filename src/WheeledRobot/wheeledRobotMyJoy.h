//
// Created by ljm on 22. 8. 2.
//

#ifndef RAISIM_WHEELEDROBOTMYJOY_H
#define RAISIM_WHEELEDROBOTMYJOY_H

#include <iostream>
#include <vector>
#include <fcntl.h>
#include <unistd.h>

#define JOY_MY "/dev/ttyACM0"

class wheeledRobotMyJoyStick {
public:
    int joy_fd;
    char read_byte=0;
    bool joyAvailable;
    std::vector<char> joy_button;

    wheeledRobotMyJoyStick() {
        joy_fd=-1;
        joyAvailable = myJoySetup();
    }

    bool myJoySetup();
    int myJoyRead();

private:

};

#endif //RAISIM_WHEELEDROBOTMYJOY_H
