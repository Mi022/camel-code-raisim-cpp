//
// Created by ljm on 22. 8. 2.
//

#include "wheeledRobotMyJoy.h"

bool wheeledRobotMyJoyStick::myJoySetup() {
    if((joy_fd = open(JOY_MY,O_RDONLY)) < 0)
    {
        std::cerr<<"Failed to open "<<JOY_MY<<std::endl;
        return false;
    }

    joy_button.resize(15, 0);
    std::cout << "Open succeeded" << std::endl;

    return true;
}

int wheeledRobotMyJoyStick::myJoyRead() {
    read(joy_fd, &read_byte, sizeof(char));

    switch (read_byte)
    {
        case 'u':
            joy_button[13] = '1';
            return 1;
        case 'd':
            joy_button[14] = '1';
            return 2;
        case 'l':
            joy_button[11] = '1';
            return 3;
        case 'r':
            joy_button[12] = '1';
            return 4;
        case 'x':
            joy_button[6] = '1';
            return 5;
        default:
            joy_button[6] = '\0';
            joy_button[11] = '\0';
            joy_button[12] = '\0';
            joy_button[13] = '\0';
            joy_button[14] = '\0';
            return 0;
    }
}