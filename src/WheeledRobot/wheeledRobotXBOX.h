//
// Created by ljm on 22. 7. 11.
//

#ifndef RAISIM_WHEELEDROBOTXBOX_H
#define RAISIM_WHEELEDROBOTXBOX_H

#include <iostream>
#include <iomanip>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

#define JOY_DEV "/dev/input/js0"

class wheeledRobotJoyStick {
public:
    int joy_fd;
    int num_of_axis;
    int num_of_buttons;
    char name_of_joystick[80];
    bool joyAvailable;
    std::vector<char> joy_button;
    std::vector<int> joy_axis;

    js_event js;

    wheeledRobotJoyStick() {
        joy_fd=-1;
        num_of_axis=0;
        num_of_buttons=0;
        name_of_joystick[80];

        joyAvailable = joySetup();
    }

    bool joySetup();
    void joyRead();
    void joyButton();

private:

};

#endif //RAISIM_WHEELEDROBOTXBOX_H
