//
// Created by ljm on 22. 7. 25.
//

#ifndef RAISIM_A1JOYSTICK_H
#define RAISIM_A1JOYSTICK_H

#include <iostream>
#include <iomanip>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

#define JOY_DEV "/dev/input/js0"

class A1JoyStick {
public:
    int joy_fd;
    int num_of_axis;
    int num_of_buttons;
    char name_of_joystick[80];
    std::vector<char> joy_button;
    std::vector<int> joy_axis;

    js_event js;

    A1JoyStick() {
        joy_fd=-1;
        num_of_axis=0;
        num_of_buttons=0;
        name_of_joystick[80];

        joySetup();
    }

    void joySetup();
    void joyRead();
    void joyButton();

private:

};

#endif //RAISIM_A1JOYSTICK_H
