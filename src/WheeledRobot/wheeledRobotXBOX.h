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

class wheeledRobotXBOX {
public:
    int joy_fd=-1;
    int num_of_axis=0;
    int num_of_buttons=0;
    char name_of_joystick[80];
    std::vector<char> joy_button;
    std::vector<int> joy_axis;

    js_event js;

    int joySetup();
    void joyRead();

private:
};

#endif //RAISIM_WHEELEDROBOTXBOX_H
