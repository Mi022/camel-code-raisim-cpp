//
// Created by hwayoung on 22. 8. 29.
//

#ifndef RAISIM_DOUBLEBARSHAREDMEMORY_H
#define RAISIM_DOUBLEBARSHAREDMEMORY_H

typedef struct _SHM_
{
    double simTime;
    double jointPosition;
    double desiredJointPosition;
    double jointVelocity;
    double desiredJointVelocity;
    double jointTorque;
}SHM, *pSHM;

#endif //RAISIM_DOUBLEBARSHAREDMEMORY_H
