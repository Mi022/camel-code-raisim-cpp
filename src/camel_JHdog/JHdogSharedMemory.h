//
// Created by jaehyeong on 22. 8. 11.
//

#ifndef RAISIM_JHDOGSHAREDMEMORY_H
#define RAISIM_JHDOGSHAREDMEMORY_H

typedef struct _SHM_
{
    double simTime;
    double jointPosition;
    double desiredJointPosition;
    double jointVelocity;
    double desiredJointVelocity;
    double jointTorque;
}SHM, *pSHM;
#endif //RAISIM_JHDOGSHAREDMEMORY_H
