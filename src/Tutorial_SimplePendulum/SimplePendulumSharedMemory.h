//
// Created by jaehoon on 22. 7. 19.
//

#ifndef RAISIM_SIMPLEPENDULUMSHAREDMEMORY_H
#define RAISIM_SIMPLEPENDULUMSHAREDMEMORY_H

typedef struct _SHM_
{
    double simTime;
    double jointPosition;
    double desiredJointPosition;
    double jointVelocity;
    double desiredJointVelocity;
    double jointTorque;
}SHM, *pSHM;

#endif //RAISIM_SIMPLEPENDULUMSHAREDMEMORY_H
