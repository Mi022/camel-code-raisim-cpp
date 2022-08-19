//
// Created by jaehyeong on 22. 8. 11.
//

#ifndef RAISIM_JHDOGSHAREDMEMORY_H
#define RAISIM_JHDOGSHAREDMEMORY_H

typedef struct _SHM_
{
    double simTime;
    double jointPosition[3];
    double desiredJointPosition[3];
    double jointVelocity[3];
    double desiredJointVelocity[3];
    double jointTorque[3];
}SHM, *pSHM;
#endif //RAISIM_JHDOGSHAREDMEMORY_H
