//
// Created by jaehoon on 22. 7. 19.
//

#ifndef RAISIM_SINGLELEGGEDSHAREDMEMORY_H
#define RAISIM_SINGLELEGGEDSHAREDMEMORY_H

typedef struct _SHM_
{
    double time;
    double position[2];
    double desiredPosition[2];
    double velocity[2];
    double desiredVelocity_z[2];
    double jointPosition[2];
    double jointVelocity[2];
    double jointTorque[2];
    double GRF;
}SHM, *pSHM;

#endif //RAISIM_SINGLELEGGEDSHAREDMEMORY_H
