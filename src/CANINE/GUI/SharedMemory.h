//
// Created by hs on 22. 7. 21.
//

#ifndef RAISIM_SHAREDMEMORY_H
#define RAISIM_SHAREDMEMORY_H

typedef struct _SHM_
{
    double simTime;
    double GetPosX;
    double GetPosY;
    double GetPosZ;
    double DesPosX;
    double DesPosY;
    double DesPosZ;

    double GetRotX;
    double GetRotY;
    double GetRotZ;
    double DesRotX;
    double DesRotY;
    double DesRotZ;


    double FR_hipJoint;
    double FL_hipJoint;
    double RR_hipJoint;
    double RL_hipJoint;

    double FR_thightJoint;
    double FL_thightJoint;
    double RR_thightJoint;
    double RL_thightJoint;

    double FR_calfJoint;
    double FL_calfJoint;
    double RR_calfJoint;
    double RL_calfJoint;
}SHM, *pSHM;

#endif //RAISIM_SHAREDMEMORY_H
