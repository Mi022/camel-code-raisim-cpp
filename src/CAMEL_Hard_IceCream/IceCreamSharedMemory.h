//
// Created by hwayoung on 22. 8. 29.
//

#ifndef RAISIM_ICECREAMSHAREDMEMORY_H
#define RAISIM_ICECREAMSHAREDMEMORY_H

typedef struct _SHM_
{
    double simTime;
    double plotW1B, plotW1R, plotW2B, plotW2R, plotW3B, plotW3R;
/* not yet */
//    int robotDim;
//    double deg2rad = 3.141592/180.0;
}SHM, *pSHM;

#endif //RAISIM_ICECREAMSHAREDMEMORY_H
