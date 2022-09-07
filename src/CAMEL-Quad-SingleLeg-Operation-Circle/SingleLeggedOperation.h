//
// Created by jaehoon on 22. 5. 25.
//

#ifndef RAISIM_SINGLELEGGEDOPERATION_H
#define RAISIM_SINGLELEGGEDOPERATION_H

#include "include/CAMEL/Operation.h"
#include "SingleLeggedRobotOperation.h"
#include "SingleLeggedPDControllerOperation.h"
#include "SingleLeggedIDControllerOperation.h"

class SingleLeggedOperation : public Operation {
public:
    SingleLeggedOperation(raisim::World *world, double fps) : Operation(world, fps) { ; }

private:
};


#endif //RAISIM_SINGLELEGGEDOPERATION_H
