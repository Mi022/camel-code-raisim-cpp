//
// Created by hwayoung on 22. 8. 29.
//

#include "IceCreamTestController.h"

void IceCreamTestController::doControl() {
    setTrajectory();
    updateState();
    computeControlInput();
    updateAcc();
    setControlInput();
}

//TODO: deg2rad placed in shared memory

// set desired states of the robot
void IceCreamTestController::setTrajectory() {
//    if(getRobot()->getWorldTime() > mEndTime){
//        mTrajectoryGenerator.updateTrajectory(position[1], -90*getRobot()->deg2rad, getRobot()->getWorldTime(), 1.0);
//        mEndTime = getRobot()->getWorldTime() + 3.0;
//    }
    desiredPosition[1] = mTrajectoryGenerator.getPositionTrajectory(getRobot()->getWorldTime());
    desiredVelocity[1] = mTrajectoryGenerator.getVelocityTrajectory(getRobot()->getWorldTime());
}

// update states of the robot
void IceCreamTestController::updateState() {
    mPreviousVelocity = velocity;
    position = getRobot()->getQ().e();
    velocity = getRobot()->getQD().e();
}

// compute control inputs based on PD control method
void IceCreamTestController::computeControlInput() {
    torque.setZero();
    torque[1] = mPGain*(desiredPosition[1] - position[1]) + mDGain*(desiredVelocity[1] - velocity[1]);
//    torque[1] = -0.1;
}

// set computed force(torque) to the robot
void IceCreamTestController::setControlInput() {
    getRobot()->robot->setGeneralizedForce(torque);
}

void IceCreamTestController::updateAcc() {
    //calculate the acc
    double lda = sqrt(pow(mRodLength*cos(position[0]) - mBodyLength*sin(position[0] + position[1])/2.0, 2.0)
            + pow(mRodLength*sin(position[0]) + mBodyLength*cos(position[0] + position[1])/2.0, 2.0));
    mIBa = parallelAxis(cubiodInertia(mBodyMass, mBodyLength, 0.1), mBodyMass, lda);
    mIICa = mIra + mIBb;

    calculatedAcc[0] = -torque[1]/mIICa;
    calculatedAcc[1] = torque[1]/(mIBb + mIICa);

    measuredAcc[0] = (velocity[0] - mPreviousVelocity[0])/mDT;
    measuredAcc[1] = (velocity[1] - mPreviousVelocity[1])/mDT;
}

double IceCreamTestController::cubiodInertia(double mass, double a, double b){
    return mass*(a*a + b*b)/12.0;
}

double IceCreamTestController::parallelAxis(double inertia, double mass, double length){
    return inertia + mass*length*length;
}