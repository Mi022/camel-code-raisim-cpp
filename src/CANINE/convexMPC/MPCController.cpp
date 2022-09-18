//
// Created by hs on 22. 6. 27.
//

#include "MPCController.h"

MPCController::MPCController(Robot *robot, double dT):
Controller(robot),
mDT(dT), mMPCHorizon(5),
stand(mMPCHorizon, Vec4<int>(100,100,100,100), Vec4<int>(100,100,100,100), 100),
trot(mMPCHorizon, Vec4<int>(0,50,50,0), Vec4<int>(50,50,50,50), 100),
pace(mMPCHorizon, Vec4<int>(50,0,50,0), Vec4<int>(50,50,50,50), 100),
bound(mMPCHorizon, Vec4<int>(0,0,50,50), Vec4<int>(50,50,50,50), 100)
{
    currentGait = &stand;
    currentGaitName = GaitType::STAND;

    bdyInertia = getRobot()->robot->getInertia()[0];
    cmpcSolver.matrixinitialize(bdyInertia);

    weightMat << 0.5, 0.5, 50, 20, 20, 80, 0, 0, 0.2, 0.05, 0.05, 0.05, 0.f;
    cmpcSolver.setParameters(mMPCHorizon, mDT);
    cmpcSolver.setWeights(weightMat, alpha);
    cmpcSolver.resizeMatrix();

    legGenerator.updateTrajectory(getRobot()->getWorldTime(), 0.25);

    initialize();
}
void MPCController::setTrajectory(){}

void MPCController::initialize(){
    position.setZero();
    velocity.setZero();
    torque.setZero();
    Legtemptorque.setZero();
    f->setZero();
    robotJacobian->setZero();
    robottorque->setZero();
    footPosition->setZero();
    iteration = 0;
}

void MPCController::doControl() {
    //std::cout<<"simTime : "<<getRobot()->getWorldTime()<<std::endl;

    currentGait->setIterations(iteration);
    mpcTable = currentGait->getGaitTable();

    updateState();
    cmpcSolver.setTrajectory(getRobot()->getWorldTime(),currentGaitName);
    cmpcSolver.getMetrices(mpcTable, position, velocity, footPosition);
    cmpcSolver.qpSolver();
    cmpcSolver.getGRF(f);
    setLegcontrol();
    computeControlInput();
    setControlInput();
    iteration++;
}

void MPCController::updateState(){
    position = getRobot()->robot->getGeneralizedCoordinate();
    velocity = getRobot()->robot->getGeneralizedVelocity();

    auto FRfootFrameIndex = getRobot()->robot->getFrameIdxByName("RF_FOOT");
    auto FLfootFrameIndex = getRobot()->robot->getFrameIdxByName("LF_FOOT");
    auto RRfootFrameIndex = getRobot()->robot->getFrameIdxByName("RH_FOOT");
    auto RLfootFrameIndex = getRobot()->robot->getFrameIdxByName("LH_FOOT");

    //Get foot position on the world frame
    getRobot()->robot->getFramePosition(FRfootFrameIndex, footPosition[0]);
    getRobot()->robot->getFramePosition(FLfootFrameIndex, footPosition[1]);
    getRobot()->robot->getFramePosition(RRfootFrameIndex, footPosition[2]);
    getRobot()->robot->getFramePosition(RLfootFrameIndex, footPosition[3]);
}

void MPCController::setLegcontrol() {
    double currentTime = getRobot()->getWorldTime();
    legGenerator.getPositionTrajectory(currentTime + mDT);
    desiredPosition[0] = legGenerator.sumX;
    desiredPosition[1] = legGenerator.sumZ;

    double d = sqrt(pow(desiredPosition[0],2)+pow(desiredPosition[1],2));
    double phi = acos(abs(desiredPosition[0])/ d);
    double psi = acos(pow(d,2)/(2*0.23*d));

    double jointPos[3];
    double jointVel[3];

    jointPos[0] = 0.f;
    if (desiredPosition[0] < 0)
        jointPos[1] = 1.57 - phi + psi;
    else if(desiredPosition[0] == 0)
        jointPos[1] = psi;
    else
        jointPos[1] = phi + psi - 1.57;
    jointPos[2] = -acos((pow(d,2)-2*pow(0.23,2)) / (2*0.23*0.23));


    jointVel[0] = 0.f;
    jointVel[1] = 0.f;
    jointVel[2] = 0.f;

    double Pgain[3];
    double Dgain[3];
    Pgain[0] = 5;
    Dgain[0] = 0.5;

    Pgain[1] = 20;
    Dgain[1] = 1;

    Pgain[2] = 30;
    Dgain[2] = 1;

    double posError[3];
    double velError[3];
    for (int i = 0; i < 4; i++){
        if (mpcTable[i] == 0){
            for(int j=0; j<3; j++)
            {
                posError[j] = jointPos[j] - position[7+i*3+j];
                velError[j] = jointVel[j] - velocity[6+i*3+j];
                Legtemptorque[i*3+j] = Pgain[j] * posError[j] + Dgain[j] * velError[j];
            }
        }
        else
        {
            for(int j=0; j<3; j++)
            {
                Legtemptorque[i*3+j] = 0.f;
            }
        }
    }
}

void MPCController::computeControlInput() {
    cmpcSolver.getJacobian(robotJacobian[0], position[ 7],position[ 8],position[ 9],1);
    cmpcSolver.getJacobian(robotJacobian[1], position[10],position[11],position[12],-1);
    cmpcSolver.getJacobian(robotJacobian[2], position[13],position[14],position[15],1);
    cmpcSolver.getJacobian(robotJacobian[3], position[16],position[17],position[18],-1);

    for(int i=0; i<4; i++)
    {
        robotJacobian[i].transposeInPlace();
        robottorque[i] = robotJacobian[i]*f[i];
        torque[i*3+6] = robottorque[i][0] + Legtemptorque[i*3];
        torque[i*3+7] = robottorque[i][1] + Legtemptorque[i*3+1];
        torque[i*3+8] = robottorque[i][2] + Legtemptorque[i*3+2];
    }
}

void MPCController::setControlInput() {
    for (int i = 6; i < 18; i++) {
        if(torque[i] > torqueLimit)
        {
            torque[i] = torqueLimit;
        }
        else if(torque[i] < -torqueLimit)
        {
            torque[i] = -torqueLimit;
        }
    }
    getRobot()->robot->setGeneralizedForce(torque);
}

void MPCController::setGait(int index) {
    switch (index) {
    case 0:
        currentGait = &stand;
        currentGaitName = GaitType::STAND;
        cmpcSolver.stopPosX = cmpcSolver.p[0];
        break;
    case 1:
        currentGait = &trot;
        currentGaitName = GaitType::TROT;
        break;
    case 2:
        currentGait = &pace;
        currentGaitName = GaitType::PACE;
        break;
    case 3:
        currentGait = &bound;
        currentGaitName = GaitType::BOUND;
        break;
    }
}

void MPCController::resetParam(int hor, double dt) {
    mMPCHorizon = hor;
    mDT = dt;
    cmpcSolver.setParameters(mMPCHorizon, mDT);
}

void MPCController::resetWeight(Vec13<double> w, double a) {
    weightMat = w;
    alpha = a;
    cmpcSolver.setWeights(weightMat, alpha);
}
