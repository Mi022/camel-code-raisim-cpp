//
// Created by user on 22. 6. 11.
// this controller is for balancing
// desired states (position1, velocity1, velocity2) are zeros
//

#include "MIPILQRController.h"
using namespace std;

void MIPILQRController::setMatrix() {
    mA <<    1.00165, 0.005, 0.0,
            0.6606, 1.0, 0.0,
            -0.6606, 0.0, 1.0;
    mB <<    0.00718,
            -2.872,
            24.442;
}
void MIPILQRController::setSNGain(double SN11, double SN22, double SN33) {
    mSN<<    SN11, 0.0, 0.0,
            0.0, SN22, 0.0,
            0.0, 0.0, SN33;
}

void MIPILQRController::setQGain(double Q11, double Q22, double Q33) {
    mQ <<    Q11, 0.0, 0.0,
            0.0, Q22, 0.0,
            0.0, 0.0, Q33;
}

void MIPILQRController::setRGain(double R) {
    this -> mR = R;
}

void MIPILQRController::findS() {
    int iteration = 0;
    Eigen::Matrix3d snext = mSN;
    double temp = mB.transpose() * snext * mB + mR;
    mS = mA.transpose() * (snext - snext * mB / temp * mB.transpose() * snext) * mA + mQ;
    while(!IsSEnough(mS, snext)){
         iteration++;
         cout<<"check: "<<iteration<<endl;
         if(iteration > 10000)
         {
             mIsSExist = false;
             break;
         }
         snext = mS;
         mS = mA.transpose() * (snext - snext * mB / (mB.transpose() * snext * mB + mR) * mB.transpose() * snext) * mA + mQ;
         cout<<"check S :\n"<<mS<<endl;
    }
}

bool MIPILQRController::IsSEnough(Eigen::Matrix3d S, Eigen::Matrix3d Snext) {
    bool tempJudge = true;
    for(int r = 0; r<3; r++){
        for(int c = 0; c<3; c++){
            tempJudge = tempJudge && abs(S(r,c) - Snext(r, c)) < 0.001;
        }
    }
    if(tempJudge){
        return true;
    }
    else{
        return false;
    }
}
void MIPILQRController::findK() {
    double temp = mB.transpose()*mS*mB + mR;
    mK = mB.transpose()*mS*mA/temp;
}

void MIPILQRController::generateExternalForce() {
    raisim::Vec<3> externalForce = {0.0, 9.6, 0.0};
    raisim::Vec<3> forcePosition = {0.0, 0.0, 0.08};

    if(mItertaion%FORCE_DURATION == 0 || mItertaion == 0){
        getRobot()->robot->setExternalForce(1, forcePosition, externalForce);
    }
}

void MIPILQRController::addNoise() {
    std::random_device rd;

    // random_device 를 통해 난수 생성 엔진을 초기화 한다.
    std::mt19937 gen(rd());

    // 0 부터 200 까지 균등하게 나타나는 난수열을 생성하기 위해 균등 분포 정의.
    std::uniform_int_distribution<int> dis(-RANDOM_BOUNDARY, RANDOM_BOUNDARY);
    double noisePosition = double(dis(gen)) / RANDOM_BOUNDARY * MAX_POSITION_NOISE; //0.15
    double noiseVelocity = double(dis(gen)) / RANDOM_BOUNDARY * MAX_VELOCITY_NOISE; //1.4
    double noiseMotorVelocity = double(dis(gen)) / RANDOM_BOUNDARY * MAX_MOTOR_VELOCITY_NOISE; //35.5

    mPosition[1] += noisePosition;
    mVelocity[1] += noiseVelocity;
    mVelocity[2] += noiseMotorVelocity;

    mX[0] = mPosition[1] + mEstPlane;
    mX[1] = mVelocity[1];
    mX[2] = mVelocity[2];
}

void MIPILQRController::inclinePDcontrol() {
    //set desiredIncline mPosition

//    setInclineUnitTrajectory(0.05, 0.0);
//    setInclineSinTrajectory(0.05, 10.0);
    setInclineRisingTrajectory(0.05, 10.0);

    double PGain = 200.0;
    double DGain = 40.0;
    double PositionError = mDesiredInclinePosition - mInclineX[0];
    double VelocityError = mDesiredInclineVelocity - mInclineX[1];

    mTorque[0] = PGain*PositionError + DGain*VelocityError;
    mItertaion++;
}

void MIPILQRController::setInclineUnitTrajectory(double desiredPosition, double desiredVelocity) {
    mDesiredInclinePosition = desiredPosition;
    mDesiredInclineVelocity = desiredVelocity;
}

void MIPILQRController::setInclineSinTrajectory(double amplitude, double period) {
    mDesiredInclinePosition = amplitude*sin(robotWorld->getWorldTime()/period*(2*M_PI)) + amplitude;
    mDesiredInclineVelocity = amplitude/period*(2*M_PI)*cos(robotWorld->getWorldTime()/period*(2*M_PI));
}

void MIPILQRController::setInclineRisingTrajectory(double stepSize, double period) {
    int intPeriod = period/mDT;
    if(mItertaion == 0)
    {
        mReference = mInclineX[0] + stepSize/2;
    }
    if((mItertaion%intPeriod) < (intPeriod/2))
    {
        mDesiredInclinePosition = stepSize/2*sin(robotWorld->getWorldTime()/period*(2*M_PI) - M_PI/2) + mReference;
        mDesiredInclineVelocity = stepSize/2/period*(2*M_PI)*cos(robotWorld->getWorldTime()/period*(2*M_PI));
    }else{
        mReference = mInclineX[0] + stepSize/2;
        mDesiredInclineVelocity = 0.0;
    }
}

void MIPILQRController::calEstPlane() {
    double PGain = 100.0;
    double DGain = 40.0;
    mEstPlane = asin((0.001741*(mRodAcc + PGain*mX[0] + DGain*mX[1]))/0.2301) - mPosition[1];
}

bool MIPILQRController::IsTorqueZero() {
    if(abs(mTorque[2]) < 1e-6)
    {
        return true;
    }
    return false;
}

void MIPILQRController::doControl() {
    updateState();
//    addNoise();
    inclinePDcontrol();
//    generateExternalForce();
    if(mIsSExist)
    {
        computeControlInput();
        setControlInput();
    }
}

void MIPILQRController::updateState() {
    double tempVelo = mVelocity[1];
    mPosition = getRobot()->robot->getGeneralizedCoordinate();
    mVelocity = getRobot()->robot->getGeneralizedVelocity();
    mRodAcc = (mVelocity[1] - tempVelo)/0.005;

    //for plane angle
    if(IsTorqueZero())
    {
        calEstPlane();
    }
    mX[0] = mPosition[1] + mEstPlane;
    mX[1] = mVelocity[1];
    mX[2] = mVelocity[2];

    //incline state
    mInclineX[0] = mPosition[0];
    mInclineX[1] = mVelocity[0];
}

void MIPILQRController::computeControlInput() {
    mTorque[2] = -mK*mX;
    if(mTorque[2] > torqueLimit)
    {
        mTorque[2] = torqueLimit;
    }
    else if(mTorque[2] < -torqueLimit)
    {
        mTorque[2] = -torqueLimit;
    }
}

void MIPILQRController::setControlInput() {
    getRobot()->robot->setGeneralizedForce(mTorque);
}

void MIPILQRController::setTrajectory() {

}


const Eigen::VectorXd &MIPILQRController::getTorque() const {
    return mTorque;
}

double MIPILQRController::getDesiredInclinePosition() const {
    return mDesiredInclinePosition;
}

double MIPILQRController::getEstPlane() const {
    return mEstPlane;
}
