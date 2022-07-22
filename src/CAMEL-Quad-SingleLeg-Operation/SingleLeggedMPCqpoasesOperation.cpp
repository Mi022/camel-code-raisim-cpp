//
// Created by hs on 22. 6. 20.
//

#include "SingleLeggedMPCqpoasesOperation.h"

void SingleLeggedMPCqpoases::doControl() {
    updateState();
    setTrajectory();
    getMatrices();
    qpSolver();
    computeControlInput();
    setControlInput();
}

void SingleLeggedMPCqpoases::setTrajectory() {
//    std::cout << "Set trajectory function" << std::endl;
    if(mIsZeroing)
    {
        if(*mCurrentTime < mHaltTime)
        {
            for(int i = 0; i < mMPCHorizon ; i++)
            {
                xd(i*2,0) = mTrajectoryGenerator.getPositionTrajectory(*mCurrentTime + mDT * i);
                xd(i*2+1,0) = mTrajectoryGenerator.getVelocityTrajectory(*mCurrentTime + mDT * i);
                xd(i*3+2,0) = 0.f;
            }
            std::cout<<"halt time: "<<mHaltTime<<std::endl;
        }
        else
        {
            mIsZeroing = false;
            setPointTrajectoryZeroing();
        }
    }
    else if(mIsCubic)
    {
        if(*mCurrentTime < mHaltTime)
        {
            for(int i = 0; i < mMPCHorizon ; i++)
            {
                xd(i*2,0) = mTrajectoryGenerator.getPositionTrajectory(*mCurrentTime + mDT * i);
                xd(i*2+1,0) = mTrajectoryGenerator.getVelocityTrajectory(*mCurrentTime + mDT * i);
                xd(i*3+2,0) = 0.f;
            }
//            std::cout<<"halt time: "<<mHaltTime<<std::endl;
        }
        else
        {
            mIsCubic = false;
            setPointTrajectory(desiredPosition);
        }
    }
    else if(mIsSin)
    {

    }

    desiredPosition = xd(0,0);
    desiredVelocity = xd(1,0);
}

void SingleLeggedMPCqpoases::setPointTrajectory(double goalPosition){
    for(int i = 0; i < mMPCHorizon ; i++)
    {
        xd(i*2,0) = goalPosition;
        xd(i*2+1,0) = 0.f;
        xd(i*3+2,0) = 0.f;
    }
    desiredPosition = xd(0,0);
    desiredVelocity = xd(1,0);
}

void SingleLeggedMPCqpoases::zeroing() {
    mIsZeroing = true;
    mIsCubic = false;
    mIsSin = false;
    updateState();
    double timeDuration = 3.0;
    mTrajectoryGenerator.updateTrajectory(position[0], 0.325269119, *mCurrentTime, timeDuration);
    mHaltTime = *mCurrentTime + timeDuration;
}

void SingleLeggedMPCqpoases::setPointTrajectoryZeroing() {
    for(int i = 0; i < mMPCHorizon ; i++)
    {
        xd(i*2,0) = 0.325269119;
        xd(i*2+1,0) = 0.f;
        xd(i*3+2,0) = 0.f;
    }
    desiredPosition = xd(0,0);
    desiredVelocity = xd(1,0);
}

void SingleLeggedMPCqpoases::updateCubicTrajectory(double goalPosition, double timeDuration) {
    mIsZeroing = false;
    mIsCubic = true;
    mIsSin = false;
    mTrajectoryGenerator.updateTrajectory(desiredPosition, goalPosition, *mCurrentTime, timeDuration);
    mHaltTime = *mCurrentTime + timeDuration;
}

void SingleLeggedMPCqpoases::getMatrices(){
    Eigen::Matrix<double,4,4> ABc;
    Eigen::Matrix<double,3,3> Adt;
    Eigen::Matrix<double,3,1> Bdt;

    A.setZero();
    A(0,1) = 1.f;
    A(1,2) = 1.f;
    B.setZero();
    B(1,0) = 1/mLumpedMass;
    B(2,0) = 0.f;
    ABc.setZero();

    ABc.block(0,0,3,3) = A;
    ABc.block(0,3,3,1) = B;
    ABc = ABc*mDT;
    ABc = ABc.exp(); // y'=My => y(t)=exp(M)y(0)
    Adt = ABc.block(0,0,3,3);
    Bdt = ABc.block(0,3,3,1);

    Eigen::Matrix<double,3,3> D[10];
    D[0].setIdentity();
    for(int i=1;i<mMPCHorizon+1; i++)
    {
        D[i] = Adt*D[i-1];
    }
    for(int r=0; r<mMPCHorizon; r++) {
        Aqp.block(3*r, 0, 3, 3) = D[r+1];
        for (int c = 0; c < mMPCHorizon; c++) {
            if (r >= c) {
                int a_num = r - c;
                Bqp.block(3*r, 1*c, 3, 1) = D[a_num] * Bdt;
            }
        }
    }
    K.setIdentity();
    K *= 1e-6;

    float tempWeight[2] = {20.0, 1};
    Eigen::Matrix<double, 3,1> weightMat;
    for(int i=0; i<2;i++)
    {
        weightMat(i) = tempWeight[i];
    }
    weightMat(2) = 0.f;
    L.setZero();
    L.diagonal() = weightMat.replicate(mMPCHorizon,1);

    H = 2*(Bqp.transpose()*L*Bqp + K);
    g = 2*Bqp.transpose()*L*(Aqp*x0 - xd);
}


void SingleLeggedMPCqpoases::qpSolver()
{
    qpOASES::real_t* H_qpoases;
    qpOASES::real_t* g_qpoases;
    qpOASES::real_t* A_qpoases;
    qpOASES::real_t* ub_qpoases;
    qpOASES::real_t* lb_qpoases;
    qpOASES::real_t* q_red;

    H_qpoases = (qpOASES::real_t*)malloc(mMPCHorizon*mMPCHorizon*sizeof(qpOASES::real_t));
    g_qpoases = (qpOASES::real_t*)malloc(mMPCHorizon*sizeof(qpOASES::real_t));
    A_qpoases = (qpOASES::real_t*)malloc(mMPCHorizon*mMPCHorizon*sizeof(qpOASES::real_t));
    ub_qpoases = (qpOASES::real_t*)malloc(mMPCHorizon*sizeof(qpOASES::real_t));
    lb_qpoases = (qpOASES::real_t*)malloc(mMPCHorizon*sizeof(qpOASES::real_t));
    q_red = (qpOASES::real_t*)malloc(mMPCHorizon*sizeof(qpOASES::real_t));

    matrix_to_real(H_qpoases, H, mMPCHorizon, mMPCHorizon);
    matrix_to_real(g_qpoases, g, mMPCHorizon, 1);

    for(int i=0; i<mMPCHorizon; i++)
    {
        for(int j=0; j<mMPCHorizon; j++)
        {
            A_qpoases[i+j] = 1.f;
        }
    }

    for(int i =0; i<mMPCHorizon; i++)
    {
        lb_qpoases[i] = 0.f;
        ub_qpoases[i] = 40.f;
    }

    qpOASES::int_t nWSR = 10000;
    qpOASES::QProblem problem(mMPCHorizon, 0);
    qpOASES::Options op;
    op.setToMPC();
    op.printLevel = qpOASES::PL_NONE;
    problem.setOptions(op);
    problem.init(H_qpoases,g_qpoases,NULL,NULL,NULL,NULL,NULL, nWSR);

    int rval = problem.getPrimalSolution(q_red);
    if(rval != qpOASES::SUCCESSFUL_RETURN)
        printf("failed to solve!\n");


    calculatedForce = q_red[0];

    free(H_qpoases);
    free(g_qpoases);
    free(A_qpoases);
    free(ub_qpoases);
    free(lb_qpoases);
    free(q_red);
}

void SingleLeggedMPCqpoases::matrix_to_real(qpOASES::real_t* dst, Eigen::Matrix<double,Dynamic,Dynamic> src, int16_t rows, int16_t cols)
{
    int32_t a = 0;
    for(int16_t r = 0; r < rows; r++)
    {
        for(int16_t c = 0; c < cols; c++)
        {
            dst[a] = src(r,c);
            a++;
        }
    }
}

void SingleLeggedMPCqpoases::updateState(){
    position = mRobot->getQ();
    velocity = mRobot->getQD();
    x0 << position[0], velocity[0], mGravity;
    mForce(0) = mInitialForce;
}

void SingleLeggedMPCqpoases::computeControlInput() {
//    calculatedForce = mForce(0);
    dz_dth1 = -0.23*sin(position[1]) - 0.23*sin(position[1] + position[2]);
    dz_dth2 = -0.23*sin(position[1] + position[2]);
    torque[0] = dz_dth1 * calculatedForce;
    torque[1] = dz_dth2 * calculatedForce;
}

void SingleLeggedMPCqpoases::setControlInput() {
    for (int i = 0; i < 2; i++) {
        if(torque[i] > mTorqueLimit)
        {
            torque[i] = mTorqueLimit;
        }
        else if(torque[i] < -mTorqueLimit)
        {
            torque[i] = -mTorqueLimit;
        }
    }
    mRobot->setTorque(torque);
}
