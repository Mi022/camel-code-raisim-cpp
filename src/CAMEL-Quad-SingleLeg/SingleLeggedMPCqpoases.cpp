//
// Created by hs on 22. 6. 20.
//

#include "SingleLeggedMPCqpoases.h"

void SingleLeggedMPCqpoases::doControl() {
    std::cout<<"simTime : "<<getRobot()->getWorldTime()<<std::endl;
    updateState();
    setTrajectory();
    getMatrices();
    qpSolver();
    computeControlInput();
    setControlInput();
}

void SingleLeggedMPCqpoases::setTrajectory() {
    std::cout << "Set trajectory function" << std::endl;
    double currentTime = getRobot()->getWorldTime();
    for(int i = 0; i < mMPCHorizon ; i++)
    {
        xd(i*2,0) = mTrajectoryGenerator.getPositionTrajectory(currentTime + mDT * i);
        xd(i*2+1,0) = mTrajectoryGenerator.getVelocityTrajectory(currentTime + mDT * i);
        xd(i*3+2,0) = 0.f;
/*        xd(i*3,0) = 0.23;
        xd(i*3+1,0) = 0.f;
        xd(i*3+2,0) = mGravity;*/
    }
    desiredPosition = xd(0,0);
    desiredVelocity = xd(1,0);
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

    float tempWeight[2] = {4.0, 0.4};
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

    //std::cout << "Aqp" << std::endl;
    //std::cout << Aqp << std::endl << std::endl;
    //std::cout << "Bqp" << std::endl;
    //std::cout << Bqp << std::endl << std::endl;
    std::cout << "x0" << std::endl;
    std::cout << x0 << std::endl << std::endl;
    //std::cout << "xd" << std::endl;
    //std::cout << xd << std::endl << std::endl;
    //std::cout << "H" << std::endl;
    //std::cout << H << std::endl << std::endl;
    //std::cout << "g" << std::endl;
    //std::cout << g << std::endl << std::endl;
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

/*    std::cout << "q_red" << std::endl;
    std::cout << q_red[0] << std::endl;
    std::cout << q_red[1] << std::endl;
    std::cout << q_red[2] << std::endl;*/

    calculatedForce = q_red[0];
    std::cout << calculatedForce << std::endl;

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
    position = getRobot()->robot->getGeneralizedCoordinate();
    velocity = getRobot()->robot->getGeneralizedVelocity();
    x0 << position[0], velocity[0], mGravity;
    mForce(0) = mInitialForce;
}

void SingleLeggedMPCqpoases::computeControlInput() {
//    calculatedForce = mForce(0);
    dz_dth1 = -0.23*sin(position[1]) - 0.23*sin(position[1] + position[2]);
    dz_dth2 = -0.23*sin(position[1] + position[2]);
    torque[1] = dz_dth1 * calculatedForce;
    torque[2] = dz_dth2 * calculatedForce;
}

void SingleLeggedMPCqpoases::setControlInput() {
    for (int i = 0; i < 3; i++) {
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
