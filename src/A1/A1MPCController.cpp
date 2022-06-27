//
// Created by hs on 22. 6. 27.
//

#include "A1MPCController.h"

void A1MPCController::doControl() {
    std::cout<<"simTime : "<<getRobot()->getWorldTime()<<std::endl;
    updateState();
    setTrajectory();
    getMetrices();
    computeControlInput();
    setControlInput();
}

void A1MPCController::updateState(){
    position = getRobot()->robot->getGeneralizedCoordinate();
    velocity = getRobot()->robot->getGeneralizedVelocity();

    p << position[0], position[1], position[2];
    quat << position[3], position[4], position[5], position[6];
    v << velocity[0], velocity[1], velocity[2];
    w << velocity[3], velocity[4], velocity[5];

    quat_to_euler(quat, q);

    x0 << q[0], q[1], q[2],
          p[0], p[1], p[2],
          w[0], w[1], w[2],
          v[0], v[1], v[2],
          mGravity;
    std::cout << x0 << std::endl;
}

void A1MPCController::setTrajectory() {
    std::cout << "Set trajectory function" << std::endl;
    for(int i = 0; i < mMPCHorizon ; i++)
    {
        xd << 0.f, 0.f, 0.f,
              0.f, -1.5, 0.3590,
              0.f, 0.f, 0.f,
              0.f, 0.f, 0.f,
              mGravity;
    }
}

void A1MPCController::getMetrices(){
    Eigen::Matrix<double,13,13> Ac;
    Eigen::Matrix<double,13,12> Bc;

    ss_mats(Ac, Bc);
    c2qp(Ac,Bc);
}

void A1MPCController::computeControlInput() {}

void A1MPCController::setControlInput() {}

//=============================================================
template <class T>
T t_min(T a, T b)
{
    if(a<b) return a;
    return b;
}

template <class T>
T sq(T a)
{
    return a*a;
}

inline Eigen::Matrix<double,3,3> getSkew(Eigen::Matrix<double,3,1> r)
{
    Eigen::Matrix3d cm;
    cm << 0.f, -r(2), r(1),
            r(2), 0.f, -r(0),
            -r(1), r(0), 0.f;
    return cm;
}

void A1MPCController::quat_to_euler(Eigen::Matrix<double,4,1>& quat, Eigen::Matrix<double,3,1>& q)
{
    //edge case!
    float as = t_min(-2.*(quat[1]*quat[3]-quat[0]*quat[2]),.99999);
    q(2) = atan2(2.f*(quat[1]*quat[2]+quat[0]*quat[3]),sq(quat[0]) + sq(quat[1]) - sq(quat[2]) - sq(quat[3]));
    q(1) = asin(as);
    q(0) = atan2(2.f*(quat[2]*quat[3]+quat[0]*quat[1]),sq(quat[0]) - sq(quat[1]) - sq(quat[2]) + sq(quat[3]));
}

void A1MPCController::ss_mats(Eigen::Matrix<double,13,13>& A, Eigen::Matrix<double,13,12>& B){
    A.setZero();
    A(3,9) = 1.f;
    A(4,10) = 1.f;
    A(5,11) = 1.f;
    A(11,12) = 1.f;

    B.setZero();

    Eigen::Matrix<double,3,3> I_world;
    I_world << 0.01683993, 0, 0,
               0, 0.056579028, 0,
               0, 0, 0.064713601;
    Eigen::Matrix<double,3,3> I_inv = I_world.inverse();

    Eigen::Matrix<double,4,3> R_feet;
    R_feet <<  0.1805,-0.127,-p[2], //FR
               0.1805, 0.127,-p[2], //FL
               -0.1805,-0.127,-p[2], //RR
               -0.1805, 0.127,-p[2]; //RL
    B.setZero();
    for(int n=0; n<4; n++)
    {
        B.block(6,n*3,3,3) = I_inv*getSkew(R_feet.row(n));
        B.block(9,n*3,3,3) = Eigen::Matrix3d::Identity() / mLumpedMass;
    }
}

void A1MPCController::c2qp(Eigen::Matrix<double,13,13> A, Eigen::Matrix<double,13,12> B)
{
    Eigen::Matrix<double,25,25> ABc, expmm;
    Eigen::Matrix<double,13,13> Adt;
    Eigen::Matrix<double,13,12> Bdt;

    ABc.setZero();
    ABc.block(0,0,13,13) = A;
    ABc.block(0,13,13,12) = B;
    ABc = mDT*ABc;
    expmm = ABc.exp();

    Adt = expmm.block(0,0,13,13);
    Bdt = expmm.block(0,13,13,12);

    Eigen::Matrix<double,13,13> D[20];
    D[0].setIdentity();
    for(int i=1; i<=mMPCHorizon; i++)
    {
        D[i] = Adt * D[i-1];
    }

    for(int r=0; r<mMPCHorizon; r++)
    {
        Aqp.block(13*r,0,13,13) = D[r+1];
        for(int c=0; c<mMPCHorizon; c++)
        {
            if(r>=c)
            {
                int a_num = r-c;
                Bqp.block(13*r,12*c,13,12) = D[a_num]*Bdt;
            }
        }
    }
}
//=============================================================