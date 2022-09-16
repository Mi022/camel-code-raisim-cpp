//
// Created by jaehoon on 22. 6. 27.
//
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "A1CollisionDetecter.h"

Eigen::Matrix2d MassMat(double q1, double q2);
Eigen::Matrix2d CoriloisMat(double q1, double dq1, double q2, double dq2);
Eigen::Vector2d Beta(double q1, double dq1, double q2, double dq2);

int main(int argc, char* argv[]) {
    double pi = 3.141592;
    double pGain = 10.0;
    double dGain = 1;
    int i =0;
    /// create raisim world
    raisim::World world;
    world.setTimeStep(0.001);

    /// for momentum observer
    Eigen::VectorXd momentum = Eigen::VectorXd(2);
    Eigen::VectorXd momentumPrev = Eigen::VectorXd(2);
    Eigen::VectorXd residual = Eigen::VectorXd(2);
    Eigen::VectorXd beta = Eigen::VectorXd(2);
    Eigen::VectorXd dqMat = Eigen::VectorXd(2);
    Eigen::Matrix2d gainK;
    gainK(0, 0) = 100, gainK(0, 1) = 0;
    gainK(1, 0) = 0, gainK(1, 1) = 100;
    bool firstRun = true;

    /// create objects
    world.addGround();
    auto testrobot = world.addArticulatedSystem("\\home\\cha\\git\\repository-group\\raisimLib\\camel-code-raisim-cpp\\rsc\\test_a1_single_leg_right\\camel_single_leg.urdf");
    Eigen::VectorXd torque(testrobot->getDOF());
    Eigen::VectorXd desiredPosition(testrobot->getDOF());
    Eigen::VectorXd desiredVelocity(testrobot->getDOF());
    Eigen::VectorXd currentPosition(testrobot->getDOF());
    Eigen::VectorXd currentVelocity(testrobot->getDOF());
    Eigen::VectorXd jointNominalConfig(testrobot->getGeneralizedCoordinateDim()), jointVelocityTarget(testrobot->getDOF());
    jointNominalConfig <<  0, 0;
    jointVelocityTarget.setZero();

    desiredPosition << pi/2, pi/2;
    desiredVelocity << 0, 0;
//    std::cout<< testrobot->getDOF() << std::endl;

    testrobot->setGeneralizedCoordinate(jointNominalConfig);
    testrobot->setGeneralizedForce(Eigen::VectorXd::Zero(testrobot->getDOF()));
    testrobot->setName("FRleg");
    std::cout<< testrobot->getGeneralizedCoordinate() << std::endl;
    /// launch raisim server
    raisim::RaisimServer server(&world);
    server.launchServer();
    server.focusOn(testrobot);
    sleep(3);
    for(int i=0; i<2000000; i++){
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        currentPosition[0] = testrobot->getGeneralizedCoordinate()[0];
        currentPosition[1] = testrobot->getGeneralizedCoordinate()[1];
        currentVelocity[0] = testrobot->getGeneralizedVelocity()[0];
        currentVelocity[1] = testrobot->getGeneralizedVelocity()[1];
        beta = Beta(testrobot->getGeneralizedCoordinate()[0], testrobot->getGeneralizedVelocity()[0],
                    testrobot->getGeneralizedCoordinate()[1], testrobot->getGeneralizedVelocity()[1]);
        torque = pGain*(desiredPosition - currentPosition) + dGain*(desiredVelocity - currentVelocity);
        testrobot->setGeneralizedForce(torque);

        if (i > 1000 && i < 10000) {
            testrobot->setExternalForce(testrobot->getBodyIdx("lower_leg"), {0, 0, -0.1}, {0, 0, -30});
        } else {
            testrobot->setExternalForce(testrobot->getBodyIdx("lower_leg"), {0, 0, -0.1}, {0, 0, 0});
        }
        server.integrateWorldThreadSafe();
        if (firstRun == true) {
            momentumPrev[0] = 0;
            momentumPrev[1] = 0;
            residual[0] = 0;
            residual[1] = 0;
            firstRun = false;
        }
        dqMat[0] = testrobot->getGeneralizedVelocity()[0];
        dqMat[1] = testrobot->getGeneralizedVelocity()[1];

        momentum = momentumPrev + torque * 0.001 - beta * 0.001 + residual * 0.001;
        residual = gainK * (-momentum +
                            MassMat(testrobot->getGeneralizedCoordinate()[0], testrobot->getGeneralizedCoordinate()[1]) *
                            dqMat);
        momentumPrev = momentum;

        std::cout<< i<< ">> " << "joint1: " << residual[0] << "," <<" joint2: "<<residual[1] << ", motor torque : "<< torque[0]<<", "<<torque[1]<<std::endl;

    }


    server.killServer();
}

Eigen::Matrix2d MassMat(double q1, double q2) {
    Eigen::Matrix2d massMat;
    double link1 = 0.1492;  //length of link1
    double link2 = 0.381;   //length of link2
    double mass1 = 0.193;   //mass of link1
    double mass2 = 0.073;   //mass of link2

    massMat(0, 0) = mass1 * link1 * link1 + mass2 * (link1 * link1 + 2 * link1 * link2 * std::cos(q1) + link2 * link2);
    massMat(0, 1) = mass2 * (link2 * link2 + link1 * link2 * std::cos(q2));
    massMat(1, 0) = mass2 * (link2 * link2 + link1 * link2 * std::cos(q2));
    massMat(1, 1) = mass2 * link2 * link2;

    return massMat;
}

Eigen::Matrix2d CoriloisMat(double q1, double dq1, double q2, double dq2) {
    Eigen::Matrix2d coriolisMat;
    double link1 = 0.1492;  //length of link1
    double link2 = 0.381;   //length of link2
    double mass1 = 0.193;   //mass of link1
    double mass2 = 0.073;   //mass of link2

    coriolisMat(0, 0) = -mass2 * link1 * link2 * std::sin(q2) * dq2;
    coriolisMat(0, 1) = -mass2 * link1 * link2 * std::sin(q2) * dq1 - mass2 * link1 * link2 * std::sin(q2) * dq2;
    coriolisMat(1, 0) = -mass2 * link1 * link2 * std::sin(q2) * dq1;
    coriolisMat(1, 1) = 0;

    return coriolisMat;
}

Eigen::Vector2d Beta(double q1, double dq1, double q2, double dq2) {
    Eigen::VectorXd dq2Mat = Eigen::VectorXd(2);
    dq2Mat[0] = dq1;
    dq2Mat[1] = dq2;
    return -CoriloisMat(q1, dq1, q2, dq2).transpose() * dq2Mat;
}