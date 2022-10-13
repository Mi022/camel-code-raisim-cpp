//
// Created by jaehoon on 22. 6. 27.
//
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "A1CollisionDetecter.h"
#include "A1CollisionDetecterNoInertia.h"

int main(int argc, char* argv[])
{
    double pi = 3.141592;
    double pGain = 10.0;
    double dGain = 1;
    int i = 0;
    /// create raisim world
    raisim::World world;
    world.setTimeStep(0.001);

    /// for momentum observer
    A1CollisionDetecter frcd;
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
    auto testrobot = world.addArticulatedSystem("\\home\\cha\\git\\repository-group\\raisimLib\\camel-code-raisim-cpp\\rsc\\test_a1_single_leg_right\\camel_single_leg_fixed.urdf");
    Eigen::VectorXd torque(testrobot->getDOF());
    Eigen::VectorXd desiredPosition(testrobot->getDOF());
    Eigen::VectorXd desiredVelocity(testrobot->getDOF());
    Eigen::VectorXd currentPosition(testrobot->getDOF());
    Eigen::VectorXd currentVelocity(testrobot->getDOF());
    Eigen::VectorXd jointNominalConfig(testrobot->getGeneralizedCoordinateDim()), jointVelocityTarget(testrobot->getDOF());
    jointNominalConfig << 0, 0;
    jointVelocityTarget.setZero();

    desiredPosition << pi/3, pi/6;
    desiredVelocity << 0, 0;

    testrobot->setGeneralizedCoordinate(jointNominalConfig);
    testrobot->setGeneralizedForce(Eigen::VectorXd::Zero(testrobot->getDOF()));
    testrobot->setName("FRleg");
    std::cout << testrobot->getGeneralizedCoordinate() << std::endl;
    /// launch raisim server
    raisim::RaisimServer server(&world);
    server.launchServer();
    server.focusOn(testrobot);
    sleep(3);
    for (int i = 0; i < 2000000; i++)
    {
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        currentPosition[0] = testrobot->getGeneralizedCoordinate()[0];
        currentPosition[1] = testrobot->getGeneralizedCoordinate()[1];
        currentVelocity[0] = testrobot->getGeneralizedVelocity()[0];
        currentVelocity[1] = testrobot->getGeneralizedVelocity()[1];

        frcd.UpdateBeta(testrobot->getGeneralizedCoordinate()[0], testrobot->getGeneralizedVelocity()[0], testrobot->getGeneralizedCoordinate()[1], testrobot->getGeneralizedVelocity()[1]);
        beta = frcd.GetBeta();
        torque = pGain * (desiredPosition - currentPosition) + dGain * (desiredVelocity - currentVelocity);
        testrobot->setGeneralizedForce(torque);

        if (i > 2000 && i < 6000)
        {
            testrobot->setExternalForce(testrobot->getBodyIdx("lower_leg"), { 0, 0, -0.1 }, { 0, 0, -30 });
        }
        else
        {
            testrobot->setExternalForce(testrobot->getBodyIdx("lower_leg"), { 0, 0, -0.1 }, { 0, 0, 0 });
        }
        server.integrateWorldThreadSafe();
        frcd.UpdateState(testrobot->getGeneralizedCoordinate(), testrobot->getGeneralizedVelocity(), torque);

        if (i % 100 == 0)
        {
            std::cout << "residual 1 : " << frcd.GetResidualVector()[0] << ", residual 2 : " << frcd.GetResidualVector()[1] << ", torque 1 : " << torque[0] << ", torque 2 : " << torque[1] <<", beta 1 : "<<beta[0] <<", beta 2 : "<<beta[1] <<std::endl;
        }
    }
    server.killServer();
    return 0;
}
