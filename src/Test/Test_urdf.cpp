//
// Created by jaehoon on 22. 6. 27.
//
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"


int main(int argc, char* argv[]) {

    /// create raisim world
    raisim::World world;
    world.setTimeStep(0.001);

    /// create objects
    world.addGround();
    auto robot = world.addArticulatedSystem("\\home\\jaehoon\\raisimLib\\camel-code-raisim-cpp\\rsc\\camel_wire_arm\\camel_wire_arm.urdf");
    auto robot1 = world.addArticulatedSystem("\\home\\jaehoon\\raisimLib\\camel-code-raisim-cpp\\rsc\\camel_wire_arm\\camel_wire_arm1.urdf");
    auto robot2 = world.addArticulatedSystem("\\home\\jaehoon\\raisimLib\\camel-code-raisim-cpp\\rsc\\camel_wire_arm\\camel_wire_arm2.urdf");

    /// kinova joint PD controller
    Eigen::VectorXd jointNominalConfig(robot->getGeneralizedCoordinateDim()), jointVelocityTarget(robot->getDOF());
    double pi = 3.141592;
    double deg2rad = 3.141592/180.0;

    double theta1 = 60.0 * deg2rad;
    double theta2 = 45.0 * deg2rad;
    double theta3 = 35.0 * deg2rad;

    jointVelocityTarget.setZero();
    jointNominalConfig << theta1;
    robot->setGeneralizedCoordinate(jointNominalConfig);
    robot->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF()));
    robot->setName("robot");

    jointNominalConfig << theta2;
    robot1->setGeneralizedCoordinate(jointNominalConfig);
    robot1->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF()));
    robot1->setName("robot1");

    jointNominalConfig << theta3;
    robot2->setGeneralizedCoordinate(jointNominalConfig);
    robot2->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF()));
    robot2->setName("robot2");
    /// launch raisim server
    raisim::RaisimServer server(&world);
    server.launchServer();
    server.focusOn(robot);
    sleep(30);
    for (int i=0; i<5; i++) {
        sleep(5);
        if(i<2)
        {
            theta1 += 40.0 * deg2rad;
            jointNominalConfig << theta1;
            robot->setGeneralizedCoordinate(jointNominalConfig);
            theta2 += 38.5 * deg2rad;
            jointNominalConfig << theta2;
            robot1->setGeneralizedCoordinate(jointNominalConfig);
            theta3 += 38.0 * deg2rad;
            jointNominalConfig << theta3;
            robot2->setGeneralizedCoordinate(jointNominalConfig);
        }
        else if(i==2)
        {
            theta1 += 10.0 * deg2rad;
            jointNominalConfig << theta1;
            robot->setGeneralizedCoordinate(jointNominalConfig);
            theta2 += 10.0 * deg2rad;
            jointNominalConfig << theta2;
            robot1->setGeneralizedCoordinate(jointNominalConfig);
            theta3 += 10.0 * deg2rad;
            jointNominalConfig << theta3;
            robot2->setGeneralizedCoordinate(jointNominalConfig);
        }
        else
        {
            theta1 += 30.0 * deg2rad;
            jointNominalConfig << theta1;
            robot->setGeneralizedCoordinate(jointNominalConfig);
            theta2 += 30.0 * deg2rad;
            jointNominalConfig << theta2;
            robot1->setGeneralizedCoordinate(jointNominalConfig);
            theta3 += 30.0 * deg2rad;
            jointNominalConfig << theta3;
            robot2->setGeneralizedCoordinate(jointNominalConfig);
        }




//
        server.integrateWorldThreadSafe();
    }

    server.killServer();
}