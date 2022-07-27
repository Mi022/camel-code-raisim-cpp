cmake_minimum_required(VERSION 3.5)
project(camel-code-raisim-cpp)

#==
# Dependencies
#==
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5 CONFIG REQUIRED COMPONENTS Widgets)
find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})

file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../rsc" DESTINATION ${CMAKE_BINARY_DIR}/camel-code-raisim-cpp)

include_directories(${EIGEN3_INCLUDE_DIRS})

set(CAMEL_LIBS
        Qt5::Widgets
        raisim::raisim
        pthread
        rt)

set(CAMEL_SOURCES
        include/TrajectoryGenerator/CubicTrajectoryGenerator.cpp
        include/TrajectoryGenerator/QuinticTrajectoryGenerator.cpp
        include/TrajectoryGenerator/RobotArmTrajectoryGenerator.cpp
        include/CAN/CanMotorX8ProV2.h
        include/Optimizer/GradientDescentOptimizer.cpp
        include/Sensor/LoadCell.cpp
        include/RT/rb_utils.cpp)

set(CAMEL_SIMULATION_UI_SOURCES
        include/SimulationUI/qcustomplot.cpp
        include/SimulationUI/simulationMainwindow.cpp)

## Simulation ##
# ====== Simple Pendulum ======  #
set(SIMPLE_PENDULUM_SOURCES
        src/SimplePendulum/SimplePendulumSimulation.cpp
        src/SimplePendulum/SimplePendulumRobot.cpp
        src/SimplePendulum/SimplePendulumPDController.cpp)

add_executable(camel-simplePendulum ${CAMEL_SOURCES} ${CAMEL_SIMULATION_UI_SOURCES} ${SIMPLE_PENDULUM_SOURCES})
target_link_libraries(camel-simplePendulum ${CAMEL_LIBS})
# ====== Simple Pendulum ======/ #

# ====== A1 ======  #
set(A1_SOURCES
        src/A1/A1Simulation.cpp
        src/A1/A1Robot.cpp
        src/A1/A1JointPDController.cpp)

add_executable(camel-a1 ${CAMEL_SOURCES} ${CAMEL_SIMULATION_UI_SOURCES} ${A1_SOURCES})
target_link_libraries(camel-a1 ${CAMEL_LIBS})
# ====== A1 ======/ #

# ====== Canine Single Leg ======  #
set(CANINE_SINGLE_LEG_SOURCES
        src/CAMEL-Quad-SingleLeg/SingleLeggedSimulation.cpp
        src/CAMEL-Quad-SingleLeg/SingleLeggedRobot.cpp
        src/CAMEL-Quad-SingleLeg/SingleLeggedPDController.cpp
        src/CAMEL-Quad-SingleLeg/SingleLeggedIDController.cpp
        src/CAMEL-Quad-SingleLeg/SingleLeggedMPCController.cpp)

add_executable(camel-canine_single_leg_simulation ${CAMEL_SOURCES} ${CAMEL_SIMULATION_UI_SOURCES} ${CANINE_SINGLE_LEG_SOURCES})
target_link_libraries(camel-canine_single_leg_simulation ${CAMEL_LIBS})
# ====== Canine Single Leg ====== /#

# ====== Robotic Arm ======  #

set(ROBOT_ARM_SOURCES
        src/RobotArm/RobotArmSimulation.cpp
        src/RobotArm/RobotArmRobot.cpp
        src/RobotArm/RobotArmPDController.cpp
        src/RobotArm/RobotArmMotionPlanning.cpp
        src/RobotArm/RobotArmCollisionChecker.cpp
        src/RobotArm/RobotArmForwardKinematics.cpp
        src/RobotArm/DistanceCalculator.cpp
        src/RobotArm/LineInterpolation.cpp
        src/RobotArm/RemoveMatrix.cpp)

add_executable(camel-robotArm ${CAMEL_SOURCES} ${CAMEL_SIMULATION_UI_SOURCES} ${ROBOT_ARM_SOURCES})
target_link_libraries(camel-robotArm ${CAMEL_LIBS})

# ====== Robotic Arm ====== /#

## Simulation /##

## Operation  ##
# ====== RMD X8 PRO ======  #
set(RMDX8PRO_SOURCES
        src/RMDX8PRO/RMDX8ProOperation.cpp
        src/RMDX8PRO/mainwindow.cpp
        src/RMDX8PRO/qcustomplot.cpp)

add_executable(camel-rmdx8operation ${CAMEL_SOURCES} ${RMDX8PRO_SOURCES})
target_link_libraries(camel-rmdx8operation ${CAMEL_LIBS})
# ====== RMD X8 PRO ====== /#

# ====== Canine Single Leg ======  #
set(CANINE_SINGLE_LEG_OPERATION_SOURCES
        src/CAMEL-Quad-SingleLeg-Operation/SingleLeggedOperation.cpp
        src/CAMEL-Quad-SingleLeg-Operation/SingleLeggedRobotOperation.cpp
        src/CAMEL-Quad-SingleLeg-Operation/SingleLegCAN.cpp
        src/CAMEL-Quad-SingleLeg-Operation/mainwindow.cpp
        src/CAMEL-Quad-SingleLeg-Operation/qcustomplot.cpp
        src/CAMEL-Quad-SingleLeg-Operation/SingleLegCAN.cpp
        src/CAMEL-Quad-SingleLeg-Operation/SingleLeggedPDControllerOperation.cpp
        src/CAMEL-Quad-SingleLeg-Operation/SingleLeggedIDControllerOperation.cpp)

add_executable(camel-canine_single_leg_operation ${CAMEL_SOURCES} ${CANINE_SINGLE_LEG_OPERATION_SOURCES})
target_link_libraries(camel-canine_single_leg_operation ${CAMEL_LIBS})
# ====== Canine Single Leg ====== /#
## Operation /##

add_executable(camel-test_trajectory_generation src/Test/Test_trajectoryGenerator.cpp include/TrajectoryGenerator/CubicTrajectoryGenerator.cpp include/TrajectoryGenerator/QuinticTrajectoryGenerator.cpp)
add_executable(camel-rmdx8test_torque src/Test/Test_RMDX8Pro.cpp)
add_executable(camel-test_real_time_thread src/Test/Test_RealTimeThread.cpp ${CAMEL_SOURCES})
target_link_libraries(camel-test_real_time_thread ${CAMEL_LIBS})
add_executable(camel-test_gradient_descent_optimizer src/Test/Test_GradientDescentOptimizer.cpp ${CAMEL_SOURCES})
target_link_libraries(camel-test_gradient_descent_optimizer ${CAMEL_LIBS})

add_executable(camel-test_sensor_loadcell src/Test/Test_SensorLoadCell.cpp ${CAMEL_SOURCES})
target_link_libraries(camel-test_sensor_loadcell ${CAMEL_LIBS})