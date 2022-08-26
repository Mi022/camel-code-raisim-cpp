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
include_directories("qpOASES/include")

add_subdirectory(qpOASES)


set(CAMEL_LIBS
        Qt5::Widgets
        raisim::raisim
        pthread
        rt
        qpOASES)

set(CAMEL_SOURCES
        include/TrajectoryGenerator/CubicTrajectoryGenerator.cpp
        include/TrajectoryGenerator/QuinticTrajectoryGenerator.cpp
        include/TrajectoryGenerator/SincurveTrajectoryGenerator.cpp
        include/TrajectoryGenerator/legtrajectory.cpp
        include/CAN/CanMotorX8ProV2.h
        include/Optimizer/GradientDescentOptimizer.cpp
        include/Sensor/LoadCell.cpp
        include/Sensor/MPU9250.cpp
        include/Sensor/RE22SC.cpp
        include/RT/rb_utils.cpp
        include/SimulationUI/qcustomplot.cpp
        include/Filter/LPF.cpp
        )

set(CAMEL_SIMULATION_UI_SOURCES
        include/SimulationUI/simulationMainwindow.cpp)

set(CAMEL_A1_SIMULATION_UI_SOURCES
        include/SimulationUI/qcustomplot.cpp
        src/A1/mainwindow.cpp)

## Simulation ##
# ====== Simple Pendulum ======  #
set(SIMPLE_PENDULUM_SOURCES
        src/Tutorial_SimplePendulum/SimplePendulumSimulation.cpp
        src/Tutorial_SimplePendulum/SimplePendulumRobot.cpp
        src/Tutorial_SimplePendulum/SimplePendulumPDController.cpp
        src/Tutorial_SimplePendulum/SimplePendulumSharedMemory.h
        src/Tutorial_SimplePendulum/UI_RealTimePlot/operationMainwindow.cpp
        )

add_executable(camel-tutorial-simplePendulum ${CAMEL_SOURCES} ${SIMPLE_PENDULUM_SOURCES})
target_link_libraries(camel-tutorial-simplePendulum ${CAMEL_LIBS})
# ====== Simple Pendulum ======/ #

# ====== A1 ======  #
set(A1_SOURCES
        src/A1/A1Simulation.cpp
        src/A1/A1Robot.cpp
        src/A1/A1JointPDController.cpp
        src/A1/A1MPCController.cpp
        src/A1/SharedMemory.h
        )

add_executable(camel-a1 ${CAMEL_SOURCES} ${CAMEL_A1_SIMULATION_UI_SOURCES} ${A1_SOURCES})
target_link_libraries(camel-a1 ${CAMEL_LIBS} qpOASES)

# ====== A1 ======/ #

# ====== Canine Single Leg ======  #
set(CANINE_SINGLE_LEG_SOURCES
        src/CAMEL-Quad-SingleLeg/SingleLeggedSimulation.cpp
        src/CAMEL-Quad-SingleLeg/SingleLeggedRobot.cpp
        src/CAMEL-Quad-SingleLeg/SingleLeggedPDController.cpp
        src/CAMEL-Quad-SingleLeg/SingleLeggedIDController.cpp
        src/CAMEL-Quad-SingleLeg/SingleLeggedMPCController.cpp
        src/CAMEL-Quad-SingleLeg/SingleLeggedMPCqpoases.cpp)

add_executable(camel-canine_single_leg_simulation ${CAMEL_SOURCES} ${CAMEL_SIMULATION_UI_SOURCES} ${CANINE_SINGLE_LEG_SOURCES})
target_link_libraries(camel-canine_single_leg_simulation ${CAMEL_LIBS} qpOASES)
# ====== Canine Single Leg ====== /#

# ====== Momentum Inverted Pendulum ======  #
set(MOMENTUM_INVERTED_PENDULUM_SOURCES
        src/MomentumInvertedPendulum/MIPSimulation.cpp
        src/MomentumInvertedPendulum/MIPRobot.cpp
        src/MomentumInvertedPendulum/MIPPDController.cpp
        src/MomentumInvertedPendulum/MIPLQRController.cpp
        src/MomentumInvertedPendulum/MIPPDDController.cpp
        src/MomentumInvertedPendulum/MIPMPCController.cpp)

add_executable(camel-momentum_inverted_pendulum_simulation ${CAMEL_SOURCES} ${CAMEL_SIMULATION_UI_SOURCES} ${MOMENTUM_INVERTED_PENDULUM_SOURCES})
target_link_libraries(camel-momentum_inverted_pendulum_simulation ${CAMEL_LIBS})
# ====== Momentum Inverted Pendulum ====== /#

# ====== Momentum Inverted Pendulum with Incline======  #
set(MOMENTUM_INVERTED_PENDULUM_WITH_INCLINE_SOURCES
        src/MomentumInvertedPendulum-Inclined/MIPISimulation.cpp
        src/MomentumInvertedPendulum-Inclined/MIPIRobot.cpp
        src/MomentumInvertedPendulum-Inclined/MIPILQRController.cpp)

add_executable(camel-momentum_inverted_pendulum_with_incline_simulation ${CAMEL_SOURCES} ${CAMEL_SIMULATION_UI_SOURCES} ${MOMENTUM_INVERTED_PENDULUM_WITH_INCLINE_SOURCES})
target_link_libraries(camel-momentum_inverted_pendulum_with_incline_simulation ${CAMEL_LIBS})
# ====== Momentum Inverted Pendulum ====== /#
## Simulation /##

## Operation  ##
# ====== RMD X8 PRO ======  #
set(RMDX8PRO_SOURCES
        src/RMDX8PRO/RMDX8ProOperation.cpp
        src/RMDX8PRO/mainwindow.cpp)

add_executable(camel-rmdx8operation ${CAMEL_SOURCES} ${RMDX8PRO_SOURCES})
target_link_libraries(camel-rmdx8operation ${CAMEL_LIBS})
# ====== RMD X8 PRO ====== /#

# ====== Canine Single Leg ======  #
set(CANINE_SINGLE_LEG_OPERATION_SOURCES
        src/CAMEL-Quad-SingleLeg-Operation/SingleLeggedOperation.cpp
        src/CAMEL-Quad-SingleLeg-Operation/SingleLeggedRobotOperation.cpp
        src/CAMEL-Quad-SingleLeg-Operation/SingleLegCAN.cpp
        src/CAMEL-Quad-SingleLeg-Operation/UI/mainwindow.cpp
        src/CAMEL-Quad-SingleLeg-Operation/SingleLegCAN.cpp
        src/CAMEL-Quad-SingleLeg-Operation/SingleLeggedPDControllerOperation.cpp
        src/CAMEL-Quad-SingleLeg-Operation/SingleLeggedIDControllerOperation.cpp
        src/CAMEL-Quad-SingleLeg-Operation/SingleLeggedMPCqpoasesOperation.cpp
        src/CAMEL-Quad-SingleLeg-Operation/SingleLeggedSharedMemoryOperation.h)

add_executable(camel-canine_single_leg_operation ${CAMEL_SOURCES} ${CANINE_SINGLE_LEG_OPERATION_SOURCES})
target_link_libraries(camel-canine_single_leg_operation ${CAMEL_LIBS})
# ====== Canine Single Leg ====== /#
## Operation /##

## TEST FILES ##
add_executable(camel-test_trajectory_generation src/Test/Test_trajectoryGenerator.cpp include/TrajectoryGenerator/CubicTrajectoryGenerator.cpp
                                                                                      include/TrajectoryGenerator/QuinticTrajectoryGenerator.cpp
                                                                                      include/TrajectoryGenerator/SincurveTrajectoryGenerator.cpp)
add_executable(camel-rmdx8test_torque src/Test/Test_RMDX8Pro.cpp)
add_executable(camel-test_real_time_thread src/Test/Test_RealTimeThread.cpp ${CAMEL_SOURCES})
target_link_libraries(camel-test_real_time_thread ${CAMEL_LIBS})
add_executable(camel-test_gradient_descent_optimizer src/Test/Test_GradientDescentOptimizer.cpp ${CAMEL_SOURCES})
target_link_libraries(camel-test_gradient_descent_optimizer ${CAMEL_LIBS})

add_executable(camel-test_sensor_loadcell src/Test/Test_SensorLoadCell.cpp ${CAMEL_SOURCES})
target_link_libraries(camel-test_sensor_loadcell ${CAMEL_LIBS})
add_executable(camel-test_urdf src/Test/Test_urdf.cpp)
target_link_libraries(camel-test_urdf ${CAMEL_LIBS})
add_executable(camel-test_real_time_graph src/Test/Test_RealTimeGraph.cpp src/Test/simulationMainwindow.cpp ${CAMEL_SOURCES})
target_link_libraries(camel-test_real_time_graph ${CAMEL_LIBS})
add_executable(camel-test_write_CSV src/Test/Test_WriteCSV.cpp)
add_executable(camel-test_sensor_MPU9250 src/Test/Test_SensorMPU9250.cpp ${CAMEL_SOURCES})
target_link_libraries(camel-test_sensor_MPU9250 ${CAMEL_LIBS})
add_executable(camel-test_sensor_RE22SC src/Test/Test_encoderRE22SC.cpp ${CAMEL_SOURCES} ${CAMEL_SIMULATION_UI_SOURCES})
target_link_libraries(camel-test_sensor_RE22SC ${CAMEL_LIBS})
add_executable(camel-test_SGDmodel src/Test/Test_SGDmodel.cpp ${CAMEL_SOURCES})
target_link_libraries(camel-test_SGDmodel ${CAMEL_LIBS})
add_executable(camel-test_Eigen src/Test/Test_Eigen.cpp ${CAMEL_SOURCES})
target_link_libraries(camel-test_Eigen ${CAMEL_LIBS})
## TEST FILES /##
