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
        src/CAMEL-Quad-SingleLeg-Operation/SingleLeggedPDControllerOperation.cpp
        src/CAMEL-Quad-SingleLeg-Operation/SingleLeggedIDControllerOperation.cpp
        src/CAMEL-Quad-SingleLeg-Operation/SingleLeggedMPCqpoasesOperation.cpp
        src/CAMEL-Quad-SingleLeg-Operation/SingleLeggedSharedMemoryOperation.h
        src/CAMEL-Quad-SingleLeg-Operation/SingleLeggedMPCOperation.cpp)

add_executable(camel-canine_single_leg_operation ${CAMEL_SOURCES} ${CANINE_SINGLE_LEG_OPERATION_SOURCES})
target_link_libraries(camel-canine_single_leg_operation ${CAMEL_LIBS})
# ====== Canine Single Leg ====== /#

# ====== JHdog Single Leg Circle======  #
set(JHDOG_SOURCES
        src/camel_JHdog/UI_RealTimePlot/operationMainwindow.cpp
        src/camel_JHdog/JHdog.cpp
        src/camel_JHdog/JHdogSimulation.cpp
        src/camel_JHdog/JHdogPDController.cpp
        )

add_executable(camel-jhdog_single_leg_simulation ${CAMEL_SOURCES} ${JHDOG_SOURCES})
target_link_libraries(camel-jhdog_single_leg_simulation ${CAMEL_LIBS})
# ====== JHdog Single Leg Circle====== /#

## Operation /##

