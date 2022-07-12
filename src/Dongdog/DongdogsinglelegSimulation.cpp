#include "DongdogsinglelegSimulation.h"
#include "include/SimulationUI/simulationMainwindow.h"
#include "include/RT/rb_utils.h"
#include <QApplication>
#include <cmath>

int main(int argc, char *argv[]) {
    std::string urdfPath = "\\home\\lee\\raisim_ws\\raisimLib\\rsc\\a1\\urdf\\a1.urdf";
    std::string name = "cuteA1";
    raisim::World world;
    double simulationDuration = 3.0;
    DongdogsinglelegSimulation sim = DongdogsinglelegSimulation(&world, dT);
    DongdogsinglelegRobot robot = DongdogsinglelegRobot(&world, urdfPath, name);
    DongdogsinglelegPDController controller = DongdogsinglelegPDController(&robot);

    raisim::RaisimServer server(&world);
    server.launchServer(8080);
    server.integrateWorldThreadSafe();

    QApplication a(argc, argv);
    MainWindow w;
    std::thread thread1(thread1task, &world, &robotA1, &PDcontroller, simulationDuration);
    std::thread thread2(thread2task);
    w.show();

    return a.exec();
}

