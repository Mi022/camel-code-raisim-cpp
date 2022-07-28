#include "include/CAMEL/Controller.h"     //include Controller class, trajectory generator class
#include "include/TrajectoryGenerator/CubicTrajectoryGenerator.h"

//extend Controller class (parent) to SimplePendulumPDController class (child)
class SimplePendulumPDController : public Controller {
public:
    //initialize
    SimplePendulumPDController(Robot *robot) : Controller(robot) {
        updateState();
        mTrajectoryGenerator.updateTrajectory(position[0], -90.0 / 180.0* 3.141592, getRobot() -> getWorldTime(), 5.0);
        setPDGain(200.0, 25.0);
    }

    //Variables used in controller
    Eigen::VectorXd torque = Eigen::VectorXd(1);
    raisim::VecDyn position = raisim::VecDyn(1);
    raisim::VecDyn velocity = raisim::VecDyn(1);
    double positionError;
    double velocityError;
    double desiredPosition;
    double desiredVelocity;

    // PD Gains
    double PGain;
    double DGain;

    // override parent class's methods
    void doControl() override;
    void setTrajectory() override;
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;
    void setPDGain(double PGain, double DGain);

private:
    // Cubic trajectory generator
    CubicTrajectoryGenerator mTrajectoryGenerator;
};