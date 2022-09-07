//
// Created by jaehoon on 22. 5. 2.
//

#ifndef RAISIM_A1JOINTPDCONTROLLER_H
#define RAISIM_A1JOINTPDCONTROLLER_H

#include "include/CAMEL/Controller.h"
#include "A1Robot.h"

/*
     * torque
     *  0: p_x
     *  1: p_y
     *  2: p_z
     *  3: r_x
     *  4: r_y
     *  5: r_z
     *  6: FR_hip
     *  7: FR_thigh
     *  8: FR_calf
     *  9: FL_hip
     *  10: FL_thigh
     *  11: FL_calf
     *  12: RR_hip
     *  13: RR_thigh
     *  14: RR_calf
     *  15: RL_hip
     *  16: RL_thigh
     *  17: RL_calf
     */

/*
 * position
 *  0: p_x
 *  1: p_y
 *  2: p_z
 *  3: q_w0
 *  4: q_w1
 *  5: q_w2
 *  6: q_w3
 *  7: FR_hip
 *  8: FR_thigh
 *  9: FR_calf
 *  10: FL_hip
 *  11: FL_thigh
 *  12: FL_calf
 *  13: RR_hip
 *  14: RR_thigh
 *  15: RR_calf
 *  16: RL_hip
 *  17: RL_thigh
 *  18: RL_calf
 */

/*
 * velocity
 *  0: p_x
 *  1: p_y
 *  2: p_z
 *  3: r_x
 *  4: r_y
 *  5: r_z
 *  6: FR_hip
 *  7: FR_thigh
 *  8: FR_calf
 *  9: FL_hip
 *  10: FL_thigh
 *  11: FL_calf
 *  12: RR_hip
 *  13: RR_thigh
 *  14: RR_calf
 *  15: RL_hip
 *  16: RL_thigh
 *  17: RL_calf
 */

class A1JointPDController : public Controller {
public:
    raisim::VecDyn torque = raisim::VecDyn(18);
    raisim::VecDyn position = raisim::VecDyn(19);
    raisim::VecDyn velocity = raisim::VecDyn(18);
    Eigen::VectorXd positionError = Eigen::VectorXd(12);
    Eigen::VectorXd velocityError = Eigen::VectorXd(12);
    Eigen::VectorXd desiredPosition = Eigen::VectorXd(12);
    Eigen::VectorXd desiredVelocity = Eigen::VectorXd(12);

    double PGain;
    double DGain;
    double torqueLimit = 20.0;

    A1JointPDController(Robot *robot) : Controller(robot) {
        torque.setZero();
        desiredPosition[0] = 0.0;
        desiredPosition[1] = 0.5;
        desiredPosition[2] = -0.9;

        desiredPosition[3] = -0.0;
        desiredPosition[4] = 0.5;
        desiredPosition[5] = -0.9;

        desiredPosition[6] = 0.0;
        desiredPosition[7] = 0.5;
        desiredPosition[8] = -0.9;

        desiredPosition[9] = -0.0;
        desiredPosition[10] = 0.5;
        desiredPosition[11] = -0.9;

        desiredVelocity.setZero();

        setPDGain(200.0, 10.0);
    }

    void doControl() override;
    void setTrajectory() override;
    void updateJointTrajectory(Eigen::VectorXd desiredPosition, Eigen::VectorXd desiredVelocity);
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;
    void setPDGain(double PGain, double DGain);

private:
};


#endif //RAISIM_A1JOINTPDCONTROLLER_H
