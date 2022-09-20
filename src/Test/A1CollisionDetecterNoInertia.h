//
// Created by cha on 22. 9. 19.
//

#ifndef RAISIM_A1COLLISIONDETECTERNOINERTIA_H
#define RAISIM_A1COLLISIONDETECTERNOINERTIA_H
#include "raisim/World.hpp"

class A1CollisionDetecterNoInertia {
public:
    A1CollisionDetecterNoInertia(){
        mlink1 = 0.2;
        mmass1 = 0;
        mclink1 = 0.027;
        minertia1 = 0.0135;
        mlink2 = 0.2;
        mmass2 = 0.166;
        mclink2 = 0.134;
        minertia2 = 0.00058;
    }

private:
    double mlink1;
    double mlink2;
    double mmass1;
    double mmass2;
    double mclink1;
    double mclink2;
    double minertia1;
    double minertia2;

public:
    /**
     *
     * @param thighLength the length of first link(a1's thigh)
     * @param calfLength  the length of second link(a1's calf)
     * @param thighMass   the mass of first link(a1's thigh)
     * @param calfMass    the mass of second link(a1's calf)
     */
    void changeConfig(double thighLength, double calfLength, double thighMass, double calfMass, double thighComLength, double calfComLength);

    /**
     *
     * @param q1    the position of first joint
     * @param dq1   the velocity of first joint
     * @param q2    the position of second joint
     * @param dq2   the velocity of second joint
     * @return      2x1 beta matrix which consist of gravity matrix and transpose of coriolis matrix
     */
    Eigen::Vector2d Beta(double q1, double dq1, double q2, double dq2);


    /**
     *
     * @param q1    the position of first joint
     * @param dq1   the velocity of first joint
     * @param q2    the position of second joint
     * @param dq2   the velocity of second joint
     * @return      2x2 corilois matrix
     */
    Eigen::Matrix2d CoriloisMat(double q1, double dq1, double q2, double dq2);

    /**
     *
     * @param q1    the position of first joint
     * @param q2    the position of second joint
     * @return      2x1 gravity matrix
     */
    Eigen::Vector2d GravityMat(double q1, double q2);

    /**
     *
     * @param q1    the position of first joint
     * @param q2    the position of second joint
     * @return      2x2 mass matrix
     */
    Eigen::Matrix2d MassMat(double q1, double q2);
};


#endif //RAISIM_A1COLLISIONDETECTERNOINERTIA_H
