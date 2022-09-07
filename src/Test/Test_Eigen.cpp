//
// Created by hwayoung on 22. 8. 19.
//

#include "Eigen/Dense"
#include <iostream>

using namespace std;
using namespace Eigen;

Quaternion<double> qCurrent;
Quaternion<double> qGoal;
Quaternion<double> q;

struct Euler{
    //radian
    double roll;
    double pitch;
    double yaw;
};

static const double rad2deg = 180.0/3.141592;
static const double deg2rad = 3.141592/180.0;

Quaterniond euler2quaternion(Vector3d euler){
    Quaterniond q;
    q = AngleAxisd(euler[0], Vector3d::UnitX())
        * AngleAxisd(euler[1], Vector3d::UnitY())
        * AngleAxisd(euler[2], Vector3d::UnitZ());
    return q;
}
int main(){
    int n;
    MatrixXd m = MatrixXd::Random(5, 3);
    cout << "m =" << endl << m << endl;

    m.conservativeResize(m.rows(), m.cols() + 2);
    cout << "m =" << endl << m << endl;

    MatrixXd m2 = MatrixXd::Random(5, 2);
    cout << "m2 =" << endl << m2 << endl;

    m.rightCols(2) = m2;
    cout << "m =" << endl << m << endl;

    VectorXd currentPosition = VectorXd::Random(9);
    MatrixXd functionValues = MatrixXd::Random(9,1);
    cout << "currentPosition" << endl << currentPosition << endl;
    cout << "functionValues" << endl << functionValues << endl;

    functionValues.rightCols(1) = currentPosition;
    cout << "n2 : " << n << endl;

    cout << "functionValues" << endl << functionValues << endl;

    MatrixXd mFunctionValue = MatrixXd::Random(9,1);
    Eigen::VectorXd zeros = Eigen::VectorXd(currentPosition.size());
    zeros.setZero();

    mFunctionValue.rightCols(1) = currentPosition;
    mFunctionValue.conservativeResize(mFunctionValue.rows(), mFunctionValue.cols() + 1);
    mFunctionValue.rightCols(1) = currentPosition;
    mFunctionValue.conservativeResize(mFunctionValue.rows(), mFunctionValue.cols() + 1);
    mFunctionValue.rightCols(1) = zeros;
    mFunctionValue.conservativeResize(mFunctionValue.rows(), mFunctionValue.cols() + 1);
    mFunctionValue.rightCols(1) = zeros;
    cout << "mFunctionValue" << endl << mFunctionValue << endl;

    //euler to quaternion
    float roll = 1.5707, pitch = 0, yaw = 0.707;
    Quaternionf q;
    q = AngleAxisf(roll, Vector3f::UnitX())
        * AngleAxisf(pitch, Vector3f::UnitY())
        * AngleAxisf(yaw, Vector3f::UnitZ());
    std::cout << "Quaternion" << std::endl << q.coeffs() << std::endl;
    //quaternion to euler
    Vector3f euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    std::cout << "Euler from quaternion in roll, pitch, yaw"<< std::endl << euler << std::endl;

    //slerp
    Vector3d eulerCurrent = {0.0, 0.0, 0.0};
    Quaterniond qCurrent = euler2quaternion(eulerCurrent);
    Quaterniond qGoal = euler2quaternion({90.0*deg2rad, 90.0*deg2rad, 90.0*deg2rad});
    Quaterniond qSLERP;

    qSLERP = qCurrent.slerp(0.5,qGoal);

    Vector3d eulerSLERP = qSLERP.toRotationMatrix().eulerAngles(0,1,2);

    cout<< "qSLERP = " << endl << qSLERP.coeffs()<< endl;
    cout<< "eulerSLERP = "<< endl << eulerSLERP << endl;
    cout<< "90.0*deg2rad = " << 90.0*deg2rad << endl;
}