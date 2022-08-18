//
// Created by hwayoung on 22. 8. 18.
//
#include <iostream>
#include <Eigen/Dense>
//rotational axis and body frame and center of mass location is same.
enum CirclePosition{
    X,
    Y,
    Z
};
void sphereInertia(Eigen::Matrix3d &inertia, double mass, double radius){
    inertia(0,0) = 2.0*mass*radius*radius/3;
    inertia(1,1) = 2.0*mass*radius*radius/3;
    inertia(2,2) = 2.0*mass*radius*radius/3;
}

void boxInertia(Eigen::Matrix3d &inertia, double mass, Eigen::Vector3d xyz){
    inertia(0,0) = mass*(xyz[1]*xyz[1] + xyz[2]*xyz[2])/12.0;
    inertia(1,1) = mass*(xyz[0]*xyz[0] + xyz[2]*xyz[2])/12.0;
    inertia(2,2) = mass*(xyz[0]*xyz[0] + xyz[1]*xyz[1])/12.0;
}

void cylinderInertia(Eigen::Matrix3d &inertia, double mass, double radius, double height, CirclePosition circlePosition){
    inertia(circlePosition,circlePosition) = mass*radius*radius/2.0;
    inertia((circlePosition+1)%3,(circlePosition+1)%3) = mass*(3.0*radius*radius + height*height)/12.0;
    inertia((circlePosition+2)%3,(circlePosition+2)%3) = mass*(3.0*radius*radius + height*height)/12.0;
}
int main (){
    Eigen::Matrix3d  inertia = Eigen::Matrix3d();
    inertia.setZero();
//    sphereInertia(inertia, 5.0, 0.3);
//    boxInertia(inertia,5.0, {0.1, 0.2, 0.3});
    cylinderInertia(inertia, 0.7, 0.05, 0.044, Y);
    std::cout<<inertia<<std::endl;
    return 0;
}