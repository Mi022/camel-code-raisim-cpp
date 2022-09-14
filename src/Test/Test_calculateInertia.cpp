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

double cubiodInertia(double mass, double a, double b){
    return mass*(a*a + b*b)/12.0;
}

double parallelAxis(double inertia, double mass, double length){
    return inertia + mass*length*length;
}

void canine(){
    Eigen::Matrix3d  inertia = Eigen::Matrix3d();
    inertia.setZero();
    double mass;

    mass = 0.57+0.143+0.03;
    std::cout<<"hip mass: "<<mass<<std::endl;

    mass = 0.109+0.043+0.7+0.146+0.136+0.064+0.031;
    std::cout<<"thigh mass: "<<mass<<std::endl;

    mass = 0.011+0.003+0.142+0.027;
    std::cout<<"calf mass: "<<mass<<std::endl;

    mass = 0.012+0.005+0.007;
    std::cout<<"foot mass: "<<mass<<std::endl;

    mass = 15.759-(0.743+1.229+0.183+0.024)*4.0;
    std::cout<<"trunk mass: "<<mass<<std::endl;

    cylinderInertia(inertia, 0.743, 0.11/2.0, 0.046, Y);
    std::cout<<"hip: "<<std::endl<<inertia<<std::endl;

    boxInertia(inertia, 1.229, {0.079, 0.09 ,0.15+0.142});
    std::cout<<"thigh: "<<std::endl<<inertia<<std::endl;

    cylinderInertia(inertia, 0.183, 0.028/2.0, 0.15+0.135, Z);
    std::cout<<"calf: "<<std::endl<<inertia<<std::endl;

    sphereInertia(inertia, 0.024, 0.03/2.0);
    std::cout<<"foot: "<<std::endl<<inertia<<std::endl;

    boxInertia(inertia, 7.043, {0.342, 0.15+0.093 ,0.14});
    std::cout<<"trunk: "<<std::endl<<inertia<<std::endl;
}

void iceCream(){
    Eigen::Matrix3d  inertia = Eigen::Matrix3d();
    inertia.setZero();
    double mass;

    boxInertia(inertia, 2.179, {0.03, 0.02 ,0.398372});
    std::cout<<"rob: "<<std::endl<<inertia<<std::endl;

    boxInertia(inertia, 3.5215, {0.482, 0.122265 ,0.111});
    std::cout<<"body: "<<std::endl<<inertia<<std::endl;

}
int main (){
    Eigen::Matrix3d  inertia = Eigen::Matrix3d();
    inertia.setZero();
//    sphereInertia(inertia, 5.0, 0.3);
//    boxInertia(inertia,5.0, {0.1, 0.2, 0.3});

//    canine();
    iceCream();
//    double rodLength = 0.35;
//    double rodMass = 1.2;
//    double bodyMass = 2.5;
//    double bodyLength = 0.3;
//    double Ira = parallelAxis(cubiodInertia(rodMass, 0.03, rodLength), rodMass, rodLength/2.0);
//    double IBd = cubiodInertia(bodyMass, bodyLength, 0.1);
//    double IBb = parallelAxis(IBd, bodyMass, bodyLength/2.0);
//
//    std::cout<<"Ira : " <<Ira<<std::endl;
//    std::cout<<"IBb : " <<IBb<<std::endl;
    return 0;
}