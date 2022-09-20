//
// Created by hwayoung on 22. 9. 19.
//

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

std::string rbdlSourcePath = URDF_RSC_DIR;
RigidBodyDynamics::Model* model = new RigidBodyDynamics::Model();
Eigen::VectorXd Q = Eigen::VectorXd::Zero (model->q_size);
Eigen::VectorXd QDot = Eigen::VectorXd::Zero (model->qdot_size);
Eigen::VectorXd Tau = Eigen::VectorXd::Zero (model->qdot_size);
Eigen::VectorXd QDDot = Eigen::VectorXd::Zero (model->qdot_size);

void getModelFromURDF(){
    std::string modelFile = rbdlSourcePath;
    modelFile.append("/camel_hard_IceCream.urdf");
//    std::string modelFile = "\\home\\hwayoung\\raisimLib\\camel-code-raisim-cpp\\rsc\\camel_hard_IceCream.urdf";
    std::cout<<"hi"<<std::endl;
    bool modelLoaded = RigidBodyDynamics::Addons::URDFReadFromFile(modelFile.c_str(), model, false);
    std::cout<<"check: "<<modelLoaded<<std::endl;

}

void Test_DOF(){
    std::cout << "Degree of freedom overview:" << std::endl;
    std::cout << RigidBodyDynamics::Utils::GetModelDOFOverview(*model);

    std::cout << "Model Hierarchy:" << std::endl;
    std::cout << RigidBodyDynamics::Utils::GetModelHierarchy(*model);

}

void Test_ForwardDynamics(){
    Q << 0, -90*3.141592/180;

    std::cout << "Forward Dynamics with q, qdot, tau set to zero:" << std::endl;
    RigidBodyDynamics::ForwardDynamics (*model, Q, QDot, Tau, QDDot);

    std::cout << QDDot.transpose() << std::endl;
}

void getDTStateEquation(){
    Eigen::VectorXd QDDot_delta = Eigen::VectorXd::Zero (model->qdot_size);
    Q << 0, -90*3.141592/180;
    double delta = 1e-3;
    RigidBodyDynamics::ForwardDynamics(*model, Q, QDot, Tau, QDDot);

    Q[0] = Q[0]+delta;
    RigidBodyDynamics::ForwardDynamics(*model, Q, QDot, Tau, QDDot_delta);
    std::cout<<"QDDot_delta: "<<std::endl<<QDDot_delta<<std::endl;
    std::cout<<"QDDot: "<<std::endl<<QDDot<<std::endl;
    std::cout<<"A21-1: "<<std::endl<<(QDDot_delta-QDDot)/1e-3<<std::endl;
    Q[0] = Q[0]-delta;

    Q[1] = Q[1]+delta;
    RigidBodyDynamics::ForwardDynamics(*model, Q, QDot, Tau, QDDot_delta);
    std::cout<<"A21-2: "<<std::endl<<(QDDot_delta-QDDot)/1e-3<<std::endl;
    Q[1] = Q[1]-delta;

    QDot[0] = QDot[0]+delta;
    RigidBodyDynamics::ForwardDynamics(*model, Q, QDot, Tau, QDDot_delta);
    std::cout<<"A22-1: "<<std::endl<<(QDDot_delta-QDDot)/1e-3<<std::endl;
    QDot[0] = QDot[0]-delta;

    QDot[1] = QDot[1]+delta;
    RigidBodyDynamics::ForwardDynamics(*model, Q, QDot, Tau, QDDot_delta);
    std::cout<<"A22-2: "<<std::endl<<(QDDot_delta-QDDot)/1e-3<<std::endl;
    QDot[1] = QDot[1]-delta;

    Tau[1] = Tau[1] + delta;
    RigidBodyDynamics::ForwardDynamics(*model, Q, QDot, Tau, QDDot_delta);
    std::cout<<"B2-2: "<<std::endl<<(QDDot_delta-QDDot)/1e-3<<std::endl;
    Tau[1] = Tau[1] - delta;
}

void Test_StateEquation(){
    Eigen::Matrix<double, 4, 1> X;
    Eigen::Matrix<double, 2, 4> A;
    Eigen::Matrix<double, 2, 1> B;
    Eigen::Matrix<double, 2, 1> QDDot_lin;

    X.block(0,0, 2, 1) = Q;
    X.block(2,0,2,1) = QDot;
    X[1] = X[1] +90*3.141592/180;

    A << 0, -69.139, -30.0552, -30.0552,
        0,  153.732,  36.699,  36.699;
    B << -38.5865,
            59.2874;

    A << 30.0552, -39.0838, 9.42587e-10, 4.23916e-10,
            -36.699,  117.033,  -2.49571e-09,  -9.42587e-10;
    B << -8.53125,
            22.5884;

    double plus = 1e-0;
    Q[0] = Q[0]+plus;
    RigidBodyDynamics::ForwardDynamics(*model, Q, QDot, Tau, QDDot);
    std::cout<<"RBDL_QDDot: "<<std::endl<<QDDot<<std::endl;
    Q[0] = Q[0]-plus;

    X[0] = X[0]+plus;
//    std::cout<<"X: "<<std::endl<<X<<std::endl;
    QDDot_lin = A*X + B*Tau[1];
    std::cout<< "Linear_QDDot: "<< std::endl << QDDot_lin << std::endl;
    X[0] = X[0]-plus;
}

int main(){
    getModelFromURDF();

    Q = Eigen::VectorXd::Zero (model->q_size);
    QDot = Eigen::VectorXd::Zero (model->qdot_size);
    Tau = Eigen::VectorXd::Zero (model->qdot_size);
    QDDot = Eigen::VectorXd::Zero (model->qdot_size);

    getDTStateEquation();
    Test_StateEquation();
    return 0;
}
