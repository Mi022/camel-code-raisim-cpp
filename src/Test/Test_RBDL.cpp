//
// Created by hwayoung on 22. 9. 19.
//

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

std::string rbdlSourcePath = URDF_RSC_DIR;
RigidBodyDynamics::Model* model = new RigidBodyDynamics::Model();
Eigen::VectorXd Q;
Eigen::VectorXd QDot;
Eigen::VectorXd Tau;
Eigen::VectorXd QDDot;

void getModelFromURDF()
{
    std::string modelFile = rbdlSourcePath;
    modelFile.append("/camel_double_bar_IceCream.urdf");
    std::cout << "hi" << std::endl;
    bool modelLoaded = RigidBodyDynamics::Addons::URDFReadFromFile(modelFile.c_str(), model, false);
    std::cout << "check: " << modelLoaded << std::endl;
}

void Test_DOF()
{
    std::cout << "Degree of freedom overview:" << std::endl;
    std::cout << RigidBodyDynamics::Utils::GetModelDOFOverview(*model);

    std::cout << "Model Hierarchy:" << std::endl;
    std::cout << RigidBodyDynamics::Utils::GetModelHierarchy(*model);
}

void Test_ForwardDynamics()
{
    Q << -30 * 3.141592 / 180, -2 * 30 * 3.141592 / 180, -(90 - 30) * 3.141592 / 180;
    Tau << -0.810015, 6.34245, -3.86885e-06;

    std::cout << "size(Q, QDot, QDDot, Tau): ( " << Q.size() << ", " << QDot.size() << ", " << QDDot.size() << ", " << Tau.size() << ")" << std::endl;

    std::cout << "desired:" << std::endl << "Q:" << std::endl << Q << std::endl << "QDot:" << std::endl << QDot << std::endl << "QDDot:" << std::endl << QDDot << std::endl;
    std::cout << "Forward Dynamics with q, qdot, tau set to zero:" << std::endl;
    RigidBodyDynamics::ForwardDynamics(*model, Q, QDot, Tau, QDDot);

    std::cout << QDDot.transpose() << std::endl;
}

void Test_InverseDynamics()
{
    Q << -30 * 3.141592 / 180, -2 * 30 * 3.141592 / 180, -(90 - 30) * 3.141592 / 180;
    std::cout << "Use Inverse dynamics get desired Tau for zero qDDot in desired Q, QDot:" << std::endl;
    std::cout << "desired:" << std::endl << "Q:" << std::endl << Q << std::endl << "QDot:" << std::endl << QDot << std::endl << "QDDot:" << std::endl << QDDot << std::endl;
    std::cout << "Tau: " << std::endl << Tau << std::endl;
    RigidBodyDynamics::InverseDynamics(*model, Q, QDot, QDDot, Tau);

    std::cout << Tau.transpose() << std::endl;
}

void Test_modelWork()
{
    Q << -30 * 3.141592 / 180, -2 * 30 * 3.141592 / 180, -(90 - 30) * 3.141592 / 180;
    for (int i = 0; i < 100; i++)
    {
        std::cout << "iteration " << i << " :" << std::endl;
        QDDot.setZero(); // desired QDDot
        RigidBodyDynamics::InverseDynamics(*model, Q, QDot, QDDot, Tau);
        RigidBodyDynamics::ForwardDynamics(*model, Q, QDot, Tau, QDDot); // calculate next step
        Q = Q + 0.005 * QDot;
        QDot = QDot + 0.005 * QDDot;
        std::cout << "Q:" << std::endl << Q << std::endl << "QDot:" << std::endl << QDot << std::endl << "QDDot:" << std::endl << QDDot << std::endl << "Tau:" << std::endl << Tau << std::endl;

    }
}

void getDTStateEquation()
{
    Eigen::VectorXd QDDot_delta = Eigen::VectorXd::Zero(model->qdot_size);
    Q << -30 * 3.141592 / 180, -2 * 30 * 3.141592 / 180, -(90 - 30) * 3.141592 / 180;
    double delta = 1e-3;
    std::cout << "Q: " << std::endl << Q << std::endl;
    std::cout << "QDot: " << std::endl << QDot << std::endl;
    std::cout << "Tau: " << std::endl << Tau << std::endl;

    RigidBodyDynamics::ForwardDynamics(*model, Q, QDot, Tau, QDDot);
    std::cout << "QDDot: " << std::endl << QDDot << std::endl;

    Q[0] = Q[0] + delta;
    RigidBodyDynamics::ForwardDynamics(*model, Q, QDot, Tau, QDDot_delta);
    std::cout << "QDDot_delta: " << std::endl << QDDot_delta << std::endl;
    std::cout << "A21-1: " << std::endl << (QDDot_delta - QDDot) / 1e-3 << std::endl;
    Q[0] = Q[0] - delta;

    Q[1] = Q[1] + delta;
    RigidBodyDynamics::ForwardDynamics(*model, Q, QDot, Tau, QDDot_delta);
    std::cout << "A21-2: " << std::endl << (QDDot_delta - QDDot) / 1e-3 << std::endl;
    Q[1] = Q[1] - delta;

    Q[2] = Q[2] + delta;
    RigidBodyDynamics::ForwardDynamics(*model, Q, QDot, Tau, QDDot_delta);
    std::cout << "A21-3: " << std::endl << (QDDot_delta - QDDot) / 1e-3 << std::endl;
    Q[2] = Q[2] - delta;

    QDot[0] = QDot[0] + delta;
    RigidBodyDynamics::ForwardDynamics(*model, Q, QDot, Tau, QDDot_delta);
    std::cout << "A22-1: " << std::endl << (QDDot_delta - QDDot) / 1e-3 << std::endl;
    QDot[0] = QDot[0] - delta;

    QDot[1] = QDot[1] + delta;
    RigidBodyDynamics::ForwardDynamics(*model, Q, QDot, Tau, QDDot_delta);
    std::cout << "A22-2: " << std::endl << (QDDot_delta - QDDot) / 1e-3 << std::endl;
    QDot[1] = QDot[1] - delta;

    QDot[2] = QDot[2] + delta;
    RigidBodyDynamics::ForwardDynamics(*model, Q, QDot, Tau, QDDot_delta);
    std::cout << "A22-3: " << std::endl << (QDDot_delta - QDDot) / 1e-3 << std::endl;
    QDot[2] = QDot[2] - delta;

    Tau[1] = Tau[1] + delta;
    RigidBodyDynamics::ForwardDynamics(*model, Q, QDot, Tau, QDDot_delta);
    std::cout << "B2-2: " << std::endl << (QDDot_delta - QDDot) / 1e-3 << std::endl;
    Tau[1] = Tau[1] - delta;

    Tau[2] = Tau[2] + delta;
    RigidBodyDynamics::ForwardDynamics(*model, Q, QDot, Tau, QDDot_delta);
    std::cout << "B2-3: " << std::endl << (QDDot_delta - QDDot) / 1e-3 << std::endl;
    Tau[2] = Tau[2] - delta;
}

void Test_StateEquation()
{
    Eigen::Matrix<double, 6, 1> X;
    Eigen::Matrix<double, 3, 4> A;
    Eigen::Matrix<double, 3, 1> B;
    Eigen::Matrix<double, 3, 1> QDDot_lin;

    X.block(0, 0, 2, 1) = Q;
    X.block(2, 0, 2, 1) = QDot;
    X[1] = X[1] + 90 * 3.141592 / 180;

    A << 30.0552, -39.0838, 9.42587e-10, 4.23916e-10,
            -36.699, 117.033, -2.49571e-09, -9.42587e-10;
    B << -8.53125,
            22.5884;

    double plus = 1e-0;
    Q[0] = Q[0] + plus;
    RigidBodyDynamics::ForwardDynamics(*model, Q, QDot, Tau, QDDot);
    std::cout << "RBDL_QDDot: " << std::endl << QDDot << std::endl;
    Q[0] = Q[0] - plus;

    X[0] = X[0] + plus;
//    std::cout<<"X: "<<std::endl<<X<<std::endl;
//    QDDot_lin = A*X + B*Tau;
    std::cout << "Linear_QDDot: " << std::endl << QDDot_lin << std::endl;
    X[0] = X[0] - plus;
}

int main()
{
    getModelFromURDF();

    Q = Eigen::VectorXd::Zero(model->q_size);
    QDot = Eigen::VectorXd::Zero(model->qdot_size);
    Tau = Eigen::VectorXd::Zero(model->qdot_size);
    QDDot = Eigen::VectorXd::Zero(model->qdot_size);

//    Test_InverseDynamics();
    Test_ForwardDynamics();

//    Test_modelWork();

//    getDTStateEquation();
//    Test_StateEquation();
    return 0;
}
