//
// Created by hs on 22. 6. 27.
//

#include "A1MPCController.h"

#define BIG_NUMBER 5e10
#define F_MAX 120
#define MU 0.6

char var_elim[2000];
char con_elim[2000];

void A1MPCController::doControl() {
    std::cout<<"simTime : "<<getRobot()->getWorldTime()<<std::endl;
    updateState();
    setTrajectory();
    getMetrices();
    qpSolver();
    computeControlInput();
    setControlInput();
}

void A1MPCController::updateState(){
    position = getRobot()->robot->getGeneralizedCoordinate();
    velocity = getRobot()->robot->getGeneralizedVelocity();

    p << position[0], position[1], position[2];
    quat << position[3], position[4], position[5], position[6];
    v << velocity[0], velocity[1], velocity[2];
    w << velocity[3], velocity[4], velocity[5];

    quat_to_euler(quat, q);

    x0 << q[0], q[1], q[2],
          p[0], p[1], p[2],
          w[0], w[1], w[2],
          v[0], v[1], v[2],
          mGravity;
}

void A1MPCController::setTrajectory() {
    std::cout << "Set trajectory function" << std::endl;
    double tempXd[13] = {0.f, 0.f, 0.f,
                         0.f, 0.f, 0.3590,
                         0.f, 0.f, 0.f,
                         0.f, 0.f, 0.f,
                         0.f};
    for(int i = 0; i < mMPCHorizon ; i++)
    {
        for(int j=0; j<13;j++)
        {
            xd(13*i+j,0) = tempXd[j];
        }
    }
    desiredPosition = xd(5,0);
    desiredVelocity = xd(11,0);
}

void A1MPCController::getMetrices(){
    Eigen::Matrix<double,13,13> Ac;
    Eigen::Matrix<double,13,12> Bc;

    ss_mats(Ac, Bc);
    c2qp(Ac,Bc);

    //weights
    Eigen::Matrix<double, 13,1> weightMat;
    weightMat << 0, 0, 0,
                 0, 0, 4,
                 0, 0, 0,
                 0, 0, 1,
                 0.f;
    L.diagonal() = weightMat.replicate(mMPCHorizon,1);

    H = 2*(Bqp.transpose()*L*Bqp + alpha*K);
    g = 2*Bqp.transpose()*L*(Aqp*x0 - xd);

    int k = 0;
    for(int i = 0; i < mMPCHorizon; i++)
    {
        for(int16_t j = 0; j < 4; j++)
        {
            U_b(5*k + 0) = BIG_NUMBER;
            U_b(5*k + 1) = BIG_NUMBER;
            U_b(5*k + 2) = BIG_NUMBER;
            U_b(5*k + 3) = BIG_NUMBER;
            U_b(5*k + 4) = F_MAX;
            //U_b(5*k + 4) = update->gait[i*4 + j] * setup->f_max;
            k++;
        }
    }

    //fmat initialize
    float mu = 1.f/MU;
    Eigen::Matrix<double,5,3> f_block;

    f_block <<  mu, 0,  1.f,
            -mu, 0,  1.f,
            0,  mu, 1.f,
            0, -mu, 1.f,
            0,   0, 1.f;

    for(int i = 0; i <  mMPCHorizon*4; i++) //12
    {
        fmat.block(i*5,i*3,5,3) = f_block; //Such like diagonal matrix
    }
}

int8_t near_zero(float a)
{
    return (a < 0.01 && a > -.01) ;
}

int8_t near_one(float a)
{
    return near_zero(a-1);
}

void A1MPCController::matrix_to_real(qpOASES::real_t* dst, Eigen::Matrix<double,Dynamic,Dynamic> src, int16_t rows, int16_t cols){
    int32_t a = 0;
    for(int16_t r = 0; r < rows; r++)
    {
        for(int16_t c = 0; c < cols; c++)
        {
            dst[a] = src(r,c);
            a++;
        }
    }
}

void A1MPCController::qpSolver(){
    qpOASES::real_t* H_qpoases;
    qpOASES::real_t* g_qpoases;
    qpOASES::real_t* A_qpoases;
    qpOASES::real_t* ub_qpoases;
    qpOASES::real_t* lb_qpoases;
    qpOASES::real_t* q_soln;

    qpOASES::real_t* H_red;
    qpOASES::real_t* g_red;
    qpOASES::real_t* A_red;
    qpOASES::real_t* lb_red;
    qpOASES::real_t* ub_red;
    qpOASES::real_t* q_red;

    H_qpoases = (qpOASES::real_t*)malloc(12*mMPCHorizon*12*mMPCHorizon*sizeof(qpOASES::real_t));
    g_qpoases = (qpOASES::real_t*)malloc(12*mMPCHorizon*sizeof(qpOASES::real_t));
    A_qpoases = (qpOASES::real_t*)malloc(12*mMPCHorizon*20*mMPCHorizon*sizeof(qpOASES::real_t));
    ub_qpoases = (qpOASES::real_t*)malloc(20*mMPCHorizon*sizeof(qpOASES::real_t));
    lb_qpoases = (qpOASES::real_t*)malloc(20*mMPCHorizon*sizeof(qpOASES::real_t));
    q_soln = (qpOASES::real_t*)malloc(12*mMPCHorizon*sizeof(qpOASES::real_t));

    H_red = (qpOASES::real_t*)malloc(12*12*mMPCHorizon*mMPCHorizon*sizeof(qpOASES::real_t));
    g_red = (qpOASES::real_t*)malloc(12*1*mMPCHorizon*sizeof(qpOASES::real_t));
    A_red = (qpOASES::real_t*)malloc(12*20*mMPCHorizon*mMPCHorizon*sizeof(qpOASES::real_t));
    lb_red = (qpOASES::real_t*)malloc(20*1*mMPCHorizon*sizeof(qpOASES::real_t));
    ub_red = (qpOASES::real_t*)malloc(20*1*mMPCHorizon*sizeof(qpOASES::real_t));
    q_red = (qpOASES::real_t*)malloc(12*mMPCHorizon*sizeof(qpOASES::real_t));

    matrix_to_real(H_qpoases, H, 12*mMPCHorizon, 12*mMPCHorizon);
    matrix_to_real(g_qpoases, g, 12*mMPCHorizon, 1);
    matrix_to_real(A_qpoases, fmat, 20*mMPCHorizon, 12*mMPCHorizon);
    matrix_to_real(ub_qpoases,U_b, 20*mMPCHorizon, 1);

    for(int i =0; i<20*mMPCHorizon; i++)
        lb_qpoases[i] = 0.f;

    // Set red matrices
    int16_t num_constraints = 20*mMPCHorizon;
    int16_t num_variables = 12*mMPCHorizon;

    int new_cons = num_constraints;
    int new_vars = num_variables;

    //Set the length of metrices depend on variables and constraintss
    for(int i=0; i<num_constraints; i++)
        con_elim[i] = 0;
    for(int i=0; i<num_variables; i++)
        var_elim[i] = 0;

    // Both of lb, ub is not near the zero, var_elim[5]/con_elim[3] => 1
    // the other side, var_elim[5]/con_elim[3] => 0
    for(int i=0; i<num_constraints; i++)
    {
        // near the zero (-0.01 ~ 0.01)
        // lb_qpoases => all zero
        // ub_qpoases => 0~3: BIG_VALUE ,
        //            =>  4 : depend on gait[0 or 1]*f_max / if stand it all set f_max.
        if(!(near_zero(lb_qpoases[i]) && near_zero(ub_qpoases[i])))
            continue;

        // Both of lb, ub is not near the zero
        // If gait is 0, the code below is passed
        double* c_row = &A_qpoases[i*num_variables];
        for(int j=0; j<num_variables; j++)
        {
            if(near_one(c_row[j])) //f_block third column values
            {
                //std::cout << i << "\t" << j << "\t" <<c_row[j] << std::endl;
                new_vars -= 3;
                new_cons -= 5;
                int cs = (j*5)/3 -3;
                var_elim[j-2] = 1;
                var_elim[j-1] = 1;
                var_elim[j  ] = 1;
                con_elim[cs+4] = 1;
                con_elim[cs+3] = 1;
                con_elim[cs+2] = 1;
                con_elim[cs+1] = 1;
                con_elim[cs  ] = 1;
            }
        }
    }

    int var_idx[new_vars];
    int con_idx[new_cons];
    int count = 0;

    // If gait is 1, save the indexes
    for(int i=0; i<num_variables; i++)
    {
        // If gait is 1
        if(!var_elim[i])
        {
            if(!(count<new_vars))
            {
                std::cout << "BAD ERROR" << std::endl;
            }
            var_idx[count] = i;
            count++;
        }
    }
    count=0;
    for(int i=0; i<num_constraints; i++)
    {
        // If gait is 1
        if(!con_elim[i])
        {
            if(!(count<new_cons))
            {
                std::cout << "BAD ERROR" << std::endl;
            }
            con_idx[count] = i;
            count++;
        }
    }

    for(int i=0; i<new_vars;i++)
    {
        int old_a = var_idx[i];
        g_red[i] = g_qpoases[old_a];
        for(int j=0; j<new_vars; j++)
        {
            int old_b = var_idx[j];
            H_red[i*new_vars+j] = H_qpoases[old_a * num_variables + old_b];
        }
    }
    for(int i=0; i<new_cons;i++)
    {
        for(int j=0; j<new_vars; j++)
        {
            float cval = A_qpoases[(num_variables*con_idx[i]) + var_idx[j]];
            A_red[i*new_vars+j] = cval;
        }
    }

    for(int i=0; i<new_cons; i++)
    {
        int old = con_idx[i];
        ub_red[i] = ub_qpoases[old];
        lb_red[i] = lb_qpoases[old];
    }

    // Solve the problem using qpOASES
    qpOASES::int_t nWSR = 10000;
    qpOASES::QProblem problem(new_vars, new_cons);
    qpOASES::Options op;
    op.setToMPC();
    op.printLevel = qpOASES::PL_NONE;
    problem.setOptions(op);
    problem.init(H_red, g_red, A_red, NULL, NULL, lb_red, ub_red, nWSR);

    int rval = problem.getPrimalSolution(q_red);
    if(rval != qpOASES::SUCCESSFUL_RETURN)
        printf("failed to solve!\n");

    int vc = 0;
    for(int i = 0; i < num_variables; i++)
    {
        if(var_elim[i])
        {
            q_soln[i] = 0.0f;
        }
        else
        {
            q_soln[i] = q_red[vc];
            vc++;
        }
    }

    // [Fx,Fy,Fz]
    std::cout << "[F values] " << std::endl;
    for(int leg = 0; leg < 4; leg++)
    {
        std::cout << "Leg " << leg+1 << ": ";
        for(int axis = 0; axis < 3; axis++)
        {
            f[leg][axis] = q_soln[leg*3 + axis];
            std::cout << f[leg][axis] << "\t";
        }
        std::cout << std::endl;
    }

    free(H_qpoases);
    free(g_qpoases);
    free(A_qpoases);
    free(lb_qpoases);
    free(ub_qpoases);
    free(q_soln);

    free(H_red);
    free(g_red);
    free(A_red);
    free(lb_red);
    free(ub_red);
    free(q_red);
}

void A1MPCController::computeControlInput() {
    getJacobian(robotJacobian[0], position[ 7],position[ 8],position[ 9],1);
    getJacobian(robotJacobian[1], position[10],position[11],position[12],-1);
    getJacobian(robotJacobian[2], position[13],position[14],position[15],1);
    getJacobian(robotJacobian[3], position[16],position[17],position[18],-1);

    for(int i=0; i<4; i++)
    {
        robotJacobian[i].transposeInPlace();
        robottorque[i] = robotJacobian[i]*f[i];
        torque[i*3+6] = 0;
        torque[i*3+7] = robottorque[i][1];
        torque[i*3+8] = robottorque[i][2];
    }
}

void A1MPCController::setControlInput() {
    for (int i = 0; i < 18; i++) {
        if(torque[i] > torqueLimit)
        {
            torque[i] = torqueLimit;
        }
        else if(torque[i] < -torqueLimit)
        {
            torque[i] = -torqueLimit;
        }
    }
    std::cout << torque << std::endl;
    getRobot()->robot->setGeneralizedForce(torque);
}

//=============================================================
template <class T>
T t_min(T a, T b)
{
    if(a<b) return a;
    return b;
}

template <class T>
T sq(T a)
{
    return a*a;
}

inline Eigen::Matrix<double,3,3> getSkew(Eigen::Matrix<double,3,1> r)
{
    Eigen::Matrix3d cm;
    cm << 0.f, -r(2), r(1),
            r(2), 0.f, -r(0),
            -r(1), r(0), 0.f;
    return cm;
}

void A1MPCController::quat_to_euler(Eigen::Matrix<double,4,1>& quat, Eigen::Matrix<double,3,1>& q)
{
    //edge case!
    float as = t_min(-2.*(quat[1]*quat[3]-quat[0]*quat[2]),.99999);
    q(0) = atan2(2.f*(quat[2]*quat[3]+quat[0]*quat[1]),sq(quat[0]) - sq(quat[1]) - sq(quat[2]) + sq(quat[3]));
    q(1) = asin(as);
    q(2) = atan2(2.f*(quat[1]*quat[2]+quat[0]*quat[3]),sq(quat[0]) + sq(quat[1]) - sq(quat[2]) - sq(quat[3]));
}

void A1MPCController::ss_mats(Eigen::Matrix<double,13,13>& A, Eigen::Matrix<double,13,12>& B){
    A.setZero();
    A(3,9) = 1.f;
    A(4,10) = 1.f;
    A(5,11) = 1.f;
    A(11,12) = 1.f;

    Eigen::Matrix<double,3,3> I_world;
    I_world << 0.01683993, 0, 0,
               0, 0.056579028, 0,
               0, 0, 0.064713601;
    Eigen::Matrix<double,3,3> I_inv = I_world.inverse();

    auto FRfootFrameIndex = getRobot()->robot->getFrameIdxByName("FR_foot_fixed");
    auto FLfootFrameIndex = getRobot()->robot->getFrameIdxByName("FL_foot_fixed");
    auto RRfootFrameIndex = getRobot()->robot->getFrameIdxByName("RR_foot_fixed");
    auto RLfootFrameIndex = getRobot()->robot->getFrameIdxByName("RL_foot_fixed");

    //Get foot position on the world frame
    raisim::Vec<3> footPosition[4];
    getRobot()->robot->getFramePosition(FRfootFrameIndex, footPosition[0]);
    getRobot()->robot->getFramePosition(FLfootFrameIndex, footPosition[1]);
    getRobot()->robot->getFramePosition(RRfootFrameIndex, footPosition[2]);
    getRobot()->robot->getFramePosition(RLfootFrameIndex, footPosition[3]);

    //Get foot position on the body frame
    for(int i=0; i<4; i++) {
        for (int j = 0; j < 3; j++){
            footPosition[i][j] -= p[j];
        }
    }
    Eigen::Matrix<double,4,3> R_feet;
    R_feet <<  footPosition[0][0], footPosition[0][1], footPosition[0][2], //FR
               footPosition[1][0], footPosition[1][1], footPosition[1][2], //FL
               footPosition[2][0], footPosition[2][1], footPosition[2][2], //RR
               footPosition[3][0], footPosition[3][1], footPosition[3][2]; //RL
    B.setZero();
    for(int n=0; n<4; n++)
    {
        B.block(6,n*3,3,3) = I_inv*getSkew(R_feet.row(n));
        B.block(9,n*3,3,3) = Eigen::Matrix3d::Identity() / mLumpedMass;
    }
}

void A1MPCController::c2qp(Eigen::Matrix<double,13,13> A, Eigen::Matrix<double,13,12> B)
{
    Eigen::Matrix<double,25,25> ABc, expmm;
    Eigen::Matrix<double,13,13> Adt;
    Eigen::Matrix<double,13,12> Bdt;

    ABc.setZero();
    ABc.block(0,0,13,13) = A;
    ABc.block(0,13,13,12) = B;
    ABc = mDT*ABc;
    expmm = ABc.exp();

    Adt = expmm.block(0,0,13,13);
    Bdt = expmm.block(0,13,13,12);

    Eigen::Matrix<double,13,13> D[20];
    D[0].setIdentity();
    for(int i=1; i<=mMPCHorizon; i++)
    {
        D[i] = Adt * D[i-1];
    }

    for(int r=0; r<mMPCHorizon; r++)
    {
        Aqp.block(13*r,0,13,13) = D[r+1];
        for(int c=0; c<mMPCHorizon; c++)
        {
            if(r>=c)
            {
                int a_num = r-c;
                Bqp.block(13*r,12*c,13,12) = D[a_num]*Bdt;
            }
        }
    }
}

void A1MPCController::getJacobian(Eigen::Matrix<double,3,3>& J, double hip, double thigh, double calf, bool side){
    double s1 = std::sin(hip);
    double s2 = std::sin(thigh);
    double s3 = std::sin(calf);

    double c1 = std::cos(hip);
    double c2 = std::cos(thigh);
    double c3 = std::cos(calf);

    double s32 = s3*c2-c3*s2;
    double c32 = c3*c2+s3*s2;

    J << 0,                             -l2*c2-l3*c32,        l3*c32,
         side*l1*s1+l2*c1*c2+l3*c1*c32, -l2*s1*s2+l3*s1*s32, -l3*s1*s32,
         (-1)*side*l1*c1-l2*s1*c2-l3*s1*c32, -l2*c1*s2+l3*c1*s32, -l3*c1*s32;

}


//=============================================================