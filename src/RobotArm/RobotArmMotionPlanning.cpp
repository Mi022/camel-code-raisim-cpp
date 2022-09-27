//
// Created by jy on 22. 7. 6.
//

#include "RobotArmMotionPlanning.h"
#include "random"
#include "algorithm"

using namespace std;

void RobotArmMotionPlanning::generatePoint()
{
    int randNum = 500;
    armPose.row(0) = startJoint;
    int currentIndex = 0;
    for (int i = 0; i < randNum; i++)
    {
        Eigen::MatrixXd joint = 180 * Eigen::MatrixXd::Random(1, 6);
//        for (int j = 0; j < 6; ++j) {
//            joint.col(j) = (max(abs(startJoint(j)),abs(goalJoint(j))) + 20.0)*Eigen::VectorXd::Random(1);
//        }
        if (collisionChecker->jointChecker(joint))
        {
            currentIndex += 1;
            armPose.conservativeResize(armPose.rows() + 1, armPose.cols());
            armPose.row(currentIndex) = joint;
        }
        else
        {
            continue;
        }
    }
    armPose.conservativeResize(armPose.rows() + 1, armPose.cols());
    armPose.row(currentIndex + 1) = goalJoint;

    cout << "pose number" << endl;
    cout << armPose.rows() << endl;

    std::cout << endl << "The end Generate Point " << std::endl;
}

void RobotArmMotionPlanning::makeTree()
{
    generatePoint();
    float jointDistance = 400;
    float eeDistance = 0.5;
    Eigen::MatrixXd currentPoint;
    Eigen::MatrixXd nextPoint;
    float currentJointDistance;
    float currentEEDistance;
    int count = 0;
    int treeNum;
    len = armPose.rows();
    childTree.conservativeResize(len, len);
    childTree.setZero();
    treeStart = time(NULL);
    for (int i = 0; i < len; i++)
    {
        treeNum = 0;
        for (int j = 0; j < len; j++)
        {
            currentJointDistance = distance.distance(armPose.row(i), armPose.row(j));
            if (currentJointDistance > 0.00001 and currentJointDistance < jointDistance)
            {
                currentPoint = forwardKinematics.forwardKinematics(armPose.row(i));
                nextPoint = forwardKinematics.forwardKinematics(armPose.row(j));
                currentEEDistance = distance.distance(currentPoint.row(6), nextPoint.row(6));
                if (currentEEDistance > 0.00001 and currentEEDistance < eeDistance)
                {
                    pareAdd(0, 0) = i;
                    pareAdd(0, 1) = j;
                    pareAdd(0, 2) = currentJointDistance;
                    pare.row(count) = pareAdd;
                    pare.conservativeResize(pare.rows() + 1, pare.cols());
                    count++;
                    childTree(i, treeNum) = j;
                    treeNum++;
                }
            }
        }
    }

    pare.conservativeResize(pare.rows() - 1, pare.cols());

    treeEnd = time(NULL);

    std::cout << "pare" << std::endl;
    std::cout << pare.rows() << std::endl;

//    std::cout << "childTree" << std::endl;
//    std::cout << childTree << std::endl;

    std::cout << endl << "The end Make Tree" << std::endl;
    std::cout << "The time is " << (double)(treeEnd - treeStart) << "ms" << std::endl;
}

void RobotArmMotionPlanning::dijkstra()
{

    searchStart = time(NULL);

    vector<int> Q(len);
    for (int i = 0; i < len; i++)
    {
        Q[i] = i;
    }
    float largeN = 1e5;

    vector<int> S;
    Eigen::MatrixXd Uweight = Eigen::MatrixXd(len, len);
    Uweight.setOnes();
    Uweight = largeN * Uweight;
    vector<float> uD(len, largeN);
    int iteration = 0;
    float limitIter = 1e4;

    int coordVertex1;
    int coordVertex2;
    float distanceVertex;
    for (int rowIdx = 0; rowIdx < pare.rows(); rowIdx++)
    {
        coordVertex1 = pare(rowIdx, 0);
        coordVertex2 = pare(rowIdx, 1);
        distanceVertex = pare(rowIdx, 2);
        Uweight(coordVertex1, coordVertex2) = distanceVertex;
    }

    Eigen::VectorXd parentTree = Eigen::VectorXd::Zero(len);
    Eigen::VectorXd uIdxChild;
    Uweight(0, 0) = 0;
    uD[0] = 0;
    int Uidx;
    vector<int> UidxBox;
    vector<int> testBox;
    vector<int> result;
    vector<float> findMin;
    vector<int>::iterator itr;

    float minD = 0;

    while (!Q.empty())
    {
        if (iteration > limitIter)
        {
            break;
        }
        findMin.clear();
        for (int i = 0; i < Q.size(); i++)
        {
            findMin.push_back(uD[Q[i]]);
        }
        minD = *min_element(findMin.begin(), findMin.end());
        UidxBox.clear();
        for (int i = 0; i < uD.size(); i++)
        {
            if (uD[i] == minD)
            {
                UidxBox.push_back(i);
            }
        }

        Uidx = UidxBox[rand() % UidxBox.size()];

        S.clear();
        S.push_back(Uidx);
        result.clear();
        result.resize(Q.size() + S.size());
        itr = set_difference(Q.begin(), Q.end(), S.begin(), S.end(), result.begin());//차집합
        result.erase(itr, result.end());
        Q = result;

        uIdxChild = childTree.row(Uidx);

        for (int i = 0; i < uIdxChild.size(); i++)
        {
            int vIdx = uIdxChild[i];
            if (uD[vIdx] > (uD[Uidx] + Uweight(Uidx, vIdx)))
            {
                uD[vIdx] = uD[Uidx] + Uweight(Uidx, vIdx);
                parentTree[vIdx] = Uidx;
            }
        }

        iteration++;
    }

    int find_idx = parentTree.size() - 1;
    float jointDistanceSum = 0;
    findTree.push_back(find_idx);

    while (find_idx != 0)
    {
        find_idx = parentTree[find_idx];
        findTree.push_back(find_idx);
    }
    reverse(findTree.begin(), findTree.end());

    for (int i = 0; i < findTree.size(); i++)
    {
        cout << "findTree " << i << endl;
        cout << armPose.row(findTree[i]) << endl;
    }
    Eigen::MatrixXd endEffector;
    wayPoints.conservativeResize(findTree.size(), 6);
    for (int i = 0; i < findTree.size(); i++)
    {
        wayPoints.row(i) = armPose.row(findTree[i]);
    }
    for (int i = 0; i < findTree.size() - 1; i++)
    {
        jointDistanceSum = jointDistanceSum + distance.distance(wayPoints.row(i), wayPoints.row(i + 1));
    }
    jointDistanceSum = jointDistanceSum + distance.distance(wayPoints.row(wayPoints.rows() - 2), wayPoints.row(wayPoints.rows() - 1));

    cout << "jointDistance : " << jointDistanceSum << endl;

    searchEnd = time(NULL);

    std::cout << "The end Find Trajectory" << std::endl;
    std::cout << "The time is " << (double)(searchEnd - searchStart) << "ms" << std::endl;

    trajectoryGenerator->setWaypoints(wayPoints);

}