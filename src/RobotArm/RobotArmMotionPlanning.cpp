//
// Created by jy on 22. 7. 6.
//

#include "RobotArmMotionPlanning.h"
#include "random"
#include "algorithm"
using namespace std;

void RobotArmMotionPlanning::setObstacle(Eigen::VectorXd obstacleRadius, Eigen::MatrixXd obstacleCenter) {
    mObstacleCenter = obstacleCenter;
    mObstacleRadius = obstacleRadius;
//    std::cout << mObstacleCenter(0,2) << std::endl;
}

void RobotArmMotionPlanning::generatePoint() {
    int randNum = 100;
    armPose.row(0)=startJoint;
    int currentIndex = 0;
    for(int i=0 ; i<randNum ; i++){
        Eigen::MatrixXd joint = 180*Eigen::MatrixXd::Random(1,6);
        if(collisionChecker.jointChecker(joint)){
            currentIndex += 1;
            armPose.conservativeResize(armPose.rows()+1, armPose.cols());
            armPose.row(currentIndex)=joint;
        }
        else
            continue;
    }
    armPose.conservativeResize(armPose.rows()+1, armPose.cols());
    armPose.row(currentIndex+1) = goalJoint;

}

void RobotArmMotionPlanning::makeTree() {

    generatePoint();
    int k=250;
    float lambda1;
    float lambda2;
    Eigen::MatrixXd currentPoint = Eigen::MatrixXd::Zero(6,3);
    Eigen::MatrixXd nextPoint = Eigen::MatrixXd::Zero(6,3);
    float currentDistance;
    int count = 0;
    int treeNum;
    len = armPose.rows();
    childTree.conservativeResize(len,len);
    childTree.setZero();

    for(int i=0; i<len ; i++){
        treeNum = 0;
        for(int j=0 ; j<len ; j++){
            currentPoint = forwardKinematics.forwardKinematics(armPose.row(i));
            nextPoint = forwardKinematics.forwardKinematics(armPose.row(j));
            currentDistance = distance.distance(armPose.row(i),armPose.row(j));
//            cout << armPose.row(i) << armPose.row(j) << endl;
//            cout << currentDistance << endl;
            if( currentDistance > 0.01 and currentDistance < k and collisionChecker.lineChecker(currentPoint.row(0),nextPoint.row(1)))
//            if( currentDistance > 0.1 and currentDistance < k)
            {
                pareAdd(0,0) = i;
                pareAdd(0,1) = j;
                pareAdd(0,2)=currentDistance;
                pare.row(count)=pareAdd;
                pare.conservativeResize(pare.rows()+1,pare.cols());
                count ++;
                childTree(i,treeNum) = j;
                treeNum++;
            }
        }
    }

//    cout << "childTree :  "<<endl;
//    cout <<  childTree <<  endl;
    pare.conservativeResize(pare.rows()-2,pare.cols());
    pare = removeMatrix.removeRow(pare,0);
}

void RobotArmMotionPlanning::dijkstra() {

    vector<int> Q(len);
    for (int i = 0; i < len; i++) {
        Q[i] = i;
    }
    float largeN = 1e5;

    vector<int> S;
    Eigen::MatrixXd Uweight = Eigen::MatrixXd(len,len);
    Uweight.setOnes();
    Uweight = largeN*Uweight;
    vector<float> uD(len,largeN);
    int iteration = 0;
    float limitIter = 1e4;

    int coordVertex1;
    int coordVertex2;
    float distanceVertex;
    for(int rowIdx =0 ; rowIdx < pare.rows() ; rowIdx++){
        coordVertex1 = pare(rowIdx,0);
        coordVertex2 = pare(rowIdx,1);
        distanceVertex = pare(rowIdx,2);
        Uweight(coordVertex1,coordVertex2) = distanceVertex;
    }
    cout << childTree << endl;

    Eigen::VectorXd parentTree = Eigen::VectorXd::Zero(len);

    Uweight(0,0)=0;
    uD[0] = 0;
    int Uidx ;
    vector<int> UidxBox ;
    vector<int> testBox ;

    vector<int> result;
    Eigen::VectorXd uIdxChild;
    vector<int> uRealChild;
    vector<float> findMin;
    vector<int>::iterator itr;
    float minD = 0;

    while (!Q.empty()){
        if(iteration > limitIter){
            break;
        }
        findMin.clear();
        for(int i =0 ; i < Q.size() ; i++){
            findMin.push_back(uD[Q[i]]);
        }
        minD = *min_element(findMin.begin(),findMin.end());
        UidxBox.clear();
        for(int i=0;i<uD.size();i++)
        {
            if (uD[i] == minD) {
                UidxBox.push_back(i);
            }
        }

//        cout << "findMin : " << endl;
//        for(int i =0 ; i < findMin.size() ; i++){
//            cout << " " << findMin[i] << " ";
//        }
//        cout << " " << endl;
//
//        cout << "minD : " << endl;
//        cout << minD << endl;
//
//        cout << "UidxBox : " << endl;
//        for(int i =0 ; i < UidxBox.size() ; i++){
//            cout << " " << UidxBox[i] << " ";
//        }
//        cout << " " << endl;

        Uidx = UidxBox[rand() % UidxBox.size()];

        S.clear();
        S.push_back(Uidx);
        result.clear();
        result.resize(Q.size()+S.size());
        itr = set_difference(Q.begin(), Q.end(), S.begin(), S.end(), result.begin());//차집합
        result.erase(itr, result.end());
        Q = result;

        uIdxChild = childTree.row(Uidx);
        uRealChild.clear();
        for(int i = 0 ; i < uIdxChild.size() ; i++){
            if(uIdxChild[i] > 0){
                uRealChild.push_back(uIdxChild[i]);
            }
        }
        for(int i = 0 ; i < uRealChild.size() ; i++){
            int vIdx = uRealChild[i];
            if( uD[vIdx] > (uD[Uidx] + Uweight(Uidx,vIdx))){
                uD[vIdx] = uD[Uidx] + Uweight(Uidx,vIdx);
                parentTree[vIdx] = Uidx;
            }
        }

        iteration++;
    }
    int find_idx = len-1;
    findTree.push_back(find_idx);
    while (find_idx != 0){
        find_idx = parentTree[find_idx,0];
        findTree.push_back(find_idx);
    }
    reverse(findTree.begin(),findTree.end());

}