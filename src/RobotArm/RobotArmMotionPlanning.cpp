//
// Created by jy on 22. 7. 6.
//
#include "iostream"
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
    int randNum = 10;
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
    int k=300;
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

vector<int> RobotArmMotionPlanning::where(Eigen::VectorXd vec, float value) {
    vector<int> findIdx;
    int idx = 0;
    for(idx =0;idx<vec.size();idx++){
        if(vec(idx)==value){
            findIdx.push_back(idx);
        }
    }
    return findIdx;
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
    Eigen::VectorXd Ud = Eigen::VectorXd(pare.rows());
    Ud.setOnes();
    Ud = largeN*Ud;
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

//    cout << " U w " << endl;
//    cout << Uweight << endl;

    Eigen::VectorXd parentTree = Eigen::VectorXd::Zero(len);

    Uweight(0,0)=0;
    Ud(0) = 0;
    int Uidx ;
    vector<int> UidxBox ;
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
            findMin.push_back(Ud[Q[i]]);
        }

        cout << "findMin : " << endl;
        for(int i =0 ; i < findMin.size() ; i++){
            cout << " " << findMin[i] << " ";
        }
        cout << " " << endl;

        minD = *min_element(findMin.begin(),findMin.end());
        UidxBox = where(Ud, minD);

        cout << "UidxBox : " << endl;
        for(int i =0 ; i < UidxBox.size() ; i++){
            cout << " " << UidxBox[i] << " ";
        }
        cout << " " << endl;

        int dd = rand() % UidxBox.size();
        cout << "dd : " << dd <<endl;
        Uidx = UidxBox[dd];
        S.push_back(Uidx);
        result.clear();
        result.resize(Q.size()+S.size());
        itr = set_difference(Q.begin(), Q.end(), S.begin(), S.end(), result.begin());//차집합
        result.erase(itr, result.end());
        Q = result;

        uIdxChild = childTree.row(Uidx);
        for(int i = 0 ; i < uIdxChild.size() ; i++){
            if(uIdxChild[i] > 0){
                uRealChild.push_back(uIdxChild[i]);
            }
        }
        for(int i = 0 ; i < uRealChild.size() ; i++){ //아니 Ud가 왜 갑자기 그냥 뿅뿅 바껴버리냐고ㅋㅋ
            int vIdx = uRealChild[i];
            cout <<"vIdx : " << vIdx << endl;
            if( Ud[vIdx] > (Ud[Uidx] + Uweight(Uidx,vIdx))){
                Ud[vIdx] = Ud[Uidx] + Uweight(Uidx,vIdx);
                parentTree[vIdx] = Uidx;
            }
        }

        iteration++;
//        cout << "parentTree : " << parentTree<< endl;

//        cout << "iteration :  " <<iteration << endl;
    }

}