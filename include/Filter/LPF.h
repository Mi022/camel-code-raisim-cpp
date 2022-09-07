//
// Created by jaehoon on 22. 7. 22.
//

#ifndef RAISIM_LPF_H
#define RAISIM_LPF_H

#include <iostream>

class LPF {
public:
    void initialize(double dt, double fc); //처음 생성할때 dt, fc 를 설정해줌
    double doFiltering(double xin);
    void setCutoffFreq(double fc); //추후에 fc 만 임의로 바꾸고 싶을때
    void showParameters(); //지워도 됨

private:
    bool firstRun= true;
    double dt_;
    double fc_;
    double alpha;
    double prevX;
    double xlpf;


};


#endif //RAISIM_LPF_H
