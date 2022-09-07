//
// Created by jaehoon on 22. 7. 22.
//

#include "LPF.h"

void LPF::initialize(double dt, double fc) {
    dt_ = dt;
    fc_ = fc;
    alpha = 2*3.141592*fc_*dt_/(2*3.141592*fc_*dt_+1); //alpha = dt/(RC+dt)
    showParameters();
}

//필터링할 값을 xin 에 넣어주면 필터링 결과 xlpf 를 return 함
double LPF::doFiltering(double xin) {
    if(firstRun == true){
        prevX = xin;
        firstRun = false;
    }
    xlpf = alpha * xin + (1-alpha)*prevX ; //yi = xi * a + yi_1 * (1-a)
    prevX = xlpf;
    return xlpf;
}

void LPF::setCutoffFreq(double fc) {
    fc_ = fc;
    alpha = 2*3.141592*fc_*dt_/(2*3.141592*fc_*dt_+1);
}

void LPF::showParameters(){
    std::cout<<"fc : "<< fc_ << " alpha : "<<alpha<<std::endl;
}