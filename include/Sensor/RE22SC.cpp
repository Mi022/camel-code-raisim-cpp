//
// Created by user on 22. 6. 30.
//

#include "RE22SC.h"

void RE22SC::readData() {
    mReadedData = 0;
    while (true) {
        mNumBytes = read(mSerialPort, &mReadBuf, sizeof(mReadBuf));
        if (mNumBytes == 1) {
            if (mReadBuf[0] == mLineFeedCode) {
                break;
            } else if (mReadBuf[0] == mCarraigeReturnCode) {
                mIsDataStore = false;
            } else if (mIsDataStore) {
                mReaded[mIdx] = (mReadBuf[0] - 48);
                mIdx++;
            }
        }
    }
    for (int i = 0; i < mIdx; i++) {
        mReadedData += mReaded[i] * pow(10.0, mIdx - i - 1);
    }
    mRawData = mReadedData;
    lowPassFilter();

    std::cout<<"ReadedData : "<<mReadedData<<std::endl;
    std::cout<<"DegreeData : "<<getDegreeData()<<std::endl;
    std::cout<<"RadianData : "<<getRadianData()<<std::endl;

    // reset
    mIdx = 0;
    mIsDataStore = true;
}

void RE22SC::lowPassFilter() {
    if(mFilterFlag)
    {
        mReadedData = BETA*mpreviousData + (1-BETA)*mReadedData;
    }
    else mFilterFlag = true;
    mpreviousData = mReadedData;
}

void RE22SC::flushData() {
    while(true){
        readData();
    }
}

double RE22SC::getRawDegreeData() {
    return mRawData*360.0/1024.0;
}

double RE22SC::getRawRadianData() {
    return mRawData*2.0*M_PI/1024.0;
}

double RE22SC::getDegreeData() {
    return mReadedData*360.0/1024.0;
}

double RE22SC::getRadianData() {
    return mReadedData*2.0*M_PI/1024.0;
}