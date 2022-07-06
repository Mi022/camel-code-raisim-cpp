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
                std::cout<<"LineFeedCode!"<<std::endl;
                break;
            } else if (mReadBuf[0] == mCarraigeReturnCode) {
                mIsDataStore = false;
                std::cout<<"mCarraigeReturnCode!"<<std::endl;
            } else if (mIsDataStore) {
                mReaded[mIdx] = (mReadBuf[0] - 48);
                mIdx++;
            }
        }
    }
    for (int i = 0; i < mIdx; i++) {
        mReadedData += mReaded[i] * pow(10.0, mIdx - i - 1);
    }

    lowPassFilter();

    std::cout<<"mReadedData : "<<mReadedData<<std::endl;

    // reset
    mIdx = 0;
    mIsDataStore = true;
}

void RE22SC::lowPassFilter() {
    //TODO: filter the mReadData with low pass filter
}

void RE22SC::flushData(int num) {
    while(true){
        readData();
    }
}