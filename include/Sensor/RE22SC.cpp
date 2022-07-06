//
// Created by user on 22. 6. 30.
//

#include "RE22SC.h"

void RE22SC::readData() {
    mReadedData = 0;
    while (true) {
        mNumBytes = read(mSerialPort, &mReadBuf, sizeof(mReadBuf));
//        std::cout<<mReadBuf[0]<<std::endl;
//        printf("data(int) %d data(char) %c",mReadBuf[0], mReadBuf[0]);
        if (mNumBytes == 1) {
            if (mReadBuf[0] == mLineFeedCode) {
//                std::cout<<"LineFeedCode!"<<std::endl;
                break;
            } else if (mReadBuf[0] == mCommaCode) {
                mIsDataStore = false;
//                std::cout<<"CommaCode!"<<std::endl;
            } else if (mReadBuf[0] == mNull) {
                mIsDataStore = false;
//                std::cout<<"mNull!"<<std::endl;
            } else if (mReadBuf[0] == mCarraigeReturnCode) {
                mIsDataStore = false;
//                std::cout<<"mCarraigeReturnCode!"<<std::endl;
            }else if (mReadBuf[0] == mNegativeValueCode) {
                mIsNegativeValue = true;
//                std::cout<<"negativeValueCode!"<<std::endl;
            } else if (mIsDataStore) {
                mReaded[mIdx] = (mReadBuf[0] - 48);
                mIdx++;
//                std::cout<<"data : "<<mReaded[mIdx-1]<<"idx"<<mIdx<<std::endl;
            }
        }
    }
//    std::cout<<"mIdx : "<<mIdx<<std::endl;
    for (int i = 0; i < mIdx; i++) {
        mReadedData += mReaded[i] * pow(10.0, mIdx - i - 1);
    }

    std::cout<<"mReadedData : "<<mReadedData<<std::endl;

    // reset
    mIdx = 0;
    mIsDataStore = true;
    mIsNegativeValue = false;
}

void RE22SC::flushData(int num) {
//    for (int i = 0; i < num; i++) {
//        readData();
//    }
    while(true){
        readData();
    }
}