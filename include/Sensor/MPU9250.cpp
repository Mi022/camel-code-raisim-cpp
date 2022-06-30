//
// Created by user on 22. 6. 23.
//

#include "MPU9250.h"

void MPU9250::readData() {
    mReadedData = 0;
    while (true) {
        mNumBytes = read(mSerialPort, &mReadBuf, sizeof(mReadBuf));
        std::cout<<mReadBuf[0]<<std::endl;
        if (mNumBytes == 1) {
            if (mReadBuf[0] == mLineFeedCode) {
                std::cout<<"LineFeedCode!"<<std::endl;
//                break;
            } else if (mReadBuf[0] == mCommaCode) {
                mIsDataStore = false;
                std::cout<<"CommaCode!"<<std::endl;
            } else if (mReadBuf[0] == mDotCode) {
                mIsDataStore = false;
                std::cout<<"DotCode!"<<std::endl;
            } else if (mReadBuf[0] == mNegativeValueCode) {
                mIsNegativeValue = true;
                std::cout<<"Negative Value!"<<std::endl;
            } else if (mIsDataStore) {
//                mReaded[mIdx] = (mReadBuf[0] - 48);
//                mIdx++;
            }
        }
    }
//
//    for (int i = 0; i < mIdx; i++) {
//        mReadedData += mReaded[i] * pow(10.0, mIdx - i - 1);
//    }
//    if (mIsNegativeValue) {
//        mReadedData = -1.0 * mReadedData;
//    }

    // reset
    mIdx = 0;
    mIsDataStore = true;
    mIsNegativeValue = false;
}

void MPU9250::flushData(int num) {
    for (int i = 0; i < num; i++) {
        readData();
    }
}