//
// Created by Raghavasimhan Sankaranarayanan on 7/1/20.
//

#ifndef GOOGLEDRUMMINGARM_UTIL_H
#define GOOGLEDRUMMINGARM_UTIL_H

#include "pch.h"


//#include "Data.h"
#include "Defines.h"
#include "ErrorDef.h"
//#include "Logger.h"

class Util {
public:
    template<typename T>
    static T constrain(T value, T lower, T upper) {
        return std::min(upper, std::max(lower, value));
    }

    static size_t argmax(const float* pfArray, size_t iLength) {
//        return std::distance(pfArray, std::max_element(pfArray, pfArray + iLength));
        float fVal = pfArray[0];
        size_t iIdx = 0;
        for (size_t i=1; i<iLength; ++i) {
            float fTemp = pfArray[i];
            if (fTemp > fVal)
            {
                iIdx = i;
                fVal = fTemp;
            }
        }
        return iIdx;
    }

    static int mode(int* piArray, size_t iLength) {
        if (iLength < 1)
            return -1;

        else if (iLength == 1 || iLength == 2)
            return piArray[0];

        std::unordered_map<int, int> hash;

        for (size_t i=0; i<iLength; i++)
            hash[piArray[i]]++;

        int max_count = 0, res = -1;
        for (auto i : hash) {
            if (max_count < i.second) {
                res = i.first;
                max_count = i.second;
            }
        }
        return res;
    }

    static void toFloat(float* pfDest, const uint16_t* piSrc, size_t iLength) {
        for (size_t i=0; i<iLength; i++) {
            pfDest[i] = float(piSrc[i] - 32768) / 32768;
        }
    }


//    [[nodiscard]] static Error_t checkFileExt(const std::string& filename, const std::string& extToCheck) {
//        auto ext = std::filesystem::path(filename).extension();
//        if (filename.empty())
//            return kNullError;
//
//        if (ext != extToCheck)
//            return kFileExtensionError;
//
//        return kNoError;
//    }

private:

};

#endif //GOOGLEDRUMMINGARM_UTIL_H
