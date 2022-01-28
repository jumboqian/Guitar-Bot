//
// Created by Raghavasimhan Sankaranarayanan on 11/3/20.
//

#ifndef GOOGLEDRUMMINGARM_MOTORCONTROLLER_H
#define GOOGLEDRUMMINGARM_MOTORCONTROLLER_H

#include "ErrorDef.h"
#include "pch.h"
#include "Definitions.h"
#include "Logger.h"
#include "Util.h"
#include "Defines.h"
#include "Striker.h"
#include "time.h"

class MotorController
{
public:

    MotorController();
    ~MotorController();

    void setGripMap(const std::vector<int>& gripMap);

    Error_t prepareStriker();
    Error_t changeGrip(uint8_t velocity);
    Error_t changePosition(float value);

    Error_t strike(int velocity);
    void setEnabled(bool bEnable);
    void stop();

private:
    typedef std::chrono::high_resolution_clock Clock;

    bool m_MotorStatus = false;
    bool m_bDeviceOpened = false;
    Striker m_striker;

    std::vector<int> m_gripMap;

    std::atomic<bool> m_bIsEnabled;
};


#endif //GOOGLEDRUMMINGARM_MOTORCONTROLLER_H
