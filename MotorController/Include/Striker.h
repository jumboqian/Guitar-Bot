//
// Created by Raghavasimhan Sankaranarayanan on 11/3/20.
//

#ifndef GOOGLEDRUMMINGARM_STRIKER_H
#define GOOGLEDRUMMINGARM_STRIKER_H

#include "Definitions.h"
#include "ErrorDef.h"
#include "Defines.h"
#include "pch.h"
#include "Logger.h"
#include <cstring>

#ifndef MMC_SUCCESS
#define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
#define MMC_FAILED 1
#endif

#ifndef MMC_MAX_LOG_MSG_SIZE
#define MMC_MAX_LOG_MSG_SIZE 512
#endif

typedef void *HANDLE;
typedef int BOOL;

class Striker
{
public:
    Striker();
    ~Striker();

    Error_t openDevice();
    Error_t closeDevice();
    void setDefaultParameters();

    Error_t prepare();
    Error_t setHome();

    Error_t clearFaultState();

    Error_t setControllerGain(unsigned long long iPGain, unsigned long long iDGain);

    Error_t moveToPosition(int position, unsigned long acc, bool wait = true);
    Error_t strike(int m_velocity = DEFAULT_MIDI_VELOCITY);

private:
    static unsigned long getAcceleration(int x);

    HANDLE m_pKeyHandle = nullptr;
    uint16_t m_iNodeID = 1;
    std::string m_strDeviceName;
    std::string m_strProtocolStackName;
    std::string m_strInterfaceName;
    std::string m_strPortName;
    unsigned int m_iBaudrate = 0;
};


#endif //GOOGLEDRUMMINGARM_STRIKER_H
