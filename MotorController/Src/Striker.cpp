//
// Created by Raghavasimhan Sankaranarayanan on 11/3/20.
//

#include "Striker.h"

Striker::Striker()
{
    setDefaultParameters();
}

Striker::~Striker() = default;

unsigned long Striker::getAcceleration(int x) {
    return (unsigned int) round((((x * -.714) - 59.29) * -27.78) + 833.33);
}

Error_t Striker::openDevice()
{
    int lResult = MMC_FAILED;
    unsigned int errorCode = 0;
    char *pDeviceName = new char[255];
    char *pProtocolStackName = new char[255];
    char *pInterfaceName = new char[255];
    char *pPortName = new char[255];

    strcpy(pDeviceName, m_strDeviceName.c_str());
    strcpy(pProtocolStackName, m_strProtocolStackName.c_str());
    strcpy(pInterfaceName, m_strInterfaceName.c_str());
    strcpy(pPortName, m_strPortName.c_str());

    LOG_DEBUG("Opening device...");
    m_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, &errorCode);

    if (m_pKeyHandle != nullptr && errorCode == 0) {
        LOG_INFO("Successfully connected to the Maxon motor.");
        unsigned int lBaudrate = 0;
        unsigned int lTimeout = 0;

        if (VCS_GetProtocolStackSettings(m_pKeyHandle, &lBaudrate, &lTimeout, &errorCode) != 0) {
            if (VCS_SetProtocolStackSettings(m_pKeyHandle, m_iBaudrate, lTimeout, &errorCode) != 0) {
                if (VCS_GetProtocolStackSettings(m_pKeyHandle, &lBaudrate, &lTimeout, &errorCode) != 0) {
                    if (m_iBaudrate == lBaudrate) {
                        lResult = MMC_SUCCESS;
                    }
                }
            }
        }
    } else {
        m_pKeyHandle = nullptr;
    }

    delete[]pDeviceName;
    delete[]pProtocolStackName;
    delete[]pInterfaceName;
    delete[]pPortName;

    if (lResult != MMC_SUCCESS)
        return kNotInitializedError;

    return kNoError;
}

Error_t Striker::closeDevice()
{
    unsigned int errorCode = 0;
    Error_t err = kNoError;
    if (VCS_SetDisableState(m_pKeyHandle, m_iNodeID, &errorCode) == 0) {
        LOG_ERROR("VCS_SetDisableState");
        err = kSetValueError;
    }

    if (VCS_CloseDevice(m_pKeyHandle, &errorCode) == 0) {
        LOG_ERROR("VCS_CloseDevice");
        err = kFileCloseError;
    }
    LOG_DEBUG("Close device");
    return err;
}

void Striker::setDefaultParameters()
{
    m_iNodeID = 1;
    m_strDeviceName = "EPOS4";
    m_strProtocolStackName = "MAXON SERIAL V2";
    m_strInterfaceName = "USB";
    m_strPortName = "USB0";
    m_iBaudrate = 1000000;
}

Error_t Striker::prepare()
{
    unsigned int errorCode = 0;
    auto err = clearFaultState();
    if (err != kNoError)
        return err;

    BOOL oIsEnabled = 0;
    if (VCS_GetEnableState(m_pKeyHandle, m_iNodeID, &oIsEnabled, &errorCode) == 0) {
        LOG_ERROR("VCS_GetEnableState");
        return kGetValueError;
    }

    if (!oIsEnabled) {
        if (VCS_SetEnableState(m_pKeyHandle, m_iNodeID, &errorCode) == 0) {
            LOG_ERROR("VCS_SetEnableState");
            return kSetValueError;
        } else if (setHome() != kNoError) {
            LOG_ERROR("setHome");
            return kSetValueError;
        }
    }

    if (VCS_ActivateProfilePositionMode(m_pKeyHandle, m_iNodeID, &errorCode) == 0) {
        LOG_ERROR("VCS_ActivateProfilePositionMode");
        return kSetValueError;
    }

    return kNoError;
}

Error_t Striker::setHome()
{
    LOG_DEBUG("Setting Home..");
    unsigned int errorCode = 0;

    if (VCS_DefinePosition(m_pKeyHandle, m_iNodeID, 0, &errorCode) == 0) {
        return kSetValueError;
    }

    return kNoError;
}

Error_t Striker::setControllerGain(unsigned long long iPGain, unsigned long long iDGain)
{
    unsigned int errorCode = 0;

    if (VCS_SetControllerGain(m_pKeyHandle, m_iNodeID, EC_PID_POSITION_CONTROLLER, EG_PIDPC_P_GAIN, iPGain, &errorCode) == 0) {
        LOG_ERROR("VCS_SetControllerGain : P Gain");
        clearFaultState();
        return kSetValueError;
    }

    if (VCS_SetControllerGain(m_pKeyHandle, m_iNodeID, EC_PID_POSITION_CONTROLLER, EG_PIDPC_D_GAIN, iDGain, &errorCode) == 0) {
        LOG_ERROR("VCS_SetControllerGain : D Gain");
        clearFaultState();
        return kSetValueError;
    }

    std::stringstream msg;
    msg << "P Gain set to " << iPGain << ", D Gain set to " << iDGain;

    LOG_TRACE(msg.str());
    return kNoError;
}




Error_t Striker::moveToPosition(int position, unsigned long acc, bool wait) {
    unsigned int errorCode = 0;

    std::stringstream msg;
    msg << "move to position = " << position << ", acc = " << acc;
    LOG_TRACE(msg.str());

    if (VCS_SetPositionProfile(m_pKeyHandle, m_iNodeID, 3000, acc, acc, &errorCode) == 0) {
        LOG_ERROR("VCS_SetPositionProfile");
        return kSetValueError;
    }

    if (VCS_MoveToPosition(m_pKeyHandle, m_iNodeID, position, 1, 1, &errorCode) == 0) {
        LOG_ERROR("VCS_MoveToPosition");
        return kSetValueError;
    }

    if (wait)
    {
        if (VCS_WaitForTargetReached(m_pKeyHandle, m_iNodeID, 100, &errorCode) == 0)
        {
            LOG_ERROR("VCS_WaitForTargetReached");
            return kGetValueError;
        }
    }

    return kNoError;
}

Error_t Striker::strike(int iMidiVelocity)
{
    auto acc = getAcceleration(iMidiVelocity);

    auto err = moveToPosition(TARGET_POSITION, acc, true);
    if (err != kNoError) {
        LOG_ERROR("MoveToPosition");
        return kNoError;
    }

    err = moveToPosition(0, acc, false);
    if (err != kNoError) {
        LOG_ERROR("MoveToPosition");
        return err;
    }

    return kNoError;
}

Error_t Striker::clearFaultState()
{
    BOOL oIsFault = 0;
    unsigned int errorCode = 0;

    if (VCS_GetFaultState(m_pKeyHandle, m_iNodeID, &oIsFault, &errorCode) == 0) {
        LOG_ERROR("VCS_GetFaultState");
        return kGetValueError;
    }

    if (oIsFault) {
        std::stringstream msg;
        msg << "clear fault, node = '" << m_iNodeID << "'";
        LOG_TRACE(msg.str());
        if (VCS_ClearFault(m_pKeyHandle, m_iNodeID, &errorCode) == 0) {
            LOG_ERROR("VCS_ClearFault");
            return kSetValueError;
        }
    }

    return kNoError;
}
