
//
// Created by Raghavasimhan Sankaranarayanan on 5/8/21.
//

#ifndef GUITARBOT_RIGHTHANDCONTROLLER_H
#define GUITARBOT_RIGHTHANDCONTROLLER_H

#include <thread>
#include <atomic>
#include "PandaArm.h"
#include "MotorController.h"
#include "ErrorDef.h"

class RightHandController {
public:

    Error_t init(const std::string& host);
    Error_t start(double timeout_sec);
    void stop();

    Error_t start_motor(double timeout_sec);
    void stop_motor();

    Error_t setPickStiffness(uint8_t velocity);
    Error_t setMotorPosition(float value);
    Error_t setMotorStatus(bool bool_);
    Error_t startRTLoop(double timeout_sec);
//    void setMotionStat_m(PandaArm::MotionStat stat);
    void setMotionStat_p(PandaArm::MotionStat stat);
    void setMotionStat_t(PandaArm::MotionStat stat);
    void setMotionStat_d(PandaArm::MotionStat stat);
    void setOmega(float value);
    void setTrackStat(PandaArm::TrackStat stat);
    // void setMotionStat_d(PandaArm::MotionStat stat);
    void setMoveParameter(struct PandaArm::MoveParam param);
    void desiredMidiVelocity(uint8_t vel_d);
    void readMidiVelocity(uint8_t vel_a);
    void readTimbreStatus(double value);
    void readDepthStatus(double value);
    void initNewTrack();

private:
    void pandaThreadHandler(double timeout_sec);

    void motorThreadHandler(double timeout_sec);

    bool m_bInitialized;
    PandaArm m_pandaArm;
    MotorController m_motorcontroller;
    Striker m_striker;
    std::unique_ptr<std::thread> m_pPandaThread;
    std::atomic<double> m_execTime = 0.0;

    float omega = 30;
    std::unique_ptr<std::thread> m_pMotorThread;
    bool m_bMotorRunning = false;
};


#endif //GUITARBOT_RIGHTHANDCONTROLLER_H
