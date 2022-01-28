
//
// Created by Raghavasimhan Sankaranarayanan on 5/8/21.
//

#include "RightHandController.h"


Error_t RightHandController::init(const std::string& host) {
    if (m_motorcontroller.prepareStriker() !=0){
        return kNotInitializedError;
    }
    if (m_pandaArm.init(host) != kNoException)
        return kPandaError;
    m_bInitialized = true;
    return kNoError;


}

Error_t RightHandController::start(double timeout_sec) {
    if (!m_bInitialized) {
        std::cerr << "Right hand controller not initialized" << std::endl;
        return kNotInitializedError;
    }

    m_pPandaThread = std::make_unique<std::thread>(&RightHandController::pandaThreadHandler, this, timeout_sec);

    return kNoError;
}

void RightHandController::stop() {
    m_pandaArm.stopRTLoop();
    //m_motorcontroller.stop();
    if (m_pPandaThread) {
        if (m_pPandaThread->joinable())
            m_pPandaThread->join();
    }
}

Error_t RightHandController::start_motor(double timeout_sec) {
    m_pMotorThread = std::make_unique<std::thread>(&RightHandController::motorThreadHandler, this, timeout_sec);
    this->m_bMotorRunning = true;
    m_pMotorThread->detach();
    LOG_INFO("MOTOR START");

}

void RightHandController::stop_motor() {
    LOG_INFO("MOTOR STOP");
    this->m_bMotorRunning = false;
    if (m_pMotorThread) {
        if (m_pMotorThread->joinable()) {

            m_pMotorThread->join();
            LOG_INFO("MOTOR STOP2");
        }
    }
}

void RightHandController::motorThreadHandler(double timeout_sec) {
    float position;

    float time = 0.0;

    while (this->m_bMotorRunning) {

        if (cos(omega * time)==1.0){
            RightHandController::setMotorPosition(-100.0);
        }
        else if (cos(omega * time)==-1.0){
            RightHandController::setMotorPosition(-500.0);
        }
//        position = round(200* cos(omega * time))-300;
//        RightHandController::setMotorPosition(position);
        time = time + 0.01;
        }
    RightHandController::setMotorPosition(0); // return to neutral position


}

void RightHandController::pandaThreadHandler(double timeout_sec) {
    m_pandaArm.startRTLoop(timeout_sec);
}

void RightHandController::setOmega(float value) {
    this->omega = value;
    LOG_INFO("motor to value");
    LOG_INFO(value);
}

void RightHandController::setMotionStat_p(PandaArm::MotionStat stat) {
    if (m_pandaArm.m_motionStat_p == PandaArm::Running) {
        return;
    }
    m_pandaArm.setMotionStat_p(stat);
}

//void RightHandController::setMotionStat_m(PandaArm::MotionStat stat) {
//    if (m_pandaArm.m_motionStat_m == PandaArm::Running) {
//        return;
//    }
//    m_pandaArm.setMotionStat_m(stat);
//}

void RightHandController::setMotionStat_t(PandaArm::MotionStat stat) {
    if (m_pandaArm.m_motionStat_t == PandaArm::Running) {
        return;
    }
    m_pandaArm.setMotionStat_t(stat);
}

void RightHandController::setMotionStat_d(PandaArm::MotionStat stat) {
    if (m_pandaArm.m_motionStat_d == PandaArm::Running) {
        return;
    }
    m_pandaArm.setMotionStat_d(stat);
}

void RightHandController::setTrackStat(PandaArm::TrackStat stat) {
    if (m_pandaArm.m_trackStat == PandaArm::Next) {
        return;
    }
    m_pandaArm.setTrackStat(stat);
}


void RightHandController::setMoveParameter(struct PandaArm::MoveParam param){
     m_pandaArm.setMoveParameter(param);
}

Error_t RightHandController::setPickStiffness(uint8_t velocity) {
    m_motorcontroller.changeGrip(velocity);
    return kNoError;
}

Error_t RightHandController::setMotorPosition(float value) {
    m_motorcontroller.changePosition(value);
    return kNoError;
}

void RightHandController::readMidiVelocity(uint8_t vel_a) {
    m_pandaArm.readMidiVelocity(vel_a);
}

void RightHandController::desiredMidiVelocity(uint8_t vel_d) {
    m_pandaArm.desiredMidiVelocity(vel_d);
}

void RightHandController::readTimbreStatus(double value) {
    m_pandaArm.readTimbreStatus(value);

}

void RightHandController::readDepthStatus(double value) {
    m_pandaArm.readDepthStatus(value);

}

void RightHandController::initNewTrack()
{
    m_pandaArm.newTrackFlag = 1;
}


