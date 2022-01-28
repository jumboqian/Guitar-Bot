
//
// Created by Raghavasimhan Sankaranarayanan on 5/8/21.
//

#ifndef GUITARBOT_PANDAARM_H
#define GUITARBOT_PANDAARM_H

#include <stdio.h>
#include <math.h>
#include "iostream"
#include <atomic>
#include "ErrorDef.h"
#include "Logger.h"
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/duration.h>
#include <franka/rate_limiting.h>
#include "MotionGenerator.h"
#include "MotorController.h"

#define TRY_CATCH_BEGIN try {
#define TRY_CATCH_END } catch (const franka::Exception& e) {handleException(e);}


franka::Torques impedanceControl(const franka::RobotState& state, franka::Duration);
class PandaArm {
    using pandaArray_t = std::array<double, NUM_PANDA_JOINTS>;
    using trMat_t = std::array<double, 16>;

public:
    enum MotionStat {
        None,
        Running,
        Finished
    };
    enum TrackStat {
        Current,
        Next
    };
    enum Technique{
        NoTechnique,
        Strum,
        Pick,
        Tremolo
    };
    struct CalibrationParam{
        double depthTau;
        double depthPose;
        double heightTau;
        double heightPose;
        double elbowPose;
        double compensationAngle = 0.0;
    };

    struct MoveParam{
        int m_index;
        Technique technique;
        Technique next_technique;
        int direction;  // 1 is down, 0 is up
        int next_direction;
        double play_time;
        double prepare_position;
        double end_position;
        double move_step;
        double prepare_time;
        double timbre;
        double depth;
        double last_position = 3.5;   // middle of the strings
    };



public:
    PandaArm();
    PandaException_t init(const std::string& host);
    PandaException_t startRTLoop(double timeout_sec);
    void stopRTLoop();
    Error_t startTorqueAnalyzer();
    //bool motionFinished() { return m_motionStat != Running; }
    void setMotionStat_p(MotionStat stat);
    void setMotionStat_m(MotionStat stat);
    void setMotionStat_t(MotionStat stat);
    void setMotionStat_d(MotionStat stat);
    void setTrackStat(TrackStat stat);
    void setMoveParameter(struct MoveParam param);
    void readMidiVelocity(uint8_t vel_a);
    void desiredMidiVelocity(uint8_t vel_d);
    void readTimbreStatus(double value);
    void readDepthStatus(double value);


    std::atomic<MotionStat> m_motionStat;
    std::atomic<MotionStat> m_motionStat_t;
    std::atomic<MotionStat> m_motionStat_f;
    std::atomic<MotionStat> m_motionStat_d;
    std::atomic<MotionStat> m_motionStat_p;
//    std::atomic<MotionStat> m_motionStat_m;
    std::atomic<TrackStat> m_trackStat;
    //std::atomic<Technique> m_technique;
    struct MoveParam m_param;
    struct CalibrationParam m_cparam;
    bool newTrackFlag;
    double timbre_direction = 1.0;
    double depth_direction = 1.0;
    double motion_destroyer = 1.0;

private:
//    franka::JointVelocities jointVelocityControl(const franka::RobotState& state, franka::Duration period);
//    friend franka::Torques impedanceControl(const franka::RobotState& state, franka::Duration);
//    std::array<double, NUM_PANDA_JOINTS> m_kQInit = {1.52833,-1.58906,-1.54433,-1.90001,-0.0661903,1.85044,-0.815024};

    std::array<double, NUM_PANDA_JOINTS> m_kQInit = {1.46129,-1.62691,-1.54878,-1.95576,-0.082834,1.81582,-0.735052};
    // std::array<double, NUM_PANDA_JOINTS> m_kQInit = {1.46361,-1.5125,-1.6621,-1.87071,0.0449357,1.79659,-0.877551};
    std::array<double, NUM_PANDA_JOINTS> m_kQInitTimbre = {1.19997,-1.61796,-1.47949,-2.20173,0.0134844,1.79301,-0.721294};   // sharp timbre position
    std::array<double, NUM_PANDA_JOINTS> m_kQInitTuned = {1.69292,-1.60188,-1.48382,-1.66542,0.0189535,1.79253,-0.72093};
    std::array<double, NUM_PANDA_JOINTS> m_kQInitDepth = {1.4287,-1.60407,-1.48271,-1.96162,0.0200001,1.79053,-0.718365};   // sharp timbre position
    std::array<double, NUM_PANDA_JOINTS> m_kQFinalDepth = {1.43499,-1.60756,-1.48411,-1.98874,0.0200018,1.8251,-0.719468};
    // std::array<double, NUM_PANDA_JOINTS> m_kQInitTuned_mid = {1.343725,-1.602795,-1.47722,-2.24285,0.02039015,1.790545,-0.71809};
    std::array<double, NUM_PANDA_JOINTS> m_qVel;
    std::array<std::array<double, NUM_PANDA_JOINTS>, NUM_PANDA_JOINTS> Strum_qGoal;
    std::array<std::array<double, NUM_PANDA_JOINTS>, NUM_PANDA_JOINTS> Pick_qGoal;
    std::array<double, NUM_PANDA_JOINTS> tech_q_goal;
    std::array<double, NUM_PANDA_JOINTS> m_GainsK;
    std::array<double, NUM_PANDA_JOINTS> m_GainsD;
    std::array<double, NUM_PANDA_JOINTS> m_omega;
    std::array<double, NUM_PANDA_JOINTS> m_omega_t;
    std::array<double, NUM_PANDA_JOINTS> m_omega_d;
    std::array<double, NUM_PANDA_JOINTS> m_omega_p;
    std::array<double, NUM_PANDA_JOINTS> m_omega_m;
    std::array<double, NUM_PANDA_JOINTS> m_omega_i;
    std::array<double, NUM_PANDA_JOINTS> tau_d_calculated;
    std::array<double, NUM_PANDA_JOINTS> tau_external_read;
    std::array<double, 16> m_initial_pose;
    std::array<double, 2> m_initial_elbow;
    double fifthPolySemicircleProfile(double q_i, double q_f, double t_total, double t);
    double fifthPolyQuartercircleProfile(double q_i, double q_f, double t_total, double t);   /// Jumbo
    double fifthPolyProfile(double q_i, double q_f, double t_total, double t);
    double fifthPolyProfile_pos(double q_i, double q_f, double t_total, double t);
    void timbreCheck();
    void readInitialPose();
    void calibration();
    void errorCompensation();
    void defineStrum_qGoal();
    void definePick_qGoal();
    void torqueAnalyzer();
    double midiVelocityAnalyzer();
    double c_timbre_time = 0.04;   // 0.00314
    double c_depth_time = 0.02;       //       to be adjusted
    double m_iTimbreStatus = 64;
    double m_iDepthStatus = 64;
    double m_init_time = 1.0;
    double m_time = 0.0;
    double m_time_t = 0.0;
    double m_time_d = 0.0;
    double m_time_p = 0.0;
    double m_time_m = 0.0;
    double m_time_i = 0.0;
    double m_iErrorCompensator = 0.0;

    MotorController m_motorcontroller;

    uint8_t m_iVel_a;
    uint8_t m_iVel_d;

    PandaException_t configure();
    static PandaException_t handleException(const franka::Exception& e);

    bool m_bInitialized;
    std::atomic<bool> m_bRunning;
    std::unique_ptr<franka::Robot> m_pRobot= nullptr;
    std::unique_ptr<franka::Model> m_pModel= nullptr;
    std::unique_ptr<std::thread> m_pStateReadingThread;
    //const franka::RobotState& robotstate;

    trMat_t m_currentPose = {};


    std::atomic<double> m_execTime = 0.0;

    ///////////dummy constant values for testing////////

    Technique technique = Strum;
    Technique nextTechnique = Strum;
    ///////////////////////////////////////////////////
};


#endif //GUITARBOT_PANDAARM_H
