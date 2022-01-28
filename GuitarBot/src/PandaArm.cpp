
//
// Created by Raghavasimhan Sankaranarayanan on 5/8/21.
//

#include "PandaArm.h"

PandaArm::PandaArm()
        : m_bInitialized(false),
        m_bRunning(false),
        m_motionStat(None)
{

}


PandaException_t PandaArm::init(const std::string& host) {
    TRY_CATCH_BEGIN


        m_pRobot = std::make_unique<franka::Robot>(host);
        m_pModel = std::make_unique<franka::Model>(m_pRobot->loadModel());
        m_pRobot->setCollisionBehavior(
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

        m_pRobot->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
        m_pRobot->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
        m_bInitialized = true;

        LOG_INFO("Panda Arm: Successfully connected.");

//        readInitialPose();
//        LOG_INFO("Panda Arm: record initial pose. ");
        MotionGenerator motionGen(0.1, m_kQInit);
        // m_motionStat = Running;

        m_pRobot->control(motionGen);
        LOG_INFO("Panda Arm: goes to the initial pose.");
        // calibration();
        // timbreCheck();
        //MotionGenerator motionGen1(0.1,  m_kQInitTuned);
        //m_pRobot->control(motionGen1);
        // errorCompensation();
        m_motionStat = Finished;
    auto err = configure();
    if (err != kNoException)
        return err;
    TRY_CATCH_END

    return kNoException;
}



PandaException_t PandaArm::configure() {
    if (!m_pRobot)
        return kUnknownException;

    TRY_CATCH_BEGIN
        // Stiffness
        m_GainsK = {{700.0, 700.0, 1000.0, 1000.0, 250.0, 150.0, 50.0}};
        // Damping
        m_GainsD = {{150.0, 50.0, 55.0, 50.0, 22.0, 25.0, 15.0}};

    TRY_CATCH_END

    return kNoException;
}

PandaException_t PandaArm::handleException(const franka::Exception &e) {
    using namespace franka;
    try {
        throw e;
    } catch (const CommandException& e) {
        std::cout << e.what() << std::endl;
        return kCommandException;
    } catch (const NetworkException& e) {
        std::cout << e.what() << std::endl;
        return kNetworkException;
    } catch (const ControlException& e) {
        std::cout << e.what() << std::endl;
        return kControlException;
    } catch (const InvalidOperationException& e) {
        std::cout << e.what() << std::endl;
        return kInvalidOperationException;
    } catch (const RealtimeException& e) {
        std::cout << e.what() << std::endl;
        return kRealtimeException;
    } catch (const ModelException& e) {
        std::cout << e.what() << std::endl;
        return kModelException;
    } catch (const IncompatibleVersionException& e) {
        std::cout << e.what() << std::endl;
        return kIncompatibleVersionException;
    } catch (const Exception& e) {
        std::cout << e.what() << std::endl;
        return kUnknownException;
    }
}

PandaException_t PandaArm::startRTLoop(double timeout_sec) {

    if (!m_bInitialized) {
        LOG_ERROR("Panda Arm not initialized.");
        return kNotInitializedException;
    }
    //startTorqueAnalyzer();
    m_bRunning = true;
    m_qVel = m_kQInit;


    TRY_CATCH_BEGIN
//        m_pRobot->control([this](const franka::RobotState& state, franka::Duration period) -> franka::Torques {
//            impedanceControl(state, period); }, [this](const franka::RobotState& state, franka::Duration period) -> franka::JointVelocities {
//            jointVelocityControl(state, period); });
        m_pRobot->control(
//        m_pRobot->control(
//                [this, &timeout_sec](const franka::RobotState& state, franka::Duration) -> franka::Torques{
//                              // Read current coriolis terms from model.
//                              std::array<double, NUM_PANDA_JOINTS> coriolis = m_pModel->coriolis(state);
//                              // Compute torque command from joint impedance control law.
//                              // Note: The answer to our Cartesian pose inverse kinematics is always in state.q_d with one
//                              // time step delay.
//
//                              for (size_t i = 0; i < NUM_PANDA_JOINTS; i++) {
//                                  tau_d_calculated[i] =
//                                          m_GainsK[i] * (state.q_d[i] - state.q[i]) - m_GainsD[i] * state.dq[i] + coriolis[i];
////        tau_external_read[i] = state.tau_ext_hat_filtered[i];
//                              }
//
//                              std::array<double, NUM_PANDA_JOINTS> tau_d_rate_limited =
//                                      franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, state.tau_J_d);
//
//                              return tau_d_rate_limited;
//            },

                      [this, &timeout_sec](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointVelocities {
                          double time_ = m_execTime.load();
                          time_ += period.toSec();
                          m_omega.fill(0);
                          m_omega_t.fill(0);
                          m_omega_d.fill(0);
                          m_omega_p.fill(0);
                          m_omega_m.fill(0);
                          m_omega_i.fill(0);
                          if (time_ == 0.0){
                              LOG_INFO("Panda Arm: starting new motion");
                          }



                          if (m_trackStat != Next) {
                              m_execTime = time_;
                              m_time_i = 0.0;
                              m_omega_i.fill(0);
                          }


                          if (m_motionStat_t != Running) {
                              m_execTime = time_;
                              m_time_t = 0.0;
                              m_omega_t.fill(0);
                          }

                          if (m_motionStat_p != Running) {
                              m_execTime = time_;
                              m_time_p = 0.0;
                              m_omega_p.fill(0);
                          }

//                          if (m_motionStat_m != Running) {
//                              m_execTime = time_;
//                              m_time_m = 0.0;
//                          }

                          if (m_motionStat_d != Running) {
                              m_execTime = time_;
                              m_time_d = 0.0;
                              m_omega_d.fill(0);
                          }


                          // track init (move to prepare position)
                          if (m_time_i <= m_init_time) {

                              m_omega_i[1] = -0.25*fifthPolyProfile(kStringSpacing*m_param.last_position, kStringSpacing*m_param.prepare_position,
                                                                    m_init_time,
                                                              m_time_i);
                              m_omega_i[2] = -fifthPolyProfile(kStringSpacing*m_param.last_position, kStringSpacing*m_param.prepare_position,
                                                               m_init_time,
                                                                   m_time_i);
                              m_omega_i[3] = -fifthPolySemicircleProfile(0.03, 0.0, m_init_time,
                                                                         m_time_i);
                              m_omega_i[4] = -fifthPolyProfile(kStringSpacing*m_param.last_position, kStringSpacing*m_param.prepare_position,
                                                               m_init_time,
                                                                   m_time_i);

//
//                              m_omega_i[2] = -fifthPolyProfile(kStringSpacing*m_param.last_position, kStringSpacing*m_param.prepare_position,
//                                                               c_init_time,
//                                                               m_time_i);
//                              m_omega_i[4] = -fifthPolyProfile(kStringSpacing*m_param.last_position, kStringSpacing*m_param.prepare_position,
//                                                               c_init_time,
//                                                               m_time_i);
//                              //////////////////////////0.013 ---> 0.03///////////////////
//                              m_omega_i[3] = -fifthPolySemicircleProfile(0.03, 0.0, c_init_time,
//                                                                         m_time_i);
                          }

                          // depth change
                          if (m_time_d <= c_depth_time) {

                              if ( m_param.depth > m_iDepthStatus){
                                  depth_direction =   1.0;
                              }

                              else if ( m_param.depth == m_iDepthStatus){
                                  depth_direction =   0.0;
                              }

                              else {
                                  depth_direction =   -1.0;
                              }

                              m_omega_d[0] += 2*depth_direction *fifthPolyProfile(0.0,(m_kQFinalDepth[0] -m_kQInitDepth[0])/127.0, c_depth_time, m_time_d);
                              m_omega_d[1] += 2*depth_direction *fifthPolyProfile(0.0,(m_kQFinalDepth[1] -m_kQInitDepth[1])/127.0, c_depth_time, m_time_d);
                              m_omega_d[2] += 2*depth_direction *fifthPolyProfile(0.0,(m_kQFinalDepth[2] -m_kQInitDepth[2])/127.0, c_depth_time, m_time_d);
                              m_omega_d[3] += 2*depth_direction *fifthPolyProfile(0.0,(m_kQFinalDepth[3] -m_kQInitDepth[3])/127.0, c_depth_time, m_time_d);
                              m_omega_d[4] += 2*depth_direction *fifthPolyProfile(0.0,(m_kQFinalDepth[4] -m_kQInitDepth[4])/127.0, c_depth_time, m_time_d);
                              m_omega_d[5] += 2*depth_direction *fifthPolyProfile(0.0,(m_kQFinalDepth[5] -m_kQInitDepth[5])/127.0, c_depth_time, m_time_d);
                              m_omega_d[6] += 2*depth_direction *fifthPolyProfile(0.0,(m_kQFinalDepth[6] -m_kQInitDepth[6])/127.0, c_depth_time, m_time_d);

                          }

                          // timber change
                          if (m_time_t <= c_timbre_time) {

                              if ( m_param.timbre > m_iTimbreStatus){
                                  // LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!");
                                  timbre_direction =   1.0;
                              }

                              else if ( m_param.timbre == m_iTimbreStatus){
                                  timbre_direction =   0.0;
                              }

                              else {
                                  timbre_direction =   -1.0;
                              }

                              m_omega_t[0] -= timbre_direction *fifthPolyProfile(0.0,
                                                                  (m_kQInitTimbre[0] -
                                                                   m_kQInitTuned[0])/127.0,
                                                               c_timbre_time,
                                                               m_time_t);
                              m_omega_t[1] -= timbre_direction *fifthPolyProfile(0.0,
                                                                  (m_kQInitTimbre[1] -
                                                                   m_kQInitTuned[1])/127.0,
                                                               c_timbre_time,
                                                               m_time_t);

                              m_omega_t[2] -= timbre_direction *fifthPolyProfile(0.0,
                                                                  (m_kQInitTimbre[2] -
                                                                   m_kQInitTuned[2])/127.0,
                                                               c_timbre_time,
                                                               m_time_t);
                              m_omega_t[3] -= timbre_direction *fifthPolyProfile(0.0,
                                                                    (m_kQInitTimbre[3] -
                                                                     m_kQInitTuned[3])/127.0,
                                                               c_timbre_time,
                                                               m_time_t); // + fifthPolySemicircleProfile( 1.0, 0.0,c_timber_time, m_time);
                              m_omega_t[4] -= timbre_direction *fifthPolyProfile(0.0,
                                                                  (m_kQInitTimbre[4] -
                                                                   m_kQInitTuned[4])/127.0,
                                                               c_timbre_time,
                                                               m_time_t);
                              m_omega_t[5] -= timbre_direction *fifthPolyProfile(0.0,
                                                                  (m_kQInitTimbre[5] -
                                                                   m_kQInitTuned[5])/127.0,
                                                               c_timbre_time,
                                                               m_time_t);
                              m_omega_t[6] -= timbre_direction * fifthPolyProfile(0.0,
                                                                  (m_kQInitTimbre[6] -
                                                                   m_kQInitTuned[6])/127.0,
                                                               c_timbre_time,
                                                               m_time_t);
                          }


                          // Play: Strum or Pick or Tremolo
                          switch (m_param.technique) {


                              case Strum:
                                  if (m_time_p <= m_param.play_time){
                                      m_omega_p[1] += 0.25*fifthPolyProfile(kStringSpacing * m_param.prepare_position, kStringSpacing * m_param.end_position,
                                                                      m_param.play_time, m_time_p);
                                      m_omega_p[2] -= fifthPolyProfile(kStringSpacing * m_param.prepare_position, kStringSpacing * m_param.end_position,
                                                                           m_param.play_time, m_time_p);
//                                      m_omega_p[3] -= 0.08*fifthPolyProfile(kStringSpacing * m_param.prepare_position, kStringSpacing * m_param.end_position,
//                                                                           m_param.play_time, m_time_p);
                                      m_omega_p[4] -= fifthPolyProfile(kStringSpacing * m_param.prepare_position, kStringSpacing * m_param.end_position,
                                                                           m_param.play_time, m_time_p);

                                  }
                                  else if (m_time_p > m_param.play_time) {



                                      m_omega_p[1] += 0.25*fifthPolyProfile(0.0, kStringSpacing * m_param.move_step,
                                                                      m_param.prepare_time,
                                                                      m_time_p - m_param.play_time);
                                      m_omega_p[2] -= fifthPolyProfile(0.0, kStringSpacing * m_param.move_step,
                                                                           m_param.prepare_time,
                                                                           m_time_p - m_param.play_time);
                                      m_omega_p[3] -= fifthPolySemicircleProfile(0.01, 0.0, m_param.prepare_time,
                                                                                 m_time_p - m_param.play_time);
                                      m_omega_p[4] -= fifthPolyProfile(0.0, kStringSpacing * m_param.move_step,
                                                                           m_param.prepare_time,
                                                                           m_time_p - m_param.play_time);
                                  }

                                      break;

                              case Pick:
                         if (m_time_p <= m_param.play_time){

//                             m_omega_p[1] = -0.25*fifthPolyProfile(kStringSpacing * m_param.prepare_position, kStringSpacing * m_param.end_position,
//                                                              m_param.play_time, m_time_p);
                                      m_omega_p[2] = -fifthPolyProfile(kStringSpacing * m_param.prepare_position, kStringSpacing * m_param.end_position,
                                                                           m_param.play_time, m_time_p);
                                      m_omega_p[3] = 0.5*fifthPolySemicircleProfile( 0.02, 0.0, m_param.play_time, m_time_p );  // move into string circle
                                      m_omega_p[4] = -3*fifthPolyProfile(kStringSpacing * m_param.prepare_position, kStringSpacing * m_param.end_position,
                                                                           m_param.play_time, m_time_p);
                                  }
                                  else if (m_time_p > m_param.play_time) {
                             if (m_param.technique == m_param.next_technique) {

                                 // If same direction, reocover to neutral position
                                 m_omega_p[4] -= 2 * fifthPolyProfile(kStringSpacing * m_param.end_position,
                                                                      kStringSpacing * m_param.prepare_position,
                                                                      m_param.prepare_time,
                                                                      m_time_p - m_param.play_time);

                                 if (m_param.next_direction != m_param.direction) {
                                     m_omega_p[4] += 2 * fifthPolyProfile(kStringSpacing * m_param.end_position,
                                                                          kStringSpacing * m_param.prepare_position,
                                                                          m_param.prepare_time,
                                                                          m_time_p - m_param.play_time);
                                 }

                                 //move to position

                                 m_omega_p[1] -= 0.25 * fifthPolyProfile(0.0, kStringSpacing * m_param.move_step,
                                                                         m_param.prepare_time,
                                                                         m_time_p - m_param.play_time);
                                 m_omega_p[2] -= fifthPolyProfile(0.0, kStringSpacing * m_param.move_step,
                                                                  m_param.prepare_time,
                                                                  m_time_p - m_param.play_time);
                                 m_omega_p[4] -= fifthPolyProfile(0.0, kStringSpacing * m_param.move_step,
                                                                  m_param.prepare_time,
                                                                  m_time_p - m_param.play_time);


                             }

                             // SWITCHING FROM PICK TO STRUM
                             else {
                                 LOG_INFO("SWITCHING FROM PICK TO STRUM!!!!!!!!!!!!!!");
                                 // If same direction, reocover to neutral position
                                 m_omega_p[4] -= 2 * fifthPolyProfile(kStringSpacing * m_param.end_position,
                                                                      kStringSpacing * m_param.prepare_position,
                                                                      m_param.prepare_time,
                                                                      m_time_p - m_param.play_time);



                                 //move to position

                                 m_omega_p[1] -= 0.25 * fifthPolyProfile(0.0, kStringSpacing * m_param.move_step,
                                                                         m_param.prepare_time,
                                                                         m_time_p - m_param.play_time);

                                 m_omega_p[1] -= 0.25 * fifthPolyProfile(0.0, kStringSpacing * m_param.move_step,
                                                                         m_param.prepare_time,
                                                                         m_time_p - m_param.play_time);


                                 m_omega_p[2] -= fifthPolyProfile(0.0, kStringSpacing * m_param.move_step,
                                                                  m_param.prepare_time,
                                                                  m_time_p - m_param.play_time);
                                 m_omega_p[4] -= fifthPolyProfile(0.0, kStringSpacing * m_param.move_step,
                                                                  m_param.prepare_time,
                                                                  m_time_p - m_param.play_time);



                             }
                         }
                          }



                          // update clock
                          m_time_p += period.toSec();
                          m_time_m += period.toSec();
                          m_time_t += period.toSec();
                          m_time_d += period.toSec();
                          m_time_i += period.toSec();
//                          m_execTime = time_;

                          // closing track init loop
                          if (m_time_i > m_init_time){
                              m_time_i = 0.0;
                              m_trackStat = Current;
                              LOG_INFO("TRACK INITIALIZED");
                              m_param.last_position = m_param.prepare_position;
                              LOG_INFO(" m_param.last_position");
                              LOG_INFO( m_param.last_position);
                              LOG_INFO("m_param.prepare_position");
                              LOG_INFO(m_param.prepare_position);
                          }

                          // closing play loop
                          if (m_time_p > m_param.play_time + m_param.prepare_time){
                              m_param.last_position = m_param.end_position + m_param.move_step;
                              std::cout << "Current Position: " << m_param.last_position << std::endl;
                              std::cout << m_param.play_time + m_param.prepare_time << std::endl;
                              m_time_p = 0.0;
                              m_motionStat_p = Finished;
                              LOG_INFO("PLAY MOTION FINISHED");
                          }


                          // closing depth loop
                          if (m_time_d > c_depth_time){
                              m_time_d = 0.0;

                              LOG_INFO("TARGET DEPTH" ,  "CURRENT");
                              LOG_INFO(m_param.depth);
                              LOG_INFO(m_iDepthStatus);
                              if (m_param.depth == m_iDepthStatus) {
                                  m_motionStat_d = Finished;
                                  LOG_INFO("DEPTH ADJUSTED");
                                  depth_direction = 0.0;
                              }
                              else {
                                  m_iDepthStatus = m_iDepthStatus + depth_direction;
                                  LOG_INFO("depth_direction");
                              }
                          }


                          // closing timber loop
                          if (m_time_t > c_timbre_time){
                              m_time_t = 0.0;

                              LOG_INFO("TARGET TIMBRE" ,  "CURRENT");
                              LOG_INFO(m_param.timbre);
                              LOG_INFO(m_iTimbreStatus);
                              if (m_param.timbre == m_iTimbreStatus) {
                                  m_motionStat_t = Finished;
                                  LOG_INFO("TIMBRE ADJUSTED");
                                  timbre_direction = 0.0;
                              }
                              else {
                                  m_iTimbreStatus = m_iTimbreStatus + timbre_direction;
                                  LOG_INFO("timbre_direction");
                              }
                          }



                          // sum up all the omegas from different operations (depth, timber, and play)
                          for (int i =0; i < NUM_PANDA_JOINTS; i++){
                              m_omega[i] = m_omega_p[i] + m_omega_t[i] + m_omega_d[i]+ m_omega_i[i];
                          }

                          franka::JointVelocities velocities = m_omega;
                          return velocities;
                      });
    TRY_CATCH_END

    return kNoException;
}

//
//franka::Torques impedanceControl(const franka::RobotState& state, franka::Duration /*period*/) {
//    // Read current coriolis terms from model.
//    std::array<double, NUM_PANDA_JOINTS> coriolis = m_pModel->coriolis(state);
//    // Compute torque command from joint impedance control law.
//    // Note: The answer to our Cartesian pose inverse kinematics is always in state.q_d with one
//    // time step delay.
//
//    for (size_t i = 0; i < NUM_PANDA_JOINTS; i++) {
//        tau_d_calculated[i] =
//                m_GainsK[i] * (state.q_d[i] - state.q[i]) - m_GainsD[i] * state.dq[i] + coriolis[i];
////        tau_external_read[i] = state.tau_ext_hat_filtered[i];
//    }
//
//    std::array<double, NUM_PANDA_JOINTS> tau_d_rate_limited =
//            franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, state.tau_J_d);
//
//    return tau_d_rate_limited;
//}

//franka::JointVelocities PandaArm::jointVelocityControl(const franka::RobotState& state, franka::Duration period) {
////    double time_ = m_execTime.load();
////    m_omega.fill(0);
//    time_ += period.toSec();
//
//    franka::JointVelocities velocities = {{0.0,0.0,0.0,0.0,0.0,0.0,0.0}};
//    if (time_ >= 300) {
//        std::cout << std::endl << "Motion finished." << std::endl;
//        return franka::MotionFinished(velocities);
//    }
//    return velocities;
//}


double PandaArm::fifthPolyProfile_pos(double q_i, double q_f, double t_total, double t) {
    double q_dot_i = 0;
    double q_dot_f = 0;
    double q_dotdot_i = 0;
    double q_dotdot_f = 0;
    double a0 = q_i;
    double a1 = q_dot_i;
    double a2 = 0.5* q_dotdot_i;

    double a3 = 1.0/(2.0*pow(t_total,3.0))*(20.0*(q_f-q_i)-(8.0*q_dot_f+12.0*q_dot_i)*t_total-(3.0*q_dotdot_f-q_dotdot_i)*pow(t_total,2.0));
    double a4 = 1.0/(2.0*pow(t_total,4.0))*(30.0*(q_i-q_f)+(14.0*q_dot_f+16.0*q_dot_i)*t_total+(3.0*q_dotdot_f-2.0*q_dotdot_i)*pow(t_total,2.0));
    double a5 = 1.0/(2.0*pow(t_total,5.0))*(12.0*(q_f-q_i)-(6.0*q_dot_f+6.0*q_dot_i)*t_total-(q_dotdot_f-q_dotdot_i)*pow(t_total,2.0));

    return a0+a1*t+a2*pow(t,2.0)+a3*pow(t,3.0)+a4*pow(t,4.0)+a5*pow(t,5.0);

}

double PandaArm::fifthPolyProfile(double q_i, double q_f, double t_total, double t) {
    double q_dot_i = 0;
    double q_dot_f = 0;
    double q_dotdot_i = 0;
    double q_dotdot_f = 0;
    double a1 = q_dot_i;
    double a2 = 0.5* q_dotdot_i;

    double a3 = 1.0/(2.0*pow(t_total,3.0))*(20.0*(q_f-q_i)-(8.0*q_dot_f+12.0*q_dot_i)*t_total-(3.0*q_dotdot_f-q_dotdot_i)*pow(t_total,2.0));
    double a4 = 1.0/(2.0*pow(t_total,4.0))*(30.0*(q_i-q_f)+(14.0*q_dot_f+16.0*q_dot_i)*t_total+(3.0*q_dotdot_f-2.0*q_dotdot_i)*pow(t_total,2.0));
    double a5 = 1.0/(2.0*pow(t_total,5.0))*(12.0*(q_f-q_i)-(6.0*q_dot_f+6.0*q_dot_i)*t_total-(q_dotdot_f-q_dotdot_i)*pow(t_total,2.0));
    if (t>t_total){
        return 0.0;
    }
    return a1 +2.0*a2*t +3.0*a3*pow(t,2.0) + 4.0*a4*pow(t,3.0) + 5.0*a5*pow(t,4.0);

}

double PandaArm::fifthPolySemicircleProfile(double q_i, double q_f, double t_total, double t){
    double q_dot_i = 0;
    double q_dot_f = 0;
    double q_dotdot_i = 0;
    double q_dotdot_f = 0;
    double a1 = q_dot_i;
    double a2 = 0.5* q_dotdot_i;
    double half_t_total = t_total / 2.0;
    double a3 = 1.0/(2.0*pow(half_t_total,3.0))*(20.0*(q_f-q_i)-(8.0*q_dot_f+12.0*q_dot_i)*half_t_total-(3.0*q_dotdot_f-q_dotdot_i)*pow(half_t_total,2.0));
    double a4 = 1.0/(2.0*pow(half_t_total,4.0))*(30.0*(q_i-q_f)+(14.0*q_dot_f+16.0*q_dot_i)*half_t_total+(3.0*q_dotdot_f-2.0*q_dotdot_i)*pow(half_t_total,2.0));
    double a5 = 1.0/(2.0*pow(half_t_total,5.0))*(12.0*(q_f-q_i)-(6.0*q_dot_f+6.0*q_dot_i)*half_t_total-(q_dotdot_f-q_dotdot_i)*pow(half_t_total,2.0));
    double trajVel = a1 +2.0*a2*t +3.0*a3*pow(t,2.0) + 4.0*a4*pow(t,3.0) + 5.0*a5*pow(t,4.0);

    if (t > half_t_total && t <= t_total) {
        a3 = 1.0/(2.0*pow(half_t_total,3.0))*(20.0*(q_i-q_f)-(8.0*q_dot_i+12.0*q_dot_f)*half_t_total-(3.0*q_dotdot_i-q_dotdot_f)*pow(half_t_total,2.0));
        a4 = 1.0/(2.0*pow(half_t_total,4.0))*(30.0*(q_f-q_i)+(14.0*q_dot_i+16.0*q_dot_f)*half_t_total+(3.0*q_dotdot_i-2.0*q_dotdot_f)*pow(half_t_total,2.0));
        a5 = 1.0/(2.0*pow(half_t_total,5.0))*(12.0*(q_i-q_f)-(6.0*q_dot_i+6.0*q_dot_f)*half_t_total-(q_dotdot_i-q_dotdot_f)*pow(half_t_total,2.0));
        trajVel = a1 +2.0*a2*(t-half_t_total) +3.0*a3*pow((t-half_t_total),2.0) + 4.0*a4*pow((t-half_t_total),3.0) + 5.0*a5*pow((t-half_t_total),4.0);
    }
    else if (t>t_total){
        trajVel=0.0;
    }
    return trajVel;

}
double PandaArm::fifthPolyQuartercircleProfile(double q_i, double q_f, double t_total, double t){
    double q_dot_i = 0;
    double q_dot_f = 0;
    double q_dotdot_i = 0;
    double q_dotdot_f = 0;
    double a1 = q_dot_i;
    double a2 = 0.5* q_dotdot_i;
    double quarter_t_total = 3* t_total / 4.0;
    double a3 = 1.0/(2.0*pow(quarter_t_total,3.0))*(20.0*(q_f-q_i)-(8.0*q_dot_f+12.0*q_dot_i)*quarter_t_total-(3.0*q_dotdot_f-q_dotdot_i)*pow(quarter_t_total,2.0));
    double a4 = 1.0/(2.0*pow(quarter_t_total,4.0))*(30.0*(q_i-q_f)+(14.0*q_dot_f+16.0*q_dot_i)*quarter_t_total+(3.0*q_dotdot_f-2.0*q_dotdot_i)*pow(quarter_t_total,2.0));
    double a5 = 1.0/(2.0*pow(quarter_t_total,5.0))*(12.0*(q_f-q_i)-(6.0*q_dot_f+6.0*q_dot_i)*quarter_t_total-(q_dotdot_f-q_dotdot_i)*pow(quarter_t_total,2.0));
    double trajVel = a1 +2.0*a2*t +3.0*a3*pow(t,2.0) + 4.0*a4*pow(t,3.0) + 5.0*a5*pow(t,4.0);

    if (t > quarter_t_total && t < t_total) {
        a3 = 1.0/(2.0*pow(quarter_t_total,3.0))*(20.0*(q_i-q_f)-(8.0*q_dot_i+12.0*q_dot_f)*quarter_t_total-(3.0*q_dotdot_i-q_dotdot_f)*pow(quarter_t_total,2.0));
        a4 = 1.0/(2.0*pow(quarter_t_total,4.0))*(30.0*(q_f-q_i)+(14.0*q_dot_i+16.0*q_dot_f)*quarter_t_total+(3.0*q_dotdot_i-2.0*q_dotdot_f)*pow(quarter_t_total,2.0));
        a5 = 1.0/(2.0*pow(quarter_t_total,5.0))*(12.0*(q_i-q_f)-(6.0*q_dot_i+6.0*q_dot_f)*quarter_t_total-(q_dotdot_i-q_dotdot_f)*pow(quarter_t_total,2.0));
        trajVel = a1 +2.0*a2*(t-quarter_t_total) +3.0*a3*pow((t-quarter_t_total),2.0) + 4.0*a4*pow((t-quarter_t_total),3.0) + 5.0*a5*pow((t-quarter_t_total),4.0);
    }
    else if (t>=t_total){
        trajVel=0.0;
    }
    return trajVel;
}

void PandaArm::defineStrum_qGoal(){
    Strum_qGoal[0] = m_kQInit;
    for (int i=1; i<NUM_PANDA_JOINTS; i++){
        Strum_qGoal[i] = {{Strum_qGoal[0][0],Strum_qGoal[0][1],Strum_qGoal[0][2]-kStringSpacing * (i+0.5),Strum_qGoal[0][3],Strum_qGoal[0][4],Strum_qGoal[0][5]- 1.1 * (kStringSpacing * i),Strum_qGoal[0][6]}};
    }
}

//void PandaArm::definePick_qGoal(){
//    Pick_q_goal[0] = {{-1.64886,1.61442,1.32616,-1.67699,-0.00963989,1.64406,0.70846}};
//    for (int i=1; i<NUM_PANDA_JOINTS; i++){
//        Pick_q_goal[i] = {{Pick_q_goal[0][0],Pick_q_goal[0][1],Pick_q_goal[0][2]-kStringSpacing * i,Strum_q_goal[0][3],Pick_q_goal[0][4],Pick_q_goal[0][5],Strum_q_goal[0][6]}};
//    }
//    return 0;
//}



void PandaArm::stopRTLoop() {
    m_bRunning = false;
    if (m_pStateReadingThread)
        if (m_pStateReadingThread->joinable()) {
            m_pStateReadingThread->join();
        }
}
void PandaArm::setMotionStat_p(MotionStat stat){
    m_motionStat_p = stat;
}

//void PandaArm::setMotionStat_m(MotionStat stat){
//    m_motionStat_m = stat;
//}

void PandaArm::setMotionStat_t(MotionStat stat){
    m_motionStat_t = stat;
}

void PandaArm::setMotionStat_d(MotionStat stat){
    m_motionStat_d = stat;
}

void PandaArm::setTrackStat(TrackStat stat){
    m_trackStat = stat;
}

void PandaArm::setMoveParameter(struct MoveParam param){
    param.last_position = m_param.last_position;
    // param.this_position = m_param.prepare_position;
//    LOG_INFO("param.last_position");
//    LOG_INFO(param.last_position);
    m_param.play_time = param.play_time;
    m_param.prepare_position = param.prepare_position;
    m_param.prepare_time = param.prepare_time;
    m_param.end_position = param.end_position;
    m_param.move_step = param.move_step;
    m_param.direction = param.direction;
    m_param.technique = param.technique;
    m_param.m_index = param.m_index;
    m_param.next_direction = param.next_direction;
    m_param.next_technique = param.next_technique;
}

Error_t PandaArm::startTorqueAnalyzer(){
    if (!m_bInitialized) {
        return kNotInitializedError;
    }
    m_pStateReadingThread = std::make_unique<std::thread>(&PandaArm::torqueAnalyzer, this);
    return kNoError;
}

void PandaArm::torqueAnalyzer(){

////////////////////////////////////TO BE IMPLEMENTED/////////////////
    LOG_INFO("Panda Arm: Torque Analyzer activated");
//////////////////////////////////////////////////////////////////////
}


void PandaArm::readTimbreStatus(double value) {
    m_param.timbre = value;
}

void PandaArm::readDepthStatus(double value) {
    LOG_INFO("DEPTH STATUS");
    LOG_INFO(value);
    m_param.depth = value;
}


void PandaArm::readMidiVelocity(uint8_t vel_a) {
    m_iVel_a = vel_a;
}

void PandaArm::desiredMidiVelocity(uint8_t vel_d) {
    m_iVel_d = vel_d;
}

double PandaArm::midiVelocityAnalyzer() {
    LOG_INFO(m_iVel_d);
    LOG_INFO(m_iVel_a);
    if(m_iVel_a==0){
       return 10.0;
    }
    int error = m_iVel_d - m_iVel_a;
    //LOG_INFO(error);
    if (abs(error)<Error_threshold){
        return 0.0;
    }
    else{
        return 0.2*error;
    }


}

void PandaArm::timbreCheck() {

    m_time = 0.0;
    m_pRobot->control([this](const franka::RobotState& state, franka::Duration period) -> franka::CartesianPose{
        m_time += period.toSec();
        double runtime = 1.5;
        if (m_time == 0.0) {
            m_initial_pose = state.O_T_EE_c;
        }

        double delta_x = fifthPolyProfile_pos(0,-0.05, runtime,m_time);
        double delta_y = fifthPolyProfile_pos(0,-0.002, runtime,m_time);
        double delta_z = fifthPolyProfile_pos(0,-0.003, runtime,m_time);
        std::array<double, 16> new_pose = m_initial_pose;
        new_pose[12] += delta_x;
        new_pose[13] += delta_y;
        new_pose[14] += delta_z;
        if (m_time > 5.0 || m_time > runtime) {
            return franka::MotionFinished(new_pose);
        }
        return new_pose;
    });
    LOG_INFO("Panda Arm: move to pose for sharp timbre");
    m_pRobot->read([this](const franka::RobotState& robot_state) {
        for (int i =0; i < NUM_PANDA_JOINTS; i++){
            m_kQInitTimbre[i] = robot_state.q[i];
            LOG_INFO("m_kQInitTimbre[i]");
            LOG_INFO(m_kQInitTimbre[i]);
        }
        return 0;
    });
    LOG_INFO("Panda Arm: record pose for sharp timbre");
}

void PandaArm::calibration(){
    // MotorController::changeGrip(127);
    m_motionStat = Running;
    try{
        m_time = 0.0;

        m_pRobot->control([this](const franka::RobotState& state, franka::Duration period) -> franka::CartesianPose{
            m_cparam.depthTau = state.tau_ext_hat_filtered[3];
            m_time += period.toSec();
            double runtime = 20.0;
            if (m_time == 0.0) {
                m_initial_pose = state.O_T_EE_c;
            }
//            LOG_INFO("slope");
//            LOG_INFO((m_cparam.tauNew-m_cparam.tauOld)/0.001);
//            LOG_INFO("tau");
//            LOG_INFO(state.tau_ext_hat_filtered[3]);


            double delta_y = fifthPolyProfile_pos(0,0.1, runtime,m_time);
            std::array<double, 16> new_pose = m_initial_pose;
            new_pose[13] += delta_y;
            m_cparam.depthPose = state.O_T_EE[13];
            if (m_time > runtime || m_cparam.depthTau < -2.0) {
                m_cparam.depthPose = state.O_T_EE[13];
                return franka::MotionFinished(new_pose);
            }
            return new_pose;

        });
    }catch (const franka::ControlException& e) {
        LOG_INFO("Panda Arm: finishing up depth calibration");
        LOG_INFO(m_cparam.depthPose);
        m_pRobot->automaticErrorRecovery();
    }
    LOG_INFO("Panda Arm: depth calibration finished");


    try{
        m_time = 0.0;
        m_pRobot->control([this](const franka::RobotState& state, franka::Duration period) -> franka::CartesianPose{
            m_cparam.heightTau = state.tau_ext_hat_filtered[4];
            m_time += period.toSec();
            double runtime1 = 1.0;
            double runtime2 = 20.0;
            if (m_time == 0.0) {
                m_initial_pose = state.O_T_EE_c;
            }
            LOG_INFO("tau");
            LOG_INFO(state.tau_ext_hat_filtered[4]);

            double delta_y = fifthPolyProfile_pos(0,-0.002, runtime1,m_time);
            if (m_time > runtime1){
                delta_y = -0.002;
            }
            double delta_z = fifthPolyProfile_pos(0,0.1, runtime2,m_time);
            std::array<double, 16> new_pose = m_initial_pose;
            new_pose[14] += delta_z;
            new_pose[13] += delta_y;
            m_cparam.heightPose = state.O_T_EE[14];
            if (m_time > runtime2 || m_cparam.heightTau > 1.5) {
                m_cparam.heightPose = state.O_T_EE[14];
                return franka::MotionFinished(new_pose);
            }
            return new_pose;

        });
    }catch (const franka::ControlException& e) {
        LOG_INFO("Panda Arm: finishing up height calibration");
        LOG_INFO(m_cparam.heightPose);
        m_pRobot->automaticErrorRecovery();
    }
    LOG_INFO("Panda Arm: height calibration finished");

    m_time = 0.0;
    m_pRobot->control([this](const franka::RobotState& state, franka::Duration period) -> franka::CartesianPose{
        m_time += period.toSec();
        double runtime1 = 5.0;
        double runtime2 = 5.0;
        double runtime3 = 5.0;
        double delta_z = 0.0;
        double delta_y = 0.0;
        double delta_x = 0.0;
        if (m_time == 0.0) {
            m_initial_pose = state.O_T_EE_c;
        }
        std::array<double, 16> new_pose = m_initial_pose;

        if (m_time <=runtime1){
            delta_y = fifthPolyProfile_pos(0.0,-0.05, runtime1,m_time);
            delta_z = 0;
            delta_x = 0;
        }

        else if (m_time > runtime1){
            ////////////-0.0085---> -0.005 0.063---->0.07//////////
            ////////////delta_y q_f: the initial depth going inside the string/////
            //////////delta_z qf: the initial height going above the string/////
            delta_y = fifthPolyProfile_pos(-0.05,-0.006, runtime2,m_time-runtime1);
            delta_z = fifthPolyProfile_pos(0.0,0.063, runtime2,m_time-runtime1);
            // delta_x = fifthPolyProfile_pos(0.0,0.1, runtime2,m_time-runtime1);
        }
        new_pose[12] += delta_x;
        new_pose[13] += delta_y;
        new_pose[14] += delta_z;
        if (m_time > runtime1+runtime2) {
            return franka::MotionFinished(new_pose);
        }
        return new_pose;
    });

    m_time = 0.0;
    m_pRobot->control(
            [this](const franka::RobotState& state,
                   franka::Duration period) -> franka::CartesianPose {
                m_time += period.toSec();

                if (m_time == 0.0) {
                    m_initial_pose = state.O_T_EE_c;
                    m_initial_elbow = state.elbow_c;
                    m_cparam.elbowPose = state.elbow[0];
                    LOG_INFO(state.elbow[0]);
                }

//                double angle = M_PI / 10.0 * (1.0 - std::cos(M_PI / 5.0 * time));
                double angle = fifthPolyProfile_pos(0,-M_PI_2-m_cparam.elbowPose, 5.0, m_time);
                auto elbow = m_initial_elbow;
                elbow[0] += angle;

                if (m_time >= 5) {
                    return franka::MotionFinished({m_initial_pose, elbow});
                }

                return {m_initial_pose, elbow};
            });
}


void PandaArm::errorCompensation() {
    try{
        m_time = 0.0;

        m_pRobot->control([this](const franka::RobotState& state, franka::Duration period) -> franka::CartesianPose{
            m_cparam.depthTau = state.tau_ext_hat_filtered[3];
            m_time += period.toSec();
            double runtime = 2.0;
            if (m_time == 0.0) {
                m_initial_pose = state.O_T_EE_c;
            }
//            LOG_INFO("slope");
//            LOG_INFO((m_cparam.tauNew-m_cparam.tauOld)/0.001);
//            LOG_INFO("tau");
//            LOG_INFO("tau");
//            LOG_INFO(state.tau_ext_hat_filtered[3]);

            double delta_y = fifthPolyProfile_pos(0,0.0005, runtime,m_time);
            std::array<double, 16> new_pose = m_initial_pose;
            new_pose[13] += delta_y;
            m_cparam.depthPose = state.O_T_EE[13];
            if (m_time > runtime) {
                m_cparam.compensationAngle = state.q[3]-m_kQInitTuned[3];
                LOG_INFO(m_cparam.compensationAngle);
                return franka::MotionFinished(new_pose);
            }
            return new_pose;

        });
    }catch (const franka::ControlException& e) {
        LOG_INFO("Panda Arm: finishing up error compensator calibration");
        m_pRobot->automaticErrorRecovery();
    }
    LOG_INFO("Panda Arm: error compensator calibration finished");
}
