
//
// Created by Raghavasimhan Sankaranarayanan on 5/8/21. Reference: https://github.com/frankaemika/libfranka/blob/master/examples/examples_common.h
//

#ifndef GUITARBOT_MOTIONGENERATOR_H
#define GUITARBOT_MOTIONGENERATOR_H

#include <array>

#include <Eigen/Core>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/exception.h>

#include "ErrorDef.h"
#include "Defines.h"

class MotionGenerator {
public:
    MotionGenerator(double speed_factor, std::array<double, NUM_PANDA_JOINTS> q_goal);
    franka::JointPositions operator()(const franka::RobotState& robot_state, franka::Duration period);

private:
    using Vector7d = Eigen::Matrix<double, NUM_PANDA_JOINTS, 1, Eigen::ColMajor>;
    using Vector7i = Eigen::Matrix<int, NUM_PANDA_JOINTS, 1, Eigen::ColMajor>;

    bool calculateDesiredValues(double t, Vector7d* delta_q_d) const;
    void calculateSynchronizedValues();

    static constexpr double kDeltaQMotionFinished = 1e-6;
    const Vector7d q_goal_;

    Vector7d q_start_;
    Vector7d delta_q_;

    Vector7d dq_max_sync_;
    Vector7d t_1_sync_;
    Vector7d t_2_sync_;
    Vector7d t_f_sync_;
    Vector7d q_1_;

    double time_ = 0.0;

    Vector7d dq_max_ = (Vector7d() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5).finished();
    Vector7d ddq_max_start_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();
    Vector7d ddq_max_goal_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();
};


#endif //GUITARBOT_MOTIONGENERATOR_H