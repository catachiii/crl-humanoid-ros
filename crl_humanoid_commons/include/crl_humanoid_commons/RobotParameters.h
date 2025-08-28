//
// Created by donghok on 14.06.21.
//

#ifndef CRL_HUMANOID_COMMONS_ROBOTPARAMETERS_H
#define CRL_HUMANOID_COMMONS_ROBOTPARAMETERS_H

#include <map>
#include <string>
#include <vector>

#include "crl-basic/utils/mathDefs.h"

namespace crl::unitree::commons {

    enum class RobotModelType {
        UNKNOWN = -1, UNITREE_G1 = 0
    };

    struct UnitreeRobotModel {
        RobotModelType modelType = RobotModelType::UNKNOWN;
        std::vector<std::string> jointNames;  // joint names
        std::vector<double> defaultJointConf;  // used mainly for the default standing posture.
        std::vector<double> jointPosMax;
        std::vector<double> jointPosMin;
        std::vector<double> jointVelMax;
        std::vector<double> jointTorqueMax;
        std::vector<double> jointStiffnessDefault;  // used in pos mode and estop
        std::vector<double> jointDampingDefault;  // used in pos mode and estop
    };

    const std::map<std::string, UnitreeRobotModel> robotModels = {
            {// unitree G1
                    "G1",
                    {
                            RobotModelType::UNITREE_G1,
                            {
                                    "left_hip_pitch_joint", "right_hip_pitch_joint",
                                    "waist_yaw_joint",
                                    "left_hip_roll_joint", "right_hip_roll_joint",
                                    "waist_roll_joint",
                                    "left_hip_yaw_joint", "right_hip_yaw_joint",
                                    "waist_pitch_joint",
                                    "left_knee_joint", "right_knee_joint",
                                    "left_shoulder_pitch_joint", "right_shoulder_pitch_joint",
                                    "left_ankle_pitch_joint", "right_ankle_pitch_joint",
                                    "left_shoulder_roll_joint", "right_shoulder_roll_joint",
                                    "left_ankle_roll_joint", "right_ankle_roll_joint",
                                    "left_shoulder_yaw_joint", "right_shoulder_yaw_joint",
                                    "left_elbow_joint", "right_elbow_joint",
                                    "left_wrist_roll_joint", "right_wrist_roll_joint",
                                    "left_wrist_pitch_joint", "right_wrist_pitch_joint",
                                    "left_wrist_yaw_joint", "right_wrist_yaw_joint"
                            },
                            {
                                    -0.1000, -0.1000,
                                    0.0000,
                                    0.0000, 0.0000,
                                    0.0000,
                                    0.0000, 0.0000,
                                    0.0000,
                                    0.3000, 0.3000,
                                    0.3500, 0.3500,
                                    -0.2000, -0.2000,
                                    0.1600, -0.1600,
                                    0.0000, 0.0000,
                                    0.0000, 0.0000,
                                    0.8700, 0.8700,
                                    0.0000, 0.0000,
                                    0.0000, 0.0000,
                                    0.0000, 0.0000
                            },
                            {
                                2.8798,  2.8798,
                                2.618,
                                2.9671,  0.5236,
                                0.52,
                                2.7576,  2.7576,
                                0.52,
                                2.8798,  2.8798,
                                2.6704,  2.6704,
                                0.5236,  0.5236,
                                2.2515,  1.5882,
                                0.2618,  0.2618,
                                2.618,   2.618,
                                2.0944,  2.0944,
                                1.97222, 1.97222,
                                1.61443, 1.61443,
                                1.61443, 1.61443
                            },
                            {
                                -2.5307, -2.5307,
                                -2.618,
                                -0.5236, -2.9671,
                                -0.52,
                                -2.7576, -2.7576,
                                -0.52,
                                -0.087267, -0.087267,
                                -3.0892, -3.0892,
                                -0.87267, -0.87267,
                                -1.5882, -2.2515,
                                -0.2618, -0.2618,
                                -2.618,  -2.618,
                                -1.0472, -1.0472,
                                -1.97222, -1.97222,
                                -1.61443, -1.61443,
                                -1.61443, -1.61443
                            },
                            {
                                15.0, 15.0,
                                10.0,
                                15.0, 15.0,
                                8.0,
                                15.0, 15.0,
                                8.0,
                                15.0, 15.0,
                                20.0, 20.0,
                                10.0, 10.0,
                                20.0, 20.0,
                                10.0, 10.0,
                                20.0, 20.0,
                                25.0, 25.0,
                                30.0, 30.0,
                                25.0, 25.0,
                                25.0, 25.0
                            },
                            {
                                88.0, 88.0,
                                88.0,
                                139.0, 139.0,
                                50.0,
                                88.0, 88.0,
                                50.0,
                                139.0, 139.0,
                                25.0, 25.0,
                                50.0, 50.0,
                                25.0, 25.0,
                                50.0, 50.0,
                                25.0, 25.0,
                                25.0, 25.0,
                                25.0, 25.0,
                                5.0, 5.0,
                                5.0, 5.0
                            },
                            {
                                100., 100.,
                                400.,
                                100., 100.,
                                400.,
                                100., 100.,
                                400.,
                                200., 200.,
                                40.,  40.,
                                20.,  20.,
                                60.,  60.,
                                20.,  20.,
                                20.,  20.,
                                60.,  60.,
                                20.,  20.,
                                20.,  20.,
                                20.,  20.
                            },
                            {
                                2.5, 2.5,
                                5.,
                                2.5, 2.5,
                                5.,
                                2.5, 2.5,
                                5.,
                                5., 5.,
                                2.5, 2.5,
                                0.2, 0.2,
                                1., 1.,
                                0.1, 0.1,
                                0.4, 0.4,
                                1., 1.,
                                0.4, 0.4,
                                0.4, 0.4,
                                0.4, 0.4,
                            }
                    },
            },
    };

}  // namespace crl::unitree::commons

#endif  //CRL_HUMANOID_COMMONS_ROBOTPARAMETERS_H
