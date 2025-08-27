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
        std::vector<double> defaultJointConf;
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
                    },
            },
    };

}  // namespace crl::unitree::commons

#endif  //CRL_HUMANOID_COMMONS_ROBOTPARAMETERS_H
