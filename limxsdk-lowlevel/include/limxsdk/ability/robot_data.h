/**
 * @file robot_data.h
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#ifndef ROBOT_DATA_H
#define ROBOT_DATA_H
#include <iostream>
#include <thread>
#include <mutex>
#include <memory>
#include <string>
#include <vector>
#include "limxsdk/macros.h"
#include "limxsdk/datatypes.h"
#include "limxsdk/apibase.h"
#include "limxsdk/pointfoot.h"
#include "limxsdk/humanoid.h"
#include "limxsdk/wheellegged.h"

namespace limxsdk {
namespace ability {

class LIMX_SDK_API RobotData {
public:
  RobotData(const std::string& robot_ip, const std::string& robot_type) {
    if (robot_type == "PointFoot") {
        robot = limxsdk::PointFoot::getInstance();
    } else if (robot_type == "Humanoid") {
        robot = limxsdk::Humanoid::getInstance();
    } else if (robot_type == "Wheellegged") {
        robot = limxsdk::Wheellegged::getInstance();
    } else {
        std::cerr<< "Unsupported robot type: " << robot_type << std::endl;
        abort();
    }

    if (!robot->init(robot_ip)) {
        std::cerr<< "Failed to connect to robot at: " << robot_ip << std::endl;
        abort();
    }

    robot->subscribeImuData([this](const limxsdk::ImuDataConstPtr& msg){
      imuDataMutex.lock();
      imuData = *msg;
      imuDataMutex.unlock();
    });

    robot->subscribeRobotState([this](const limxsdk::RobotStateConstPtr& msg){
      robotStateMutex.lock();
      robotState = *msg;
      robotStateMutex.unlock();
    });
  }

  limxsdk::ImuData get_imu_data()  {
      std::lock_guard<std::mutex> lock(imuDataMutex);
      return imuData;
  }

  limxsdk::RobotState get_robot_state() { 
    std::lock_guard<std::mutex> lock(robotStateMutex);
    return robotState;
  }

  limxsdk::ApiBase* get_robot_instance() const {
    return robot;
  }

private:
  limxsdk::ApiBase* robot;  // Robot instance
  limxsdk::RobotState robotState;     // Shared robot state
  std::mutex robotStateMutex;
  limxsdk::ImuData imuData;           // Shared IMU data
  std::mutex imuDataMutex;
};

} // namespace ability
}  // namespace limxsdk

#endif