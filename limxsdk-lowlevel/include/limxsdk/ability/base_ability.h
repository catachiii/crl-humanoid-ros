/**
 * @file base_ability.h
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#ifndef BASE_ABILITY_H
#define BASE_ABILITY_H

#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <iostream>
#include <exception>
#include <yaml-cpp/yaml.h>
#include "limxsdk/macros.h"
#include "limxsdk/datatypes.h"
#include "limxsdk/apibase.h"
#include "limxsdk/ability/rate.h"
#include "limxsdk/ability/robot_data.h"
#include "limxsdk/ability/plugin_registry.h"

namespace limxsdk
{
  namespace ability
  {

    class LIMX_SDK_API BaseAbility
    {
    public:
      // Default constructor and destructor
      BaseAbility() = default;

      // Destructor of the BaseAbility class
      virtual ~BaseAbility() = default;

      // Delete copy and move constructors/operators to prevent copying or moving
      BaseAbility(const BaseAbility &) = delete;
      BaseAbility &operator=(const BaseAbility &) = delete;
      BaseAbility(BaseAbility &&) = delete;
      BaseAbility &operator=(BaseAbility &&) = delete;

      // Virtual methods
      virtual bool on_init(const YAML::Node &config) { return true; }
      virtual void on_start() {}
      virtual void on_stop() {}
      virtual void on_main() {}

      // Control methods
      void start()
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (running_)
        {
          std::cout << "Ability already active: " << name_ << std::endl;
          return;
        }

        if (thread_.joinable())
        {
          thread_.join();
        }

        running_ = true;
        thread_ = std::thread(&BaseAbility::_run, this);
        std::cout << "Ability started: " << name_ << std::endl;
      }

      void stop()
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_)
        {
          std::cout << "Ability not active: " << name_ << std::endl;
          return;
        }

        running_ = false;
        if (thread_.joinable())
        {
          thread_.join();
        }
        std::cout << "Ability stopped: " << name_ << std::endl;
      }

      // Status methods
      bool isRunning() const { return running_; }
      std::string getName() const { return name_; }
      std::string getType() const { return type_; }

      // Interface methods
      limxsdk::ImuData get_imu_data() const { return robot_->get_imu_data(); }
      limxsdk::RobotState get_robot_state() const { return robot_->get_robot_state(); }
      limxsdk::ApiBase *get_robot_instance() const { return robot_->get_robot_instance(); }

      void _run()
      {
        try
        {
          get_robot_instance()->publishDiagnostic("ability/" + name_, "start", 0, 0);
          on_start();
          on_main();
          get_robot_instance()->publishDiagnostic("ability/" + name_, "stop", 0, 0);
        }
        catch (const std::exception &e)
        {
          std::cerr << "Ability failed: " << e.what() << std::endl;
          get_robot_instance()->publishDiagnostic("ability/" + name_, "start", -1, 2, std::string("Ability failed: ") + e.what());
        }
        catch (...)
        {
          std::cerr << "Ability failed: Unknown exception" << std::endl;
          get_robot_instance()->publishDiagnostic("ability/" + name_, "start", -1, 2, "Unknown exception");
        }

        on_stop();
        running_ = false;
      }

      std::string name_;
      std::string type_;
      std::atomic<bool> running_;
      std::thread thread_;
      std::mutex mutex_;
      RobotData *robot_;
    };

    namespace path
    {
      /**
       * Retrieves the path of the etc directory.
       * This path is typically used for configuration files.
       * @return The path specified by the environment variable LIMX_ABILITY_ETC_PATH.
       */
      std::string etc();

      /**
       * Retrieves the path of the bin directory.
       * This path is typically used for executable binaries.
       * @return The path specified by the environment variable LIMX_ABILITY_BIN_PATH.
       */
      std::string bin();

      /**
       * Retrieves the path of the lib directory.
       * This path is typically used for shared libraries.
       * @return The path specified by the environment variable LIMX_ABILITY_LIB_PATH.
       */
      std::string lib();

      /**
       * Retrieves the root path of the application.
       * This path serves as the base directory for the application's file structure.
       * @return The path specified by the environment variable LIMX_ABILITY_ROOT_PATH.
       */
      std::string root();
    }
  } // namespace ability
} // namespace limxsdk

#endif
