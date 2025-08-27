//
// Created by Dongho Kang on 21.01.22.
//

#ifndef CRL_HUMANOID_UNITREELEGGEDROBOTDATA_H
#define CRL_HUMANOID_UNITREELEGGEDROBOTDATA_H

#include <atomic>
#include <mutex>
#include <memory>

#include "crl-basic/utils/mathDefs.h"
#include "crl_humanoid_commons/RobotParameters.h"

namespace crl::unitree::commons {
    enum class RBJointControlMode {
        PASSIVE_MODE = 0,
        POSITION_MODE,
        VELOCITY_MODE,
        FORCE_MODE,
    };

    /**
     * user command
     */
    struct LeggedRobotCommand {
        double targetForwardSpeed = 0;
        double targetSidewaysSpeed = 0;
        double targetTurningSpeed = 0;
    };

    /**
     * sensor values
     */
    struct LeggedRobotSensor {
        // accelerometer = Rbw * (a-g)
        V3D accelerometer = V3D(0, 0, 0);
        // gyroscope = Rbw * w
        V3D gyroscope = V3D(0, 0, 0);
        // imu often has its own mechanism to estimate orientation
        Quaternion imuOrientation = Quaternion::Identity();

        struct LeggedRobotJointSensor {
            std::string jointName;
            double jointPos;     // rad
            double jointVel;     // rad/s
            double jointTorque;  // N.m
        };
        std::vector<LeggedRobotJointSensor> jointSensors;
    };


    /**
     * full state and estimation info
     */
    struct LeggedRobotState {
        P3D basePosition = P3D(0, 0, 0);
        V3D basePositionCov = V3D(0, 0, 0); // covariance of base position
        V3D baseVelocity = V3D(0, 0, 0);
        V3D baseVelocityCov = V3D(0, 0, 0); // covariance of base velocity
        Quaternion baseOrientation = Quaternion::Identity();
        V3D baseOrientationCov = V3D(0, 0, 0); // covariance of base orientation
        V3D baseAngularVelocity = V3D(0, 0, 0);

        struct LeggedRobotJointState {
            std::string jointName;
            double jointPos;  // rad
            double jointVel;  // rad/s
        };
        std::vector<LeggedRobotJointState> jointStates;
    };

    struct LeggedRobotControlSignal {
        P3D targetBasePos = P3D(0, 0, 0);
        Quaternion targetBaseOrientation = Quaternion::Identity();
        V3D targetBaseVel = V3D(0, 0, 0);
        V3D targetBaseAngularVelocity = V3D(0, 0, 0);
        V3D targetBaseAcc = V3D(0, 0, 0); // desired base acceleration
        V3D targetBaseAngularAcc = V3D(0, 0, 0); // desired base angular acceleration

        struct LeggedRobotJointControlSignal {
            std::string name;
            RBJointControlMode mode = RBJointControlMode::PASSIVE_MODE;
            double desiredPos = 0;
            double desiredSpeed = 0;
            double desiredTorque = 0;
        };
        std::vector<LeggedRobotJointControlSignal> jointControl;
    };

    struct ProfilingInfo {
        double mainExecutionTime = 0;
        double communicationExecutionTime = 0;
        double controllerExecutionTime = 0;
    };

    class UnitreeLeggedRobotData {
    public:
        std::atomic<double> timeStamp{0};
        std::atomic<bool> softEStop{false};

    protected:
        LeggedRobotCommand command;
        LeggedRobotSensor sensor;
        LeggedRobotState state;
        LeggedRobotControlSignal control;

        std::mutex profilingMutex;
        std::mutex commandMutex;
        std::mutex sensorMutex;
        std::mutex stateMutex;
        std::mutex controlMutex;
        std::mutex parameterMutex;

        // monitoring purpose
        ProfilingInfo profilingInfo;

    public:
        UnitreeLeggedRobotData();

        // Constructor that initializes joint information
        UnitreeLeggedRobotData(const std::vector<std::string>& jointNames,
                              const std::vector<double>& defaultJointConf);

        ~UnitreeLeggedRobotData();

        void PropagateJointInformation(std::vector<std::string> jointNames, std::vector<double> defaultJointConf);

        double getTimeStamp() const;

        void advanceInTime(double dt);

        /* ---------------------------------------- Profiling Info ---------------------------------------- */

        ProfilingInfo getProfilingInfo();

        void setProfilingInfo(const ProfilingInfo &status);

        /* ---------------------------------------- User Command ---------------------------------------- */

        LeggedRobotCommand getCommand();

        void setCommand(const LeggedRobotCommand &command);

        /* ---------------------------------------- Sensor ---------------------------------------- */

        LeggedRobotSensor getSensor();

        void setSensor(const LeggedRobotSensor &sensor);

        /* ---------------------------------------- State ---------------------------------------- */

        LeggedRobotState getLeggedRobotState();

        /**
         * Legged robot state estimator should call this.
         */
        void setLeggedRobotState(const LeggedRobotState &state);


        /* ---------------------------------------- Control ---------------------------------------- */

        LeggedRobotControlSignal getControlSignal();

        /**
         * Controller should call this.
         */
        void setControlSignal(const LeggedRobotControlSignal &control);

    };

}  // namespace crl::unitree::commons

#endif  //CRL_HUMANOID_UNITREELEGGEDROBOTDATA_H
