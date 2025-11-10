//
// Created by Dongho Kang on 21.01.22.
//

#ifndef CRL_HUMANOID_ROBOTDATA_H
#define CRL_HUMANOID_ROBOTDATA_H

#include <atomic>
#include <mutex>
#include <memory>

#include "crl-basic/utils/mathDefs.h"
#include "crl_humanoid_commons/RobotParameters.h"

namespace crl::humanoid::commons {
    enum class RBJointControlMode {
        PASSIVE_MODE = 0,
        POSITION_MODE,
        VELOCITY_MODE,
        FORCE_MODE,
    };

    /**
     * user command
     */
    struct RobotCommand {
        double targetPositionX = 0;
        double targetPositionY = 0;
        double targetPositionZ = 0;
        double targetOrientationRoll = 0;
        double targetOrientationPitch = 0;
        double targetOrientationYaw = 0;
        double targetForwardSpeed = 0;
        double targetSidewaysSpeed = 0;
        double targetTurningSpeed = 0;
    };

    /**
     * sensor values
     */
    struct RobotSensor {
        // accelerometer = Rbw * (a-g)
        V3D accelerometer = V3D(0, 0, 0);
        // gyroscope = Rbw * w
        V3D gyroscope = V3D(0, 0, 0);
        // imu often has its own mechanism to estimate orientation
        Quaternion imuOrientation = Quaternion::Identity();

        struct RobotJointSensor {
            std::string jointName;
            double jointPos;     // rad
            double jointVel;     // rad/s
            double jointTorque;  // N.m
        };
        std::vector<RobotJointSensor> jointSensors;
    };


    /**
     * full state and estimation info
     */
    struct RobotState {
        P3D basePosition = P3D(0, 0, 0);
        V3D basePositionCov = V3D(0, 0, 0); // covariance of base position
        V3D baseVelocity = V3D(0, 0, 0);
        V3D baseVelocityCov = V3D(0, 0, 0); // covariance of base velocity
        Quaternion baseOrientation = Quaternion::Identity();
        V3D baseOrientationCov = V3D(0, 0, 0); // covariance of base orientation
        V3D baseAngularVelocity = V3D(0, 0, 0);
        V3D baseAngularVelocityCov = V3D(0, 0, 0); // covariance of base angular velocity

        struct RobotJointState {
            std::string jointName;
            double jointPos;  // rad
            double jointVel;  // rad/s
        };
        std::vector<RobotJointState> jointStates;
    };

    struct RobotControlSignal {
        struct RobotJointControlSignal {
            std::string name;
            RBJointControlMode mode = RBJointControlMode::PASSIVE_MODE;
            double desiredPos = 0;
            double desiredSpeed = 0;
            double desiredTorque = 0;
            double stiffness = 0;
            double damping = 0;
        };
        std::vector<RobotJointControlSignal> jointControl;
    };

    struct ProfilingInfo {
        double mainExecutionTime = 0;
        double communicationExecutionTime = 0;
        double controllerExecutionTime = 0;
    };

    class RobotData {
    public:
        std::atomic<double> timeStamp{0};
        std::atomic<bool> softEStop{false};

    protected:
        RobotCommand command;
        RobotSensor sensor;
        RobotState state;
        RobotControlSignal control;

        std::mutex profilingMutex;
        std::mutex commandMutex;
        std::mutex sensorMutex;
        std::mutex stateMutex;
        std::mutex controlMutex;
        std::mutex parameterMutex;

        // monitoring purpose
        ProfilingInfo profilingInfo;

    public:
        RobotData();

        // Constructor that initializes joint information
        RobotData(const std::vector<std::string>& jointNames,
                              const std::vector<double>& defaultJointConf);

        ~RobotData();

        void PropagateJointInformation(std::vector<std::string> jointNames, std::vector<double> defaultJointConf);

        double getTimeStamp() const;

        void advanceInTime(double dt);

        /* ---------------------------------------- Profiling Info ---------------------------------------- */

        ProfilingInfo getProfilingInfo();

        void setProfilingInfo(const ProfilingInfo &status);

        /* ---------------------------------------- User Command ---------------------------------------- */

        RobotCommand getCommand();

        void setCommand(const RobotCommand &command);

        /* ---------------------------------------- Sensor ---------------------------------------- */

        RobotSensor getSensor();

        void setSensor(const RobotSensor &sensor);

        /* ---------------------------------------- State ---------------------------------------- */

        RobotState getRobotState();

        /**
         * Legged robot state estimator should call this.
         */
        void setRobotState(const RobotState &state);


        /* ---------------------------------------- Control ---------------------------------------- */

        RobotControlSignal getControlSignal();

        /**
         * Controller should call this.
         */
        void setControlSignal(const RobotControlSignal &control);

    };

}  // namespace crl::humanoid::commons

#endif  //CRL_HUMANOID_ROBOTDATA_H
