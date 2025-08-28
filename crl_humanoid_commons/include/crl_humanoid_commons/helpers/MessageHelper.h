//
// Created by Dongho Kang on 14.06.21.
//

#ifndef CRLUNITREE_MESSAGEHELPER_H
#define CRLUNITREE_MESSAGEHELPER_H

#include "crl_humanoid_commons/RobotData.h"
#include "crl_humanoid_msgs/msg/control.hpp"
#include "crl_humanoid_msgs/msg/profiling_info.hpp"
#include "crl_humanoid_msgs/msg/remote.hpp"
#include "crl_humanoid_msgs/msg/sensor.hpp"
#include "crl_humanoid_msgs/msg/state.hpp"

namespace crl::unitree::commons {

    inline void populatePointFromP3D(const crl::P3D& p3d, geometry_msgs::msg::Point& point) {
        point.x = p3d.x;
        point.y = p3d.y;
        point.z = p3d.z;
    }

    inline void populateP3DFromPoint(const geometry_msgs::msg::Point& point, crl::P3D& p3d) {
        p3d.x = point.x;
        p3d.y = point.y;
        p3d.z = point.z;
    }

    inline void populateQuaternionFromQuaternion(const crl::Quaternion& q, geometry_msgs::msg::Quaternion& quat) {
        quat.w = q.w();
        quat.x = q.x();
        quat.y = q.y();
        quat.z = q.z();
    }

    inline void populateQuaternionFromQuaternion(const geometry_msgs::msg::Quaternion& quat, crl::Quaternion& q) {
        q.w() = quat.w;
        q.x() = quat.x;
        q.y() = quat.y;
        q.z() = quat.z;
    }

    inline void populateVectorFromV3D(const crl::V3D& v3d, geometry_msgs::msg::Vector3& vector) {
        vector.x = v3d.x();
        vector.y = v3d.y();
        vector.z = v3d.z();
    }

    inline void populateV3DFromVector(const geometry_msgs::msg::Vector3& vector, crl::V3D& v3d) {
        v3d.x() = vector.x;
        v3d.y() = vector.y;
        v3d.z() = vector.z;
    }

    inline void populatePoseCovarianceFromV3D(const crl::V3D& v3d, std::array<double, 36>& covariance) {
        // note. populate only position
        covariance[0] = v3d.x();
        covariance[7] = v3d.y();
        covariance[14] = v3d.z();
    }

    inline void populateV3DFromPoseCovariance(const std::array<double, 36>& covariance, crl::V3D& v3d) {
        // note. populate only position
        v3d.x() = covariance[0];
        v3d.y() = covariance[7];
        v3d.z() = covariance[14];
    }

    inline void populateRemoteMessageFromData(const crl::unitree::commons::LeggedRobotCommand& data, crl_humanoid_msgs::msg::Remote& remote) {
        remote.speed_forward = data.targetForwardSpeed;
        remote.speed_sideways = data.targetSidewaysSpeed;
        remote.speed_turning = data.targetTurningSpeed;
    }

    inline void populateDataFromRemoteMessage(const crl_humanoid_msgs::msg::Remote& remote, crl::unitree::commons::LeggedRobotCommand& data) {
        data.targetForwardSpeed = remote.speed_forward;
        data.targetSidewaysSpeed = remote.speed_sideways;
        data.targetTurningSpeed = remote.speed_turning;
    }

    inline void populateSensorMessageFromData(const crl::unitree::commons::LeggedRobotSensor& data, crl_humanoid_msgs::msg::Sensor& sensor) {
        populateVectorFromV3D(data.accelerometer, sensor.imu.linear_acceleration);
        populateVectorFromV3D(data.gyroscope, sensor.imu.angular_velocity);
        populateQuaternionFromQuaternion(data.imuOrientation, sensor.imu.orientation);
        int nj = data.jointSensors.size();
        sensor.joint.position.resize(nj);
        sensor.joint.velocity.resize(nj);
        sensor.joint.name.resize(nj);
        sensor.joint.effort.resize(nj);
        for (int i = 0; i < nj; i++) {
            sensor.joint.name[i] = data.jointSensors[i].jointName;
            sensor.joint.position[i] = data.jointSensors[i].jointPos;
            sensor.joint.velocity[i] = data.jointSensors[i].jointVel;
            sensor.joint.effort[i] = data.jointSensors[i].jointTorque;
        }
    }

    inline void populateDataFromSensorMessage(const crl_humanoid_msgs::msg::Sensor& sensor, crl::unitree::commons::LeggedRobotSensor& data) {
        populateV3DFromVector(sensor.imu.linear_acceleration, data.accelerometer);
        populateV3DFromVector(sensor.imu.angular_velocity, data.gyroscope);
        populateQuaternionFromQuaternion(sensor.imu.orientation, data.imuOrientation);
        int nj = sensor.joint.name.size();
        data.jointSensors.resize(nj);
        for (int i = 0; i < nj; i++) {
            data.jointSensors[i].jointName = sensor.joint.name[i];
            data.jointSensors[i].jointPos = sensor.joint.position[i];
            data.jointSensors[i].jointVel = sensor.joint.velocity[i];
            data.jointSensors[i].jointTorque = sensor.joint.effort[i];
        }
    }

    inline void populateStateMessageFromData(const crl::unitree::commons::LeggedRobotState& data, crl_humanoid_msgs::msg::State& state) {
        populatePointFromP3D(data.basePosition, state.base_pose.pose.position);
        populateQuaternionFromQuaternion(data.baseOrientation, state.base_pose.pose.orientation);
        populateVectorFromV3D(data.baseVelocity, state.base_twist.twist.linear);
        populateVectorFromV3D(data.baseAngularVelocity, state.base_twist.twist.angular);
        // TODO: populate the covariance as well.
        int nj = data.jointStates.size();
        state.joint.name.resize(nj);
        state.joint.position.resize(nj);
        state.joint.velocity.resize(nj);
        for (int i = 0; i < nj; i++) {
            state.joint.name[i] = data.jointStates[i].jointName;
            state.joint.position[i] = data.jointStates[i].jointPos;
            state.joint.velocity[i] = data.jointStates[i].jointVel;
        }
    }

    inline void populateDataFromStateMessage(const crl_humanoid_msgs::msg::State& state, crl::unitree::commons::LeggedRobotState& data) {
        populateP3DFromPoint(state.base_pose.pose.position, data.basePosition);
        populateQuaternionFromQuaternion(state.base_pose.pose.orientation, data.baseOrientation);
        populateV3DFromVector(state.base_twist.twist.linear, data.baseVelocity);
        populateV3DFromVector(state.base_twist.twist.angular, data.baseAngularVelocity);
        // TODO: populate the covariance as well.
        int nj = state.joint.name.size();
        data.jointStates.resize(nj);
        for (int i = 0; i < nj; i++) {
            data.jointStates[i].jointName = state.joint.name[i];
            data.jointStates[i].jointPos = state.joint.position[i];
            data.jointStates[i].jointVel = state.joint.velocity[i];
        }
    }

    inline void populateControlMessageFromData(const crl::unitree::commons::LeggedRobotControlSignal& data, crl_humanoid_msgs::msg::Control& control) {
        int nj = data.jointControl.size();
        control.joint.position.resize(nj);
        control.joint.velocity.resize(nj);
        control.joint.effort.resize(nj);
        control.joint.stiffness.resize(nj);
        control.joint.damping.resize(nj);
        control.joint.name.resize(nj);
        control.joint.mode.resize(nj);
        for (int i = 0; i < nj; i++) {
            control.joint.name[i] = data.jointControl[i].name;
            control.joint.mode[i] = static_cast<int32_t>(data.jointControl[i].mode);
            control.joint.position[i] = data.jointControl[i].desiredPos;
            control.joint.velocity[i] = data.jointControl[i].desiredSpeed;
            control.joint.effort[i] = data.jointControl[i].desiredTorque;
            control.joint.stiffness[i] = data.jointControl[i].stiffness;
            control.joint.damping[i] = data.jointControl[i].damping;
        }
    }

    inline void populateDataFromControlMessage(const crl_humanoid_msgs::msg::Control& control, crl::unitree::commons::LeggedRobotControlSignal& data) {
        int nj = control.joint.name.size();
        data.jointControl.resize(nj);
        for (int i = 0; i < nj; i++) {
            data.jointControl[i].name = control.joint.name[i];
            data.jointControl[i].mode = static_cast<crl::unitree::commons::RBJointControlMode>(control.joint.mode[i]);
            data.jointControl[i].desiredPos = control.joint.position[i];
            data.jointControl[i].desiredSpeed = control.joint.velocity[i];
            data.jointControl[i].desiredTorque = control.joint.effort[i];
            data.jointControl[i].stiffness = control.joint.stiffness[i];
            data.jointControl[i].damping = control.joint.damping[i];
        }
    }

    inline void populateProfilingInfoMessageFromData(const crl::unitree::commons::ProfilingInfo& data, crl_humanoid_msgs::msg::ProfilingInfo& profilingInfo) {
        profilingInfo.main_execution_time = data.mainExecutionTime;
        profilingInfo.comm_execution_time = data.communicationExecutionTime;
        profilingInfo.cont_execution_time = data.controllerExecutionTime;
    }

    inline void populateDataFromProfilingInfoMessage(const crl_humanoid_msgs::msg::ProfilingInfo& profilingInfo, crl::unitree::commons::ProfilingInfo& data) {
        data.mainExecutionTime = profilingInfo.main_execution_time;
        data.communicationExecutionTime = profilingInfo.comm_execution_time;
        data.controllerExecutionTime = profilingInfo.cont_execution_time;
    }
}  // namespace crl::unitree::commons

#endif  //CRLUNITREE_MESSAGEHELPER_H
