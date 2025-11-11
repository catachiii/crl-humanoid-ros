import rclpy
from rclpy.node import Node
import time
import os
import numpy as np
import onnxruntime as ort

from crl_humanoid_msgs.msg import Monitor
from crl_humanoid_msgs.msg import Control
from .data import MODELS_DIR


def quat_rotate_inverse(quat, vec):
    """Apply inverse quaternion rotation to a vector (NumPy version).

    Args:
        quat: Quaternion in (w, x, y, z) format. Shape (4,)
        vec: Vector in (x, y, z) format. Shape (3,)

    Returns:
        Rotated vector. Shape (3,)
    """
    # Extract quaternion components
    w, x, y, z = quat[0], quat[1], quat[2], quat[3]
    xyz = quat[1:]

    # Compute: v + 2 * cross(xyz, cross(xyz, v) + w * v)
    t = np.cross(xyz, vec) * 2
    return vec - w * t + np.cross(xyz, t)


def euler_xyz_from_quat(quat):
    """Extract roll, pitch, yaw from quaternion in (w, x, y, z) format.

    Args:
        quat: Quaternion in (w, x, y, z) format. Shape (4,)

    Returns:
        Tuple of (roll, pitch, yaw) in radians
    """
    w, x, y, z = quat[0], quat[1], quat[2], quat[3]

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if np.abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def wrap_to_pi(angle):
    """Wrap angle to [-pi, pi] range.

    Args:
        angle: Angle in radians

    Returns:
        Wrapped angle in [-pi, pi]
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi


class goal_amp_controller(Node):

    def __init__(self):
        super().__init__('vanillarl_unitree_pycontroller')
        self.publisher = self.create_publisher(Control, 'py_control', 10)
        timer_period = 1 / 60  # seconds
        self.timer = self.create_timer(timer_period, self.publisher_callback)
        self.subscription = self.create_subscription(
            Monitor, 'monitor', self.subscriber_callback, 10)
        self.subscription  # prevent unused variable warning

        # Define constants and parameters
        model_path = os.path.join(MODELS_DIR, 'goal_amp/model_10000.onnx')
        self.kps = np.array([40.1792, 40.1792, 40.1792, 99.0984, 99.0984, 28.5012, 40.1792, 40.1792,
                             28.5012, 99.0984, 99.0984, 14.2506, 14.2506, 28.5012, 28.5012, 14.2506,
                             14.2506, 28.5012, 28.5012, 14.2506, 14.2506, 14.2506, 14.2506, 14.2506,
                             14.2506, 16.7783, 16.7783, 16.7783, 16.7783])  # in policy order
        self.kds = np.array([2.5579, 2.5579, 2.5579, 6.3088, 6.3088, 1.8144, 2.5579, 2.5579, 1.8144,
                             6.3088, 6.3088, 0.9072, 0.9072, 1.8144, 1.8144, 0.9072, 0.9072, 1.8144,
                             1.8144, 0.9072, 0.9072, 0.9072, 0.9072, 0.9072, 0.9072, 1.0681, 1.0681,
                             1.0681, 1.0681])  # in policy order
        self.default_angles = np.array([-0.1000, -0.1000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,
                                        0.0000,  0.3000,  0.3000,  0.3500,  0.3500, -0.2000, -0.2000,  0.1600,
                                        -0.1600,  0.0000,  0.0000,  0.0000,  0.0000,  0.8700,  0.8700,  0.0000,
                                        0.0000,  0.0000,  0.0000,  0.0000,  0.0000])  # in policy order
        self.action_scale = np.array([0.5475, 0.5475, 0.5475, 0.3507, 0.3507, 0.4386, 0.5475, 0.5475, 0.4386,
                                      0.3507, 0.3507, 0.4386, 0.4386, 0.4386, 0.4386, 0.4386, 0.4386, 0.4386,
                                      0.4386, 0.4386, 0.4386, 0.4386, 0.4386, 0.4386, 0.4386, 0.0745, 0.0745,
                                      0.0745, 0.0745])  # in policy order
        self.num_actions = 29
        self.num_obs = 96
        self.num_history = 5
        self.num_commands = 3

        # Create policy
        self.session = ort.InferenceSession(model_path)
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name

        # Constants
        self.GRAVITY_VEC_W = np.array([0.0, 0.0, -1.0], dtype=np.float32)

        # Joint names in policy order
        self.joint_names = [
            'left_hip_pitch_joint',
            'right_hip_pitch_joint',
            'waist_yaw_joint',
            'left_hip_roll_joint',
            'right_hip_roll_joint',
            'waist_roll_joint',
            'left_hip_yaw_joint',
            'right_hip_yaw_joint',
            'waist_pitch_joint',
            'left_knee_joint',
            'right_knee_joint',
            'left_shoulder_pitch_joint',
            'right_shoulder_pitch_joint',
            'left_ankle_pitch_joint',
            'right_ankle_pitch_joint',
            'left_shoulder_roll_joint',
            'right_shoulder_roll_joint',
            'left_ankle_roll_joint',
            'right_ankle_roll_joint',
            'left_shoulder_yaw_joint',
            'right_shoulder_yaw_joint',
            'left_elbow_joint',
            'right_elbow_joint',
            'left_wrist_roll_joint',
            'right_wrist_roll_joint',
            'left_wrist_pitch_joint',
            'right_wrist_pitch_joint',
            'left_wrist_yaw_joint',
            'right_wrist_yaw_joint'
        ]

        # create buffers for raw data (updated by subscriber)
        self.raw_root_pos = np.zeros(3, dtype=np.float32)
        self.raw_root_quat_w = np.zeros(4, dtype=np.float32)
        self.raw_root_ang_vel_b = np.zeros(3, dtype=np.float32)
        self.raw_joint_pos = np.zeros(self.num_actions, dtype=np.float32)
        self.raw_joint_vel = np.zeros(self.num_actions, dtype=np.float32)
        self.raw_command = np.zeros(self.num_commands, dtype=np.float32)

        # create buffers for processed data (updated by publisher)
        self.obs = np.zeros(self.num_obs, dtype=np.float32)
        self.obs_history = np.zeros(
            (self.num_history, self.num_obs), dtype=np.float32)
        self.action = np.zeros(self.num_actions, dtype=np.float32)

    def subscriber_callback(self, msg):
        self.raw_root_pos[0] = msg.state.base_pose.pose.position.x
        self.raw_root_pos[1] = msg.state.base_pose.pose.position.y
        self.raw_root_pos[2] = msg.state.base_pose.pose.position.z
        self.raw_root_quat_w[0] = msg.sensor.imu.orientation.w
        self.raw_root_quat_w[1] = msg.sensor.imu.orientation.x
        self.raw_root_quat_w[2] = msg.sensor.imu.orientation.y
        self.raw_root_quat_w[3] = msg.sensor.imu.orientation.z
        self.raw_root_ang_vel_b[0] = msg.sensor.imu.angular_velocity.x
        self.raw_root_ang_vel_b[1] = msg.sensor.imu.angular_velocity.y
        self.raw_root_ang_vel_b[2] = msg.sensor.imu.angular_velocity.z
        for i in range(self.num_actions):
            self.raw_joint_pos[i] = msg.sensor.joint.position[i]
            self.raw_joint_vel[i] = msg.sensor.joint.velocity[i]
        self.raw_command[0] = msg.remote.position_x
        self.raw_command[1] = msg.remote.position_y
        self.raw_command[2] = msg.remote.orientation_yaw

        # Build observation vector from raw sensor data
        pelvis_ang_vel_b = self.raw_root_ang_vel_b
        gravity_b = quat_rotate_inverse(
            self.raw_root_quat_w, self.GRAVITY_VEC_W)
        joint_pos = self.raw_joint_pos - self.default_angles
        joint_vel = self.raw_joint_vel
        actions = self.action

        # Convert global target to local frame
        current_pos = self.raw_root_pos[:2]  # xy position
        _, _, current_yaw = euler_xyz_from_quat(
            self.raw_root_quat_w)  # extract yaw

        target_pos = self.raw_command[:2]  # global target xy
        target_yaw = self.raw_command[2]  # global target yaw

        # Transform target position to local frame
        target_pos_b = np.zeros(2, dtype=np.float32)
        target_pos_b[0] = np.cos(current_yaw) * (target_pos[0] - current_pos[0]) + \
            np.sin(current_yaw) * (target_pos[1] - current_pos[1])
        target_pos_b[1] = -np.sin(current_yaw) * (target_pos[0] - current_pos[0]) + \
            np.cos(current_yaw) * (target_pos[1] - current_pos[1])

        # Transform target yaw to local frame
        target_yaw_b = wrap_to_pi(target_yaw - current_yaw)

        # Build commands in local frame
        commands = np.array(
            [target_pos_b[0], target_pos_b[1], target_yaw_b], dtype=np.float32)

        self.obs = np.concatenate([
            pelvis_ang_vel_b,
            gravity_b,
            joint_pos,
            joint_vel,
            actions,
            commands,
        ], axis=-1).astype(np.float32)

    def publisher_callback(self):
        # Update observation history
        self.obs_history = np.concatenate([
            self.obs_history[1:, :],
            self.obs.reshape(1, -1)
        ], axis=0)

        # Inference policy using ONNX Runtime
        obs_history_flat = self.obs_history.reshape(-1).astype(np.float32)
        onnx_input = {self.input_name: obs_history_flat.reshape(1, -1)}
        onnx_output = self.session.run([self.output_name], onnx_input)
        self.action = onnx_output[0].flatten()

        # Post process action
        joint_target = self.default_angles + self.action * self.action_scale

        # Publish control message
        msg = Control()
        msg.joint.name = []
        msg.joint.position = []
        msg.joint.velocity = []
        msg.joint.effort = []
        for i in range(self.num_actions):
            msg.joint.name.append(self.joint_names[i])
            msg.joint.mode.append(1)  # position control mode
            msg.joint.position.append(float(joint_target[i]))
            msg.joint.velocity.append(0.0)
            msg.joint.effort.append(0.0)
            msg.joint.stiffness.append(self.kps[i])
            msg.joint.damping.append(self.kds[i])

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    pycontroller = goal_amp_controller()

    rclpy.spin(pycontroller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pycontroller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
