import mujoco
import numpy as np
import mujoco_viewer
import casadi_ik
import time
import pygame
import os
import DrEmpower as dr

os.environ["SDL_JOYSTICK_DEVICE"] = "/dev/input/js0"

SCENE_XML_PATH = '/home/ycn/mujoco/teleoperation_robot/models/robot_arm/scene.xml'
ARM_XML_PATH = '/home/ycn/mujoco/teleoperation_robot/models/robot_arm/robot_arm.xml'


class XboxController:
    """Xbox手柄控制器（末端位姿控制 + 姿态旋转）"""
    
    def __init__(self):
        self.x = -0.2
        self.y = 0.04
        self.z = 0.3
        self.R = self._rpy_to_matrix(np.pi/2, 0, -np.pi/2)
        
        self.x_min, self.x_max = -0.4, 0.4
        self.y_min, self.y_max = -0.4, 0.4
        self.z_min, self.z_max = 0.05, 0.35
        
        self.pos_sensitivity = 0.001
        self.ori_sensitivity = 0.007
        self.deadzone = 0.1
        self.controller = self.init_controller()
        
    def _rpy_to_matrix(self, roll, pitch, yaw):
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        Rz = np.array([[cy, -sy, 0],
                       [sy,  cy, 0],
                       [0,    0, 1]])
        Ry = np.array([[cp, 0, sp],
                       [0, 1, 0],
                       [-sp, 0, cp]])
        Rx = np.array([[1, 0, 0],
                       [0, cr, -sr],
                       [0, sr,  cr]])
        return Rz @ Ry @ Rx

    def _matrix_to_rpy(self, R):
        pitch = -np.arcsin(R[2, 0])
        if abs(R[2, 0]) < 0.999999:
            roll = np.arctan2(R[2, 1], R[2, 2])
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = 0
            yaw = np.arctan2(-R[0, 1], R[1, 1])
        return roll, pitch, yaw

    def _axis_angle_to_matrix(self, axis, angle):
        if angle == 0:
            return np.eye(3)
        axis = axis / np.linalg.norm(axis)
        K = np.array([[0, -axis[2], axis[1]],
                      [axis[2], 0, -axis[0]],
                      [-axis[1], axis[0], 0]])
        R = np.eye(3) + np.sin(angle)*K + (1-np.cos(angle))*(K @ K)
        return R

    def init_controller(self):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            print("未检测到任何游戏杆设备")
            return None
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"检测到手柄: {joystick.get_name()}")
        return joystick
        
    def is_connected(self):
        return self.controller is not None
        
    def handle_input(self, arm, current_qpos):
        if not self.is_connected():
            return
        pygame.event.pump()
        x_axis = self.controller.get_axis(0)
        y_axis = self.controller.get_axis(3)
        z_axis = -self.controller.get_axis(1)
        if abs(x_axis) < self.deadzone: x_axis = 0.0
        if abs(y_axis) < self.deadzone: y_axis = 0.0
        if abs(z_axis) < self.deadzone: z_axis = 0.0

        delta_local = np.array([-x_axis, -y_axis, z_axis]) * self.pos_sensitivity
        R_ee = self.R
        delta_world = R_ee @ delta_local
        
        self.x = np.clip(self.x + delta_world[0], self.x_min, self.x_max)
        self.y = np.clip(self.y + delta_world[1], self.y_min, self.y_max)
        self.z = np.clip(self.z + delta_world[2], self.z_min, self.z_max)

        hat = self.controller.get_hat(0)
        pitch_axis = -self.controller.get_axis(2)
        d_yaw = hat[0] * self.ori_sensitivity
        d_pitch = pitch_axis * self.ori_sensitivity
        d_roll = hat[1] * self.ori_sensitivity
        R_inc = (
            self._axis_angle_to_matrix(np.array([1,0,0]), d_roll) @
            self._axis_angle_to_matrix(np.array([0,1,0]), d_pitch) @
            self._axis_angle_to_matrix(np.array([0,0,1]), d_yaw)
        )
        self.R = self.R @ R_inc

    def get_pose_target(self):
        roll, pitch, yaw = self._matrix_to_rpy(self.R)
        return self.x, self.y, self.z, roll, pitch, yaw
        
    def cleanup(self):
        pygame.quit()


class RobotController(mujoco_viewer.CustomViewer):
    """仿真界面 + 实物机械臂控制"""
    
    def __init__(self, scene_path, arm_path, controller, init_dof):
        super().__init__(scene_path, distance=1.5, azimuth=135, elevation=-30)
        self.controller = controller
        self.arm = casadi_ik.Kinematics("grasp_point")
        self.arm.buildFromMJCF(arm_path)
        self.last_dof = init_dof  # ✅ 初始化为实物机械臂角度（弧度）
        self.frame_count = 0

    def runFunc(self):
        self.frame_count += 1
        self.controller.handle_input(self.arm, self.last_dof)
        x, y, z, roll, pitch, yaw = self.controller.get_pose_target()

        tf_target = self.build_transform(x, y, z, roll, pitch, yaw)
        dof, info = self.arm.ik(tf_target, current_arm_motor_q=self.last_dof)
        self.last_dof = dof

        # 计算误差
        tf_actual = self.arm.fk(self.last_dof)
        pos_actual = tf_actual[:3, 3]
        pos_target = tf_target[:3, 3]
        delta_pos = pos_target - pos_actual

        R_actual = tf_actual[:3, :3]
        R_target = tf_target[:3, :3]
        R_diff = R_target.T @ R_actual
        roll_err, pitch_err, yaw_err = self.controller._matrix_to_rpy(R_diff)

        # 输出信息
        print(f"\n🎯 目标位姿: x={x:.3f}, y={y:.3f}, z={z:.3f}, "
              f"roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")
        print("关节角度 (rad):", np.round(dof[:6],4))
        print(f"位置误差 Δpos [m]: {delta_pos}")
        print(f"姿态误差 Δrpy [rad]: roll={roll_err:.4f}, pitch={pitch_err:.4f}, yaw={yaw_err:.4f}")

        # 冻结逻辑
        if self.frame_count > 60:
            POS_ERR_LIMIT = 0.02
            ORI_ERR_LIMIT = 0.1
            pos_err_norm = np.linalg.norm(delta_pos)
            ori_err_norm = np.linalg.norm([roll_err, pitch_err, yaw_err])
            if pos_err_norm > POS_ERR_LIMIT or ori_err_norm > ORI_ERR_LIMIT:
                print("⚠️ [警告] 末端误差超限，冻结目标位姿！")
                self.controller.x, self.controller.y, self.controller.z = pos_actual
                self.controller.R = R_actual.copy()
                tf_target[:3, 3] = pos_actual
                tf_target[:3, :3] = R_actual
                dof[:6] = self.last_dof[:6]

        # 下发到实物机械臂（弧度 → 角度）
        deg_list = np.degrees(dof[:6]).tolist()
        dr.set_angles(id_list=[1,2,3,4,5,6], angle_list=deg_list, speed=50, param=10, mode=0)

        # 更新仿真
        self.data.qpos[:6] = dof[:6]
        mujoco.mj_step(self.model, self.data)
        time.sleep(0.01)

    def build_transform(self, x, y, z, roll, pitch, yaw):
        R = self.controller._rpy_to_matrix(roll, pitch, yaw)
        tf = np.eye(4)
        tf[:3, :3] = R
        tf[:3, 3] = [x, y, z]
        return tf


# 等待机械臂到达初始位置
def wait_until_reached(target_deg, tol=1.0, timeout=15):
    start = time.time()
    while time.time() - start < timeout:
        cur_deg = []
        for i in range(1, 7):
            try:
                angle = dr.get_angle(id_num=i)
            except Exception as e:
                print(f"读取关节{i}角度失败：{e}")
                angle = 9999
            cur_deg.append(angle)
        diffs = [abs(a - b) for a, b in zip(cur_deg, target_deg)]
        if max(diffs) < tol:
            print("\n机械臂已到达初始位置。")
            return True
        print(f"当前最大误差: {max(diffs):.2f}°", end='\r')
        time.sleep(0.3)

    print("\n⚠️ 等待超时，机械臂可能未完全到达目标位置。")
    return False


if __name__ == "__main__":

    # 发送初始位置
    target_deg = [117.5,56.35,115.79,57.89,-113.5,0]
    dr.set_angles(id_list=[1,2,3,4,5,6], angle_list=target_deg, speed=2, param=10, mode=1)

    print("等待机械臂到达初始位置...")
    if not wait_until_reached(target_deg, tol=1.0, timeout=15):
        print("机械臂未到达初始位置，程序退出。")
        exit(1)

    # 初始化 Xbox 控制器
    controller = XboxController()
    if not controller.is_connected():
        print("控制器连接失败，程序将退出。")
        exit(1)

    # 初始化 RobotController，读取实物角度作为 last_dof
    cur_deg = [dr.get_angle(i) for i in range(1,7)]
    last_dof_init = np.radians(cur_deg)  # 弧度
    robot = RobotController(SCENE_XML_PATH, ARM_XML_PATH, controller, init_dof=last_dof_init)

    try:
        robot.run_loop()  # 启动仿真界面
    finally:
        controller.cleanup()
