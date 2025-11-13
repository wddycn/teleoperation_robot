import mujoco
import numpy as np
import mujoco_viewer
import casadi_ik
import time
import pygame
import os
import threading
import DrEmpower as dr
import math
import random
import sys


# ----------------- 可调参数 -----------------
TRANSLATION_MIN_MOVE = 0.0008      # m，最小末端平移量，低于此值不触发 IK 计算
ROTATION_MIN_MOVE = 0.3 * math.pi/180.0  # rad，最小末端旋转量（约0.3度）低于此值不触发 IK
JOINT_SEND_THRESHOLD_DEG = 0.5     # deg，若关节角变化小于该值则不下发命令
MIN_SEND_INTERVAL = 0.08           # s，最小下发间隔（节流，防止过快发送）
AXIS_EMA_ALPHA = 0.35              # EMA平滑系数(0~1)，用于摇杆输入平滑
JOY_DEADZONE = 0.12                # 摇杆死区，小于该值忽略输入
IK_RETRY_TRIES = 6                 # IK失败时的扰动重试次数
IK_PERTURB_STD = 0.05              # rad，IK初始seed扰动标准差（约2.8°）
REALARM_SEND_HZ = 20               # 实物下发频率上限（Hz）
REALARM_MIN_CHANGE_DEG = 0.3       # deg，实物线程再次判断的最小变化阈值
REALARM_SPEED = 4                 # 实物执行速度（mode=0时建议较大）
REALARM_PARAM = 10                 # 实物控制参数（根据SDK定义）
REALARM_MODE = 0                   # 模式选择：0为“track”实时跟随模式
# --------------------------------------------

motors_key = [
    "shoulder_pan",   # 1号关节：肩关节水平旋转
    "shoulder_lift",  # 2号关节：肩关节抬升
    "elbow_flex",     # 3号关节：肘部弯曲
    "wrist_flex",     # 4号关节：腕部俯仰
    "wrist_roll",     # 5号关节：腕部旋转
    "gripper"         # 6号关节：夹爪
]

# =============================
# ======== 实物异步接口 =======
# =============================
class RealArmInterface:
    def __init__(self,
                 speed=REALARM_SPEED,              # 执行速度
                 param=REALARM_PARAM,              # SDK自定义参数
                 mode=REALARM_MODE,                # 控制模式
                 min_change_deg=REALARM_MIN_CHANGE_DEG,  # 下发最小变化阈值
                 send_hz=REALARM_SEND_HZ,          # 最大发送频率
                 min_send_interval=MIN_SEND_INTERVAL):  # 最小下发间隔
        self.speed = speed
        self.param = param
        self.mode = mode
        self.min_change_deg = min_change_deg
        self.period = 1.0 / send_hz                # 后台循环周期（秒）
        self.last_cmd = None                       # 上一次待发送的角度列表（度）
        self.last_sent = None                      # 上一次实际下发的角度列表（度）
        self.running = True                        # 线程运行标志
        self.min_send_interval = min_send_interval  # 下发节流间隔
        self._last_send_time = 0.0                 # 上次下发时间戳

        # 启动后台线程执行下发任务
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()
        print("✅ 实物机械臂异步线程已启动")

    def send_angles_async(self, joint_deg):
        """主线程调用：更新目标角度（覆盖式）"""  # 仅更新缓存，不立即下发
        self.last_cmd = list(map(float, joint_deg))  # 确保为float类型列表

    def _should_send_now(self, cmd):
        """判断是否需要立即发送命令"""  # 根据角度变化和时间间隔判定
        if cmd is None:
            return False  # 无命令则不发
        now = time.time()
        if (now - self._last_send_time) < self.min_send_interval:
            return False  # 未到最小间隔则不发
        if self.last_sent is None:
            return True   # 第一次下发必须发送
        diffs = np.abs(np.array(cmd) - np.array(self.last_sent))  # 计算角度差
        return float(np.max(diffs)) >= self.min_change_deg        # 若变化超过阈值则允许发送

    def _worker(self):
        """后台线程：限频下发角度指令"""  # 自动判断是否下发
        while self.running:
            cmd = self.last_cmd  # 读取最新目标角度（不加锁，简单读）
            if cmd is not None and self._should_send_now(cmd):
                try:
                    ok = dr.set_angles(               # 调用SDK发送角度命令
                        id_list=[1,2,3,4,5,6],       # 机械臂关节ID列表
                        angle_list=cmd,              # 角度目标（度）
                        speed=self.speed,            # 执行速度
                        param=self.param,            # 其他控制参数
                        mode=self.mode               # 控制模式
                    )
                    if not ok:
                        print("[RealArm] set_angles returned False")  # SDK返回失败
                    else:
                        self.last_sent = cmd.copy()                  # 更新最后下发记录
                        self._last_send_time = time.time()           # 更新时间戳
                        print(f"[send→real] {np.round(cmd,2)}")      # 打印下发角度
                except Exception as e:
                    print(f"[error] 下发失败: {e}")                   # 异常打印
            time.sleep(self.period)  # 控制循环频率（防止CPU占用过高）

    def stop(self):
        """停止后台线程"""  # 退出时调用，安全关闭
        self.running = False
        self.thread.join()  # 等待线程结束
        print("🧹 实物线程已关闭")


# =============================
# ======== 手柄控制类 =========
# =============================
class XboxController:
    def __init__(self):
        # 初始位姿（末端执行器）
        self.x, self.y, self.z = -0.2, 0.04, 0.3  # 初始位置坐标(x, y, z)
        self.R = self._rpy_to_matrix(np.pi/2, 0, -np.pi/2)  # 初始旋转矩阵，由RPY角转换而来
        # 位置限制（各轴运动范围）
        self.x_min, self.x_max = -0.4, 0.4  # x轴最小值和最大值
        self.y_min, self.y_max = -0.4, 0.4  # y轴最小值和最大值
        self.z_min, self.z_max = 0.05, 0.4  # z轴最小值和最大值
        self.control_enabled = True  # 控制使能标志（是否允许下发指令到机械臂）

        # 灵敏度参数
        self.pos_sensitivity = 0.0015  # 位置控制灵敏度
        self.ori_sensitivity = 0.007  # 姿态控制灵敏度
        # 抖动控制参数
        self.deadzone = JOY_DEADZONE  # 死区阈值（小于该值的输入视为无效，防止抖动）
        self.axis_ema_alpha = AXIS_EMA_ALPHA  # EMA平滑系数（用于平滑摇杆输入）
        # 摇杆轴数据存储 + EMA平滑结果
        self.joy_axes = [0.0] * 8  # 存储处理后的摇杆轴数据（共8个轴）
        self.joy_axes_ema = [0.0] * 8  # 存储EMA平滑后的原始摇杆轴数据

        self.controller = self.init_controller()  # 初始化手柄

    def _rpy_to_matrix(self, roll, pitch, yaw):
        """将RPY角（滚转、俯仰、偏航）转换为旋转矩阵"""
        cr, sr = np.cos(roll), np.sin(roll)  # 滚转角的余弦和正弦值
        cp, sp = np.cos(pitch), np.sin(pitch)  # 俯仰角的余弦和正弦值
        cy, sy = np.cos(yaw), np.sin(yaw)  # 偏航角的余弦和正弦值
        Rz = np.array([[cy, -sy, 0],[sy, cy, 0],[0,0,1]])  # 绕z轴旋转的矩阵
        Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])  # 绕y轴旋转的矩阵
        Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])  # 绕x轴旋转的矩阵
        return Rz @ Ry @ Rx  # 组合旋转矩阵（先滚转、再俯仰、最后偏航）

    def _matrix_to_rpy(self, R):
        """将旋转矩阵转换为RPY角（滚转、俯仰、偏航）"""
        pitch = -np.arcsin(R[2,0])  # 计算俯仰角（从旋转矩阵第3行第1列提取）
        # 避免奇异值（当俯仰角接近±90度时）
        if abs(R[2,0]) < 0.999999:
            roll = np.arctan2(R[2,1], R[2,2])  # 计算滚转角（使用第3行第2和3列）
            yaw = np.arctan2(R[1,0], R[0,0])   # 计算偏航角（使用第1列第2和1行）
        else:
            roll = 0  # 奇异情况下滚转角设为0
            yaw = np.arctan2(-R[0,1], R[1,1])  # 奇异情况下重新计算偏航角
        return roll, pitch, yaw  # 返回RPY角

    def _axis_angle_to_matrix(self, axis, angle):
        """将轴角表示（旋转轴+旋转角）转换为旋转矩阵"""
        if angle == 0: return np.eye(3)  # 旋转角为0时返回单位矩阵
        axis = axis / np.linalg.norm(axis)  # 归一化旋转轴
        # 构造反对称矩阵K
        K = np.array([[0,-axis[2],axis[1]],[axis[2],0,-axis[0]],[-axis[1],axis[0],0]])
        # 使用罗德里格斯公式计算旋转矩阵
        return np.eye(3) + np.sin(angle)*K + (1-np.cos(angle))*(K@K)

    def init_controller(self):
        """初始化手柄设备"""
        pygame.init(); pygame.joystick.init()  # 初始化pygame和手柄模块
        if pygame.joystick.get_count() == 0:  # 检查是否有手柄连接
            print("未检测到任何游戏杆设备")
            return None
        j = pygame.joystick.Joystick(0); j.init()  # 初始化第一个检测到的手柄
        # 打印手柄信息
        print(f"检测到手柄: {j.get_name()}"); print(f"轴: {j.get_numaxes()}, 按钮: {j.get_numbuttons()}")
        return j  # 返回初始化后的手柄对象

    def is_connected(self): return self.controller is not None  # 检查手柄是否连接

    def handle_input(self, arm, qpos):
        """读取手柄输入并用EMA平滑处理，同时应用死区过滤"""
        if not self.is_connected(): return  # 手柄未连接则直接返回
        pygame.event.pump()  # 处理pygame事件（必须调用以更新手柄状态）
        n_axes = min(8, self.controller.get_numaxes())  # 获取有效轴数量（最多8个）
        # 读取并处理每个轴的数据
        for i in range(n_axes):
            raw = float(self.controller.get_axis(i))  # 读取原始轴值
            ema_prev = self.joy_axes_ema[i]  # 上一次的EMA平滑结果
            # 计算新的EMA平滑值（指数移动平均）
            ema_new = (1.0 - self.axis_ema_alpha) * ema_prev + self.axis_ema_alpha * raw
            self.joy_axes_ema[i] = ema_new  # 更新EMA平滑结果
            # 应用死区：绝对值小于阈值则视为0，否则使用平滑后的值
            self.joy_axes[i] = ema_new if abs(ema_new) > self.deadzone else 0.0

        # 从过滤后的轴数据计算末端执行器本地坐标系下的位移
        x_axis = self.joy_axes[0]  # x轴控制（通常对应左摇杆左右）
        y_axis = self.joy_axes[3]  # y轴控制（通常对应右摇杆上下）
        z_axis = -self.joy_axes[1]  # z轴控制（通常对应左摇杆上下，取反）
        # 计算本地坐标系下的位移增量（乘以灵敏度）
        delta_local = np.array([-x_axis, -y_axis, z_axis]) * self.pos_sensitivity

        # 将本地坐标系位移转换到世界坐标系
        tf_current = arm.fk(qpos); R_ee = tf_current[:3,:3]  # 获取当前末端执行器的旋转矩阵
        delta_world = R_ee @ delta_local  # 本地位移 -> 世界位移（旋转矩阵乘法）
        # 更新位置并限制在范围内（防止超出机械臂工作空间）
        self.x = np.clip(self.x + delta_world[0], self.x_min, self.x_max)
        self.y = np.clip(self.y + delta_world[1], self.y_min, self.y_max)
        self.z = np.clip(self.z + delta_world[2], self.z_min, self.z_max)

        # 处理姿态控制
        hat = self.controller.get_hat(0)  # 获取方向键状态（通常是十字键）
        pitch_axis = -self.joy_axes[2]  # 俯仰角控制（通常对应右摇杆左右，取反）
        d_yaw = hat[0] * self.ori_sensitivity  # 偏航角增量（方向键左右）
        d_pitch = pitch_axis * self.ori_sensitivity  # 俯仰角增量
        d_roll = hat[1] * self.ori_sensitivity  # 滚转角增量（方向键上下）
        # 计算姿态增量的旋转矩阵（先滚转、再俯仰、最后偏航）
        R_inc = (
            self._axis_angle_to_matrix(np.array([1,0,0]), d_roll) @  # 绕x轴旋转（滚转）
            self._axis_angle_to_matrix(np.array([0,1,0]), d_pitch) @  # 绕y轴旋转（俯仰）
            self._axis_angle_to_matrix(np.array([0,0,1]), d_yaw)  # 绕z轴旋转（偏航）
        )
        self.R = self.R @ R_inc  # 更新旋转矩阵（右乘增量矩阵，在当前姿态基础上叠加旋转）


        # ============================
        # ======== 按钮功能扩展 ========
        # ============================
        n_buttons = self.controller.get_numbuttons()
        buttons = [self.controller.get_button(i) for i in range(n_buttons)]

        # --- A键：重启系统，既机械臂回到初始位置 ---
        if buttons[0]:  # A键
            print("🔄 检测到 A 键 —— 正在重新启动程序...")
            time.sleep(0.5)  # 防止重启太快，看不到提示
            os.execl(sys.executable, sys.executable, *sys.argv)

        # --- B键：夹爪闭合 ---
        if buttons[7]:  # RB
            print("🤏 夹爪闭合")
            try:
                dr.set_torque(id_num=7, torque=0.2, param=0.2, mode=1)
            except Exception as e:
                print(f"夹爪闭合失败: {e}")

        # --- X键：夹爪张开 ---
        if buttons[6]:  # LB
            print("🖐 张开夹爪")
            try:
                dr.set_angle(id_num=7, angle=-45, speed=10, param=10, mode=1)
            except Exception as e:
                print(f"夹爪张开失败: {e}")

    def get_pose_target(self):
        """获取目标位姿（位置+RPY角）"""
        roll, pitch, yaw = self._matrix_to_rpy(self.R)  # 将旋转矩阵转换为RPY角
        return self.x, self.y, self.z, roll, pitch, yaw  # 返回位置和姿态

    def cleanup(self): pygame.quit()  # 清理pygame资源


# =============================
# ======== 仿真控制类 =========
# =============================
class RobotController(mujoco_viewer.CustomViewer):
    def __init__(self, scene_path, arm_path, controller, real_arm):
        # 调用父类构造函数，初始化仿真场景视图，设置初始视角参数（距离、方位角、仰角）
        super().__init__(scene_path, distance=1.5, azimuth=135, elevation=-30)
        self.arm = casadi_ik.Kinematics("grasp_point")  # 初始化机械臂运动学求解器（用于逆运动学）
        self.arm.buildFromMJCF(arm_path)                # 从MJCF文件加载机械臂模型
        self.controller = controller                    # 手柄控制器实例（用于获取目标位姿）
        self.last_dof = np.zeros(self.arm.model.nq)     # 记录上一时刻的关节角度（初始化为0）
        self.real_arm = real_arm                        # 实物机械臂实例（用于发送控制指令）
        # 记录上一次发送给实物机械臂的关节角度（度）
        self._last_sent_degs = real_arm.last_sent.copy() if real_arm.last_sent is not None else None
        self._last_send_time = 0.0                      # 记录上一次发送指令的时间（用于控制发送频率）
        # 假设实物初始角度为：
        init_deg = [117.5, 56.35, 115.79, 57.89, -113.5, 0]  # 单位：度
        init_rad = [np.radians(d) for d in init_deg]  # 转成弧度

        # 在 RobotController.__init__ 里：
        self.last_dof[:6] = init_rad
        self.data.qpos[:6] = init_rad


    def runFunc(self):
        """仿真主循环函数：处理输入、计算逆运动学、更新仿真并控制实物机械臂"""
      
        # 1) 处理手柄输入（包含EMA平滑和死区过滤）
        self.controller.handle_input(self.arm, self.last_dof)

        # 2) 获取目标位姿（位置+姿态）
        x, y, z, roll, pitch, yaw = self.controller.get_pose_target()

        # 3) 仅当位姿变化量超过阈值时才尝试求解逆运动学（IK）
        # 计算当前末端执行器位姿与目标位姿的线性和角度差异
        tf_current = self.arm.fk(self.last_dof)            # 通过正运动学获取当前关节对应的末端位姿
        pos_curr = tf_current[:3, 3]                       # 当前位置（从变换矩阵提取）
        rot_curr = tf_current[:3, :3]                      # 当前旋转矩阵（从变换矩阵提取）
        target_tf = self.build_transform(x, y, z, roll, pitch, yaw)  # 构建目标位姿的变换矩阵
        pos_target = target_tf[:3, 3]                      # 目标位置
        trans_mag = np.linalg.norm(pos_target - pos_curr)  # 计算位置变化的欧氏距离（线性变化量）
        
        # 计算角度变化量：使用旋转矩阵之间的轴角表示
        R_target = target_tf[:3, :3]                       # 目标旋转矩阵
        R_delta = R_target @ rot_curr.T                    # 计算旋转差异矩阵（目标相对于当前的旋转）
        # 通过迹计算旋转角度（确保在[-1,1]范围内避免数值问题）
        angle = math.acos(max(-1.0, min(1.0, (np.trace(R_delta) - 1.0) / 2.0)))
        rot_mag = abs(angle)                               # 角度变化量（取绝对值）

        # 4) 使用当前关节角度作为初始值求解IK；若失败则尝试多次扰动初始值重试
        dof_sol, info = self.arm.ik(target_tf, current_arm_motor_q=self.last_dof)  # 求解IK
        # 判断求解是否成功（根据info的返回格式处理）
        success = info.get("success", True) if isinstance(info, dict) else True
        # 若求解失败（无结果或明确返回失败）
        if dof_sol is None or (hasattr(info, "get") and info.get("success") is False):
            found = False  # 标记是否找到有效解
            # 多次尝试扰动初始值重新求解
            for t in range(IK_RETRY_TRIES):
                # 在当前关节角度基础上添加随机扰动作为新的初始值
                seed = self.last_dof + np.random.normal(scale=IK_PERTURB_STD, size=self.last_dof.shape)
                try:
                    dof_try, info_try = self.arm.ik(target_tf, current_arm_motor_q=seed)  # 重试IK
                    if dof_try is not None:  # 找到有效解
                        dof_sol = dof_try
                        found = True
                        break
                except Exception:  # 忽略求解过程中的异常
                    continue
            if not found:  # 多次重试后仍无有效解
                print("[IK] 无可用解，跳过本次更新")
                mujoco.mj_step(self.model, self.data)  # 推进仿真
                time.sleep(0.02)
                return

        # 5) 比较求解结果与当前实物机械臂关节角度（优先使用real_arm.last_sent，否则用仿真的last_dof）
        # 仅当最大角度差超过阈值且满足最小发送间隔时，才发送指令到实物机械臂
        sol_deg = [float(np.degrees(d)) for d in dof_sol[:6]]  # 将求解结果（弧度）转换为度
        
        # 确定当前关节角度（优先用实物机械臂的最后指令，否则用仿真的上一时刻值）
        if self.real_arm.last_sent is not None:
            current_deg = self.real_arm.last_sent
        else:
            current_deg = [float(np.degrees(q)) for q in (self.last_dof[:6])]

        # 计算每个关节的角度差
        diffs = [abs(sol_deg[i] - current_deg[i]) for i in range(6)]
        max_diff = max(diffs)  # 最大角度差
        now = time.time()  # 当前时间

        # 若最大角度差超过阈值且距离上次发送时间超过最小间隔
        if max_diff >= JOINT_SEND_THRESHOLD_DEG and (now - self._last_send_time) >= MIN_SEND_INTERVAL:
            self.real_arm.send_angles_async(sol_deg)  # 异步发送关节角度到实物机械臂
            self._last_send_time = now  # 更新最后发送时间
            # 打印目标位姿（保留3位小数的位置，2位小数的角度）
            print(f"目标位姿: x={x:.3f}, y={y:.3f}, z={z:.3f}, roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")
        
        # 6) 用求解得到的关节角度更新仿真，并推进仿真一步
        self.last_dof = dof_sol  # 更新上一时刻关节角度
        self.data.qpos[:6] = dof_sol[:6]  # 将求解结果设置到仿真模型中
        mujoco.mj_step(self.model, self.data)  # 执行一次仿真步
        time.sleep(0.02)  # 休眠以控制仿真帧率

    def build_transform(self, x, y, z, roll, pitch, yaw):
        """根据位置(x,y,z)和RPY角构建4x4变换矩阵"""
        R = self.controller._rpy_to_matrix(roll, pitch, yaw)  # 将RPY角转换为旋转矩阵
        tf = np.eye(4)  # 初始化4x4单位矩阵
        tf[:3, :3] = R  # 填充旋转部分
        tf[:3, 3] = [x, y, z]  # 填充平移部分
        return tf  # 返回变换矩阵


# 安全初始化检测函数，判断是否到达初始位置
def wait_until_reached(target_deg, tol=1.0, timeout=15):
    """
    等待机械臂到达目标角度（简单版）
    :param target_deg: list[float] 目标角度（单位度）
    :param tol: 允许误差范围（°）
    :param timeout: 超时时间（秒）
    """
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


# =============================
# ========== 主程序 ===========
# =============================
if __name__ == "__main__":
    SCENE_XML_PATH = 'teleoperation_robot/models/robot_arm/scene.xml'
    ARM_XML_PATH = 'teleoperation_robot/models/robot_arm/robot_arm.xml'

    target_deg = [117.5,56.35,115.79,57.89,-113.5,0]
    dr.set_angles(id_list=[1,2,3,4,5,6], angle_list=target_deg, speed=2, param=10, mode=1)

    print("等待机械臂到达初始位置...")
    if not wait_until_reached(target_deg, tol=1.0, timeout=15):
        print("机械臂未到达初始位置，程序退出。")
        exit(1)

    controller = XboxController()
    if not controller.is_connected():
        print("控制器连接失败，程序将退出。")
        exit(1)

    # choose params: for teleop prefer mode=0 (track) with higher speed/param, but you can test mode=1
    real_arm = RealArmInterface(speed=REALARM_SPEED, param=REALARM_PARAM, mode=REALARM_MODE,
                                min_change_deg=REALARM_MIN_CHANGE_DEG, send_hz=REALARM_SEND_HZ,
                                min_send_interval=MIN_SEND_INTERVAL)

    try:
        robot = RobotController(SCENE_XML_PATH, ARM_XML_PATH, controller, real_arm)
        robot.run_loop()
    finally:
        controller.cleanup()
        real_arm.stop()
