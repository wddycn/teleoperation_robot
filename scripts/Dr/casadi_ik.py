# 该代码基于以下项目扩展:
# https://github.com/unitreerobotics/xr_teleoperate/blob/main/teleop/robot_control/robot_arm_ik.py
# https://github.com/ccrpRepo/mocap_retarget/blob/master/src/mocap/src/robot_ik.py

import casadi          # 用于符号计算和优化问题求解的库
import numpy as np      # 用于数值计算和矩阵操作
import pinocchio as pin  # 用于机器人运动学/动力学计算的库
from pinocchio import casadi as cpin  # Pinocchio的Casadi接口，用于符号化运动学计算               

class Kinematics:
    def __init__(self, ee_frame) -> None:  # 初始化函数，ee_frame为末端执行器坐标系名称
        self.frame_name = ee_frame  # 存储末端执行器坐标系名称

    def buildFromMJCF(self, mcjf_file):  # 从MJCF格式文件（MuJoCo模型）构建机器人模型
        self.arm = pin.RobotWrapper.BuildFromMJCF(mcjf_file)  # 使用Pinocchio加载MJCF模型
        self.createSolver()  # 初始化逆运动学求解器

    def buildFromURDF(self, urdf_file):  # 从URDF格式文件构建机器人模型
        self.arm = pin.RobotWrapper.BuildFromURDF(urdf_file)  # 使用Pinocchio加载URDF模型
        self.createSolver()  # 初始化逆运动学求解器

    def createSolver(self):  # 创建逆运动学求解器（核心方法）
        self.model = self.arm.model  # 获取机器人模型的运动学参数（关节、连杆等）
        self.data = self.arm.data  # 获取存储机器人状态的数据结构（位置、速度等）

        # 创建用于符号计算的Casadi模型和数据
        self.cmodel = cpin.Model(self.model)  # 将Pinocchio模型转换为Casadi符号模型
        self.cdata = self.cmodel.createData()  # 创建符号模型对应的数据结构

        # 创建符号变量
        self.cq = casadi.SX.sym("q", self.model.nq, 1)  # 关节角度符号变量（nq为关节数量）
        self.cTf = casadi.SX.sym("tf", 4, 4)  # 目标位姿符号变量（4x4齐次变换矩阵）
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)  # 符号化正向运动学计算
        
        # 获取末端执行器坐标系ID并定义误差函数
        self.ee_id = self.model.getFrameId(self.frame_name)  # 根据名称获取末端执行器坐标系ID

        # 定义位置误差函数：末端执行器当前位置与目标位置的差
        self.translational_error = casadi.Function(
            "translational_error",  # 函数名称
            [self.cq, self.cTf],  # 输入：关节角度、目标位姿
            [
                casadi.vertcat(  # 输出：将3个位置误差合并为向量
                    self.cdata.oMf[self.ee_id].translation - self.cTf[:3,3]  # 位置误差 = 当前位置 - 目标位置（齐次矩阵最后一列）
                )
            ],
        )
        # 定义姿态误差函数：末端执行器当前姿态与目标姿态的差（使用旋转矩阵的对数映射）
        self.rotational_error = casadi.Function(
            "rotational_error",  # 函数名称
            [self.cq, self.cTf],  # 输入：关节角度、目标位姿
            [
                casadi.vertcat(  # 输出：将3个姿态误差合并为向量
                    # 旋转误差 = 对数映射(当前旋转矩阵 @ 目标旋转矩阵的转置)
                    cpin.log3(self.cdata.oMf[self.ee_id].rotation @ self.cTf[:3,:3].T)
                )
            ],
        )

        # 定义优化问题
        self.opti = casadi.Opti()  # 创建Casadi优化问题实例
        self.var_q = self.opti.variable(self.model.nq)  # 优化变量：关节角度（待求解）
        self.var_q_last = self.opti.parameter(self.model.nq)  # 上一时刻关节角度（用于平滑约束）
        self.param_tf = self.opti.parameter(4, 4)  # 优化参数：目标位姿（外部输入）
        # 计算各项代价函数
        self.translational_cost = casadi.sumsqr(self.translational_error(self.var_q, self.param_tf))  # 位置误差平方和
        self.rotation_cost = casadi.sumsqr(self.rotational_error(self.var_q, self.param_tf))  # 姿态误差平方和
        self.regularization_cost = casadi.sumsqr(self.var_q)  # 关节角度正则化（避免过大角度）
        self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)  # 平滑代价（与上一时刻角度差的平方和）

        # 设置优化约束和目标
        self.opti.subject_to(self.opti.bounded(  # 关节角度范围约束（限制在模型定义的上下限内）
            self.model.lowerPositionLimit,  # 关节角度下限
            self.var_q,  # 优化变量
            self.model.upperPositionLimit)  # 关节角度上限
        )
        # 总代价函数（加权求和）：位置精度权重最高，其次是平滑性，姿态精度权重较低
        self.opti.minimize(10.0 * self.translational_cost + 0.5*self.rotation_cost + 0.001 * self.regularization_cost + 0.1 * self.smooth_cost)

        ##### 配置IPOPT求解器 #####
        opts = {
            'ipopt':{
                'print_level': 0,  # 不打印求解过程
                'max_iter': 500,  # 最大迭代次数
                'tol': 1e-4,  # 收敛精度
                # 'hessian_approximation':"limited-memory"  # 可选：使用有限内存近似Hessian矩阵
            },
            'print_time':False,  # 不打印求解时间
            'calc_lam_p':False  # 避免NaN问题（参考Casadi FAQ）
        }
        self.opti.solver("ipopt", opts)  # 设置求解器为IPOPT并应用配置

        self.init_data = np.zeros(self.model.nq)  # 初始化关节角度（默认全为0）
      
    def ik(self, T , current_arm_motor_q = None, current_arm_motor_dq = None):  # 逆运动学求解函数，T为目标位姿
        if current_arm_motor_q is not None:  # 如果提供当前关节角度，用它作为初始值（加速收敛）
            self.init_data = current_arm_motor_q
        self.opti.set_initial(self.var_q, self.init_data)  # 设置优化变量的初始猜测值

        self.opti.set_value(self.param_tf, T)  # 设置目标位姿参数
        self.opti.set_value(self.var_q_last, self.init_data)  # 设置上一时刻关节角度（用于平滑约束）

        try:
            sol = self.opti.solve()  # 完整求解（包含更多检查）
            # sol = self.opti.solve_limited()  # 有限求解（更快，适合实时场景）

            sol_q = self.opti.value(self.var_q)  # 获取求解得到的关节角度
            # self.smooth_filter.add_data(sol_q)  # 平滑滤波（注释掉的备用代码）
            # sol_q = self.smooth_filter.filtered_data

            # 计算关节速度（此处设为0，可根据实际需求修改）
            if current_arm_motor_dq is not None:
                v = current_arm_motor_dq * 0.0  # 若提供当前速度，基于此计算
            else:
                v = (sol_q - self.init_data) * 0.0  # 否则基于角度差近似（此处禁用）

            self.init_data = sol_q  # 更新初始值为当前求解结果（用于下次迭代）

            # 计算逆动力学（关节力矩）：使用RNEA（递归牛顿-欧拉算法）
            sol_tauff = pin.rnea(self.model, self.data, sol_q, v, np.zeros(self.model.nv))
            # 补全力矩数组（确保长度与关节数量一致）
            sol_tauff = np.concatenate([sol_tauff, np.zeros(self.model.nq - sol_tauff.shape[0])], axis=0)
            
            info = {"sol_tauff": sol_tauff, "success": True}  # 求解成功的信息

            dof = np.zeros(self.model.nq)  # 初始化关节角度数组
            dof[:len(sol_q)] = sol_q  # 填充求解得到的关节角度
            return dof, info  # 返回关节角度和求解信息
        
        except Exception as e:  # 捕获求解失败的异常
            print(f"ERROR in convergence, plotting debug info.{e}")  # 打印错误信息

            sol_q = self.opti.debug.value(self.var_q)  # 获取调试用的关节角度（可能未完全收敛）
            # self.smooth_filter.add_data(sol_q)  # 平滑滤波（注释掉的备用代码）
            # sol_q = self.smooth_filter.filtered_data

            # 同上方，计算关节速度（此处设为0）
            if current_arm_motor_dq is not None:
                v = current_arm_motor_dq * 0.0
            else:
                v = (sol_q - self.init_data) * 0.0

            self.init_data = sol_q  # 更新初始值

            # 计算逆动力学（关节力矩）
            sol_tauff = pin.rnea(self.model, self.data, sol_q, v, np.zeros(self.model.nv))
            import ipdb; ipdb.set_trace()  # 触发调试器，用于排查问题
            # 补全力矩数组
            sol_tauff = np.concatenate([sol_tauff, np.zeros(self.model.nq - sol_tauff.shape[0])], axis=0)

            # 打印调试信息
            print(f"sol_q:{sol_q} \nmotorstate: \n{current_arm_motor_q} \nright_pose: \n{T}")

            info = {"sol_tauff": sol_tauff * 0.0, "success": False}  # 求解失败的信息（力矩设为0）

            dof = np.zeros(self.model.nq)  # 初始化关节角度数组
            # dof[:len(sol_q)] = current_arm_motor_q  # 可选：使用当前关节角度（注释掉）
            dof[:len(sol_q)] = self.init_data  # 使用调试得到的角度
            
            raise e  # 抛出异常，通知上层调用者
    def fk(self, q):
        """根据关节角计算末端齐次变换矩阵"""
        import pinocchio as pin
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        oMf = self.data.oMf[self.ee_id]  # 末端frame的位姿
        tf = np.eye(4)
        tf[:3, :3] = oMf.rotation
        tf[:3, 3] = oMf.translation
        return tf


if __name__ == "__main__":  # 主函数：测试逆运动学求解器
    
    arm = Kinematics("Jaw")  # 创建机械臂1的运动学实例，末端执行器为"Jaw"
    arm.buildFromMJCF("/home/ycn/mujoco/my_code/models/trs_so_arm100/so_arm100.xml")  # 从MJCF文件加载模型
    theta = np.pi  # 旋转角度（180度）
    # 定义目标位姿（4x4齐次矩阵）：沿x=0.1, y=0.2, z=0.3，绕x轴旋转180度
    tf = np.array([
            [1, 0, 0, 0.1],
            [0, np.cos(theta), -np.sin(theta), 0.2],
            [0, np.sin(theta), np.cos(theta), 0.3],
        ])
    tf = np.vstack((tf, [0, 0, 0, 1]))  # 补全为4x4齐次矩阵
    dof, info = arm.ik(tf)  # 求解逆运动学
    print(f"DoF: {dof}, Info: {info}")  # 打印结果

    arm2 = Kinematics("joint7")  # 创建机械臂2的运动学实例，末端执行器为"joint7"
    arm2.buildFromMJCF("/home/ycn/mujoco/my_code/models/franka_emika_panda/panda_nohand.xml")  # 加载Franka Panda模型
    theta = np.pi  # 旋转角度（180度）
    # 定义相同的目标位姿
    tf = np.array([
            [1, 0, 0, 0.1],
            [0, np.cos(theta), -np.sin(theta), 0.2],
            [0, np.sin(theta), np.cos(theta), 0.3],
        ])
    tf = np.vstack((tf, [0, 0, 0, 1]))  # 补全为4x4齐次矩阵
    dof, info = arm2.ik(tf)  # 求解逆运动学
    print(f"DoF: {dof}, Info: {info}")  # 打印结果
