#!/usr/bin/env python3

import DrEmpower as dr
import time 

id_num = 3 #关节电机ID号

def motor_control():

    '''设置id号'''
    # dr.set_id(0, id_num)

    '''设置零点位置'''
    # dr.set_zero_position(id_num)

    '''设置临时零点位置'''
    # dr.set_zero_position_temp(id_num)

    '''单个绝对角度控制'''
    # dr.set_angle(id_num=id_num, angle=60, speed=2, param=10, mode=1)

    '''多个绝对角度控制'''
    # dr.set_angles(id_list=[1,2,3,4,5,6], angle_list=[0,0,0,0,0,0], speed=2, param=10, mode=1)

    '''单个关节相对/步进角度控制'''
    # dr.step_angle(id_num=id_num, angle=10, speed=10, param=10, mode=1)

    '''多个关节相对/步进角度控制'''
    # dr.step_angles(id_list=id_list, angle_list=[-20, -20], speed=8, param=8, mode=1)

    '''单个关节自适应绝对角度控制'''
    # dr.set_angle_adaptive(id_num=id_num, angle=180, speed=2, torque=1)

    '''多个关节自适应绝对角度控制'''
    # dr.set_angles_adaptive(id_list=id_list, angle_list=[0, 180], speed_list=[10, 30], torque_list=[3, 2])

    '''单关节阻抗控制'''
    # dr.impedance_control(id_num=id_num, angle=60, speed=2, tff=-2, kp=1, kd=0.5)

    '''多关节阻抗控制'''
    # dr.impedance_control_multi(id_list=id_list, angle_list=[90, 100], speed_list=[2, 1], tff_list=[2, 2], kp_list=[0.1, 0.2], kd_list=[0.1, 0.2], mode=1)

    '''单个关节运动跟随与助力'''
    # dr.motion_aid(id_num=id_num, angle=0, speed=5, angle_err=1, speed_err=1, torque=8)

    '''多个关节运动跟随与助力'''
    # dr.motion_aid_multi(id_list=id_list, angle_list=[60, 60], speed_list=[5, 3], angle_err_list=[1, 1], speed_err_list=[1, 0.1], torque_list=[3, 3])

    '''检查并等待单个关节转动到目标角度'''
    # dr.set_angle(id_num=id_num, angle=90, speed=10, param=10, mode=1)
    # dr.position_done(id_num)
    # print("绝对角度控制运行结束")
    # dr.step_angle(id_num=id_num, angle=-10, speed=10, param=10, mode=1)
    # dr.position_done(id_num)
    # print("相对角度控制运行结束")
    # dr.set_angle_adaptive(id_num=id_num, angle=360, speed=22, torque=2)
    # dr.position_done(id_num)
    # print("自适应绝对角度控制运行结束")
    # dr.impedance_control(id_num=id_num, angle=60, speed=2, tff=1, kp=1, kd=1)
    # dr.position_done(id_num)
    # print("阻抗控制运行结束")
    # dr.motion_follow_aid(id_num=id_num, angle=0, speed=5, angle_err=1, speed_err=1, torque=3)
    # dr.position_done(id_num)
    # print("运动助力控制运行结束")

    '''检查并等待多个关节转动到目标角度'''
    # dr.set_angles(id_list=id_list, angle_list=[180, 180], speed=20, param=10, mode=1)
    # dr.positions_done(id_list=id_list)
    # print("绝对角度控制运行结束")
    # dr.step_angles(id_list=id_list, angle_list=[-20, -20], speed=8, param=8, mode=1)
    # dr.positions_done(id_list=id_list)
    # print("相对角度控制运行结束")
    # dr.set_angles_adaptive(id_list=id_list, angle_list=[0, 180], speed_list=[10, 30], torque_list=[3, 2])
    # dr.positions_done(id_list=id_list)
    # print("自适应绝对角度控制运行结束")
    # dr.impedance_control_multi(id_list=id_list, angle_list=[90, 100], speed_list=[2, 1], tff_list=[2, 2], kp_list=[0.1, 0.2], kd_list=[0.1, 0.2])
    # dr.positions_done(id_list=id_list)
    # print("阻抗控制运行结束")
    # dr.motion_follow_aid_multi(id_list=id_list, angle_list=[60, 0], speed_list=[2, 3], angle_err_list=[1, 1], speed_err_list=[0.1, 0.1], torque_list=[5, 3])
    # dr.positions_done(id_list=id_list)
    # print("运动助力控制运行结束")

    '''多个关节执行规划的轨迹'''
    # id_list = [1, 2, 3, 4, 5, 6]
    # N = len(id_list)
    # t = 5
    # n = 1000
    # start = time.time()
    # while (time.time() - start) < (t / n):
    #     time.sleep(0.0001)
    # bit_wideth1 = 1 / (time.time() - start) / 2  # 计算轨迹跟踪模式下指令发送带宽
    # angles = []
    # angle_speed_torques_list = []
    # for i in range(n):
    #     angle = 90 * math.sin(i / n * math.pi * 2)  # 振幅为 90° 的正弦函数
    #     angle_list = [angle] * N
    #     if i == 0:
    #         print("i = ", i)
    #         dr.set_angles(id_list=id_list, angle_list=angle_list, speed=10, param=10, mode=1)  # 先控制关节已梯形估计模型平缓运动到曲线起点
    #         dr.positions_done(id_list=id_list)  # 等待达到起点
    #         print("i = ", i)
    #     start = time.time()
    #     dr.set_angles(id_list=id_list, angle_list=angle_list, speed=20, param=bit_wideth1, mode=0)
    #     while (time.time() - start) < (t / n):
    #         time.sleep(0.0001)
    #     bit_wideth1 = 1 / (time.time() - start) / 2  # 时刻监控在 t>n * bit_time 情况下单条指令发送的时间
    # dr.clear_uart()

    '''单个关节转速控制'''
    # dr.set_speed(id_num=id_num, speed=10, param=1, mode=0)
    # time.sleep(10)
    # dr.estop(id_num=id_num) # 急停

    '''多个关节转速控制'''
    # dr.set_speeds(id_list=id_list, speed_list=[10, 10], param=2, mode=0)
    # time.sleep(10)
    # dr.estop(id_num=0) # 急停

    '''单个关节力矩控制'''
    # dr.set_torque(id_num=id_num, torque=-0.15, param=0.15, mode=1)
    # time.sleep(1)
    # dr.estop(id_num=id_num) # 急停

    '''多个关节力矩控制'''
    # dr.set_torques(id_list=id_list, torque_list=[10, -10], param=3, mode=0)
    # time.sleep(5)
    # dr.estop(id_num=0) # 急停

    '''回读关节 ID 号 此时总线上只能有 1 个关节'''
    # dr.get_id()

    '''回读总线上多个关节 ID 号'''
    # dr.get_ids()

    '''回读关节当前角度'''
    # angle = dr.get_angle(id_num=id_num)
    # print(angle)

    '''回读关节当前转速'''
    # speed = dr.get_speed(id_num=id_num)
    # print(speed)

    '''同时回读关节当前角度与转速'''
    # angle_speed = dr.get_state(id_num=id_num)
    # print(angle_speed)

    '''回读关节当前输出力矩'''
    # torque = dr.get_torque(id_num=id_num)
    # print(torque)

    '''开启角度、转速、力矩实时反馈'''
    # dr.enable_angle_speed_torque_state(id_num=id_num)

    '''设置角度、转速、力矩状态实时反馈时间间隔，单位 ms，默认为 2'''
    # dr.set_state_feedback_rate_ms(id_num=id_num, n_ms=4)
    #

    '''单个关节角度、转速、力矩实时反馈'''
    # dr.set_state_feedback_rate_ms(id_num=1, n_ms=3) # 设置状态反馈间隔时间 n_ms，注意过小会影响密集指令发送
    # dr.enable_angle_speed_torque_state(id_num=1)
    # N = 1000
    # j = 0
    # for i in range(N):
    #     angle_speed_torque = dr.angle_speed_torque_state(id_num=1, n=1)  # 获取实时角度、转速、力矩信息
    #     if angle_speed_torque is None:
    #         pass
    #     else:
    #         j += 1
    #         print(angle_speed_torque)
    # print(j/N) # 准确率
    # dr.disable_angle_speed_torque_state(id_num=1)

    '''单个关节角度、转速、力矩实时反馈 + 轨迹控制'''
    # start_time = time.time()
    # dr.set_state_feedback_rate_ms(id_num=id_num, n_ms=3) # 设置状态反馈间隔时间 n_ms，注意过小会影响密集指令发送
    # dr.enable_angle_speed_torque_state(id_num)
    # position = []
    # speed = []
    # torque = []
    # while time.time() - start_time < 6: # 实时检测 6 秒
    #     angle_speed_torque = dr.angle_speed_torque_state(id_num, 1)  # 获取实时角度、转速、力矩信息
    #     position.append(angle_speed_torque[0])
    #     print("角度：", position[-1])
    #     speed.append(angle_speed_torque[1])
    #     print("转速：", speed[-1])
    #     torque.append(angle_speed_torque[2])
    #     print("力矩：", torque[-1])
    # print("角度数量：", len(position))
    # print("转速数量：", len(speed))
    # print("力矩数量：", len(torque))


    # dr.angle_speed_torque_state(id_num=id_num, n=6) # 首次运行，由于总线持续高速反馈，有可能截取的总线数据有误
    # i = 0
    # while i < 1000:
    #     angle_speed_torque = dr.angle_speed_torque_state(id_num=id_num, n=6)  # 第二次之后方能得到有效数据
    #     print(angle_speed_torque)
    #     i += 1
    # #### sine 正弦角度曲线轨迹 ####
    # t = 10
    # n = 1000
    # dr.set_state_feedback_rate_ms(id_num=id_num, n_ms=8) # 设置状态反馈间隔时间 n_ms，注意过小会影响密集指令发送
    # dr.disable_angle_speed_torque_state(id_num=id_num) # 先取消状态反馈
    # time.sleep(0.5)
    # start = time.time()
    # while (time.time() - start) < (t / n):
    #     time.sleep(0.0001)
    # bit_wideth1 = 1 / (time.time() - start) / 2  # 计算轨迹跟踪模式下指令发送带宽
    # angle_list = []
    # speed_list = []
    # torque_list = []
    # for i in range(n):
    #     start = time.time()
    #     angle = 90 * math.sin(i/n * math.pi * 2) # 振幅为 90° 的正弦函数
    #     if i == 0:
    #         print("i = ", i)
    #         dr.set_angle(id_num=id_num, angle=angle, speed=10, param=10, mode=1) # 先控制关节已梯形估计模型平缓运动到曲线起点
    #         dr.position_done(id_num=id_num) # 等待达到起点
    #         time.sleep(0.2) # 延时 0.2s 防止信号串扰
    #         dr.enable_angle_speed_torque_state(id_num)
    #         time.sleep(0.2) # 延时 0.2s 防止信号串扰
    #     dr.set_angle(id_num=id_num, angle=angle, speed=20, param=bit_wideth1, mode=0)
    #     while (time.time() - start) < (t / n):
    #         time.sleep(0.0001)
    #     bit_wideth1 = 1 / (time.time() - start) / 2  # 时刻监控在 t>n * bit_time 情况下单条指令发送的时间
    #     angle_speed_torque = dr.angle_speed_torque_state(id_num, 6) # 获取实时角度、转速、力矩信息
    #     print(angle_speed_torque) # 打印实时数据
    #     if angle_speed_torque != None: # 判断数据是否为空，若非空则记录数据（有时数据传输过快或其他干扰会导致信号传输有误）
    #         angle_list.append(angle_speed_torque[0]) # 得到转角数据
    #         speed_list.append(angle_speed_torque[1]) # 得到转速数据
    #         torque_list.append(angle_speed_torque[2]) # 得到力矩数据
    # print(angle_list)
    # import xlwt # 插入 Excel 表格写入模块，需先安装 1.2.0 版本 xlwt 模块
    # book = xlwt.Workbook(encoding='utf-8',style_compression=0) # 创建excel表格类型文件
    # sheet_angle = book.add_sheet('关节角度',cell_overwrite_ok=False) # 创建关节角度表单
    # for i in range(len(angle_list)):
    #     sheet_angle.write(i, 0, angle_list[i]) # 将转角数据写入 Excel 文件
    #
    # sheet_speed = book.add_sheet('关节转速',cell_overwrite_ok=False) # 创建关节转速表单
    # for i in range(len(speed_list)):
    #     sheet_speed.write(i, 0, speed_list[i]) # 将转速数据写入 Excel 文件
    #
    # sheet_torque = book.add_sheet('关节力矩',cell_overwrite_ok=False) # 创建关节力矩表单
    # for i in range(len(torque_list)):
    #     sheet_torque.write(i, 0, torque_list[i]) # 将力矩数据写入 Excel 文件
    #
    # savepath = '角度_转速_力矩.xls' # 保持 Excel 文件至当前目录
    # book.save(savepath)

    '''单个关节轨迹控制 + 状态反馈 + 实时曲线测试'''
    # import matplotlib.pyplot as map # 需要提前安装 matplotlib 库
    #
    # fig = map.figure() # 创建一个图
    #
    # map.rcParams['font.sans-serif'] = ['SimHei']  # 让图表显示中文标签
    # map.rcParams['axes.unicode_minus'] = False
    #
    # angle = fig.add_subplot(3, 1, 1) # 创建3个图，第一个为角度图
    # speed = fig.add_subplot(3, 1, 2) # 转速图
    # torque = fig.add_subplot(3, 1, 3) # 力矩图
    #
    # # 添加 Y 轴标签，rotation 代表文字方向，fontsize 代表字体大小，labelpad 代表文字到坐标轴的距离
    # angle.set_ylabel('角度/°', rotation=90, fontsize=20, labelpad=10)
    # speed.set_ylabel('转速/r/min', rotation=90, fontsize=20, labelpad=10)
    # torque.set_ylabel('力矩/Nm', rotation=90, fontsize=20, labelpad=10)
    #
    # line_angle = None # 角度曲线
    # line_speed = None # 转速曲线
    # line_torque = None # 力矩曲线
    #
    # # X 轴和 Y 轴的参数列表
    # angle_X = [] # 角度图的 X 轴
    # angle_Y = [] # 角度图的 Y 轴
    # speed_X = [] # 转速图的 X 轴
    # speed_Y = [] # 转速图的 Y 轴
    # torque_X = [] # 力矩图的 X 轴
    # torque_Y = [] # 力矩图的 Y 轴
    # Resolution_X = 0.1 # X 轴分度值，即最小刻度，相邻点在 X 轴上的距离
    #
    # dr.disable_angle_speed_torque_state(id_num=id_num) # 先取消试试状态反馈，以免影响后面的普通参数回读指令
    # t = 10 # 曲线运行大概周期
    # i = 0 # 计数初始值
    # n = 200 # 曲线切割的点数
    # start = time.time()
    # while (time.time() - start) < (t / n):
    #     time.sleep(0.0001)
    # bit_wideth1 = 1 / (time.time() - start) / 2  # 计算轨迹跟踪模式下指令发送带宽
    #
    # dr.set_state_feedback_rate_ms(id_num=id_num, n_ms=4) # 设置状态反馈时间间隔，过小可能会影响控制指令发送，需根据实际情况调整
    # r = 0 # 曲线已执行的周期数量
    # while True:
    #     j = 0
    #     for i in range(n):
    #         angle_pos = 180 * math.sin(i / n * math.pi * 2)  # 定义角度曲线，这里为正弦
    #         if i == 0:
    #             dr.set_angle(id_num=id_num, angle=angle_pos, speed=10, param=10, mode=1)  # 先控制关节以梯形转速轨迹模式平缓运动到曲线起点
    #             dr.position_done(id_num=id_num)  # 等待达到起点
    #             dr.enable_angle_speed_torque_state(id_num=id_num)
    #
    #         start = time.time()
    #         dr.set_angle(id_num=id_num, angle=angle_pos, speed=30, param=bit_wideth1, mode=0) # 轨迹控制指令
    #
    #         angle_speed_torque = dr.angle_speed_torque_state(id_num=id_num, n=1) # 使用实时状态反馈函数
    #
    #         angle_X.append((i + r * n) * Resolution_X) # 将角度曲线上的点添加到列表中
    #         angle_Y.append(angle_speed_torque[0])
    #
    #         speed_X.append((i + r * n) * Resolution_X) # 将转速曲线上的点添加到列表中
    #         speed_Y.append(angle_speed_torque[1])
    #
    #         torque_X.append((i + r * n) * Resolution_X) # 将力矩曲线上的点添加到列表中
    #         torque_Y.append(angle_speed_torque[2])
    #
    #         if line_angle is None:  # 如果图还没有画，则创建一个画图
    #             line_angle = angle.plot(angle_X, angle_Y, '-b', marker='.')[0]  # -代表用横线画，b代表线的颜色是蓝色，.代表，画图的坐标点。
    #         # 描角度曲线
    #         line_angle.set_xdata(angle_X)
    #         line_angle.set_ydata(angle_Y)
    #
    #         if line_speed is None:  # 如果图还没有画，则创建一个画图
    #             line_speed = speed.plot(speed_X, speed_Y, '-g', marker='.')[0]  # -代表用横线画，g代表线的颜色是绿色，.代表，画图的关键点，用点代替。
    #         # 这里插入需要画图的参数，由于图线，是由很多个点组成的，所以这里需要的是一个列表
    #         line_speed.set_xdata(speed_X)
    #         line_speed.set_ydata(speed_Y)
    #
    #         if line_torque is None:  # 如果图还没有画，则创建一个画图
    #             line_torque = torque.plot(torque_X, torque_Y, '-r', marker='.')[0]  # -代表用横线画，r代表线的颜色是红色，.代表，画图的关键点，用点代替。
    #         # 这里插入需要画图的参数，由于图线，是由很多个点组成的，所以这里需要的是一个列表
    #         line_torque.set_xdata(torque_X)
    #         line_torque.set_ydata(torque_Y)
    #
    #         # 当 X 轴坐标值更新至 n，则让 X 坐标原点随曲线运动
    #         if len(angle_X) < n:
    #             angle.set_xlim([min(angle_X), max(angle_X) + 10])
    #         else:
    #             angle.set_xlim([angle_X[-n], max(angle_X) + 10]) # X 轴坐标值超出曲线最新 X 轴坐标 10
    #
    #         if len(speed_X) < n:
    #             speed.set_xlim([min(speed_X), max(speed_X) + 10])
    #         else:
    #             speed.set_xlim([speed_X[-n], max(speed_X) + 10])
    #
    #         if len(torque_X) < n:
    #             torque.set_xlim([min(torque_X), max(torque_X) + 10])
    #         else:
    #             torque.set_xlim([torque_X[-n], max(torque_X) + 10])
    #
    #         angle.set_ylim([min(angle_Y) - 10, max(angle_Y) + 10]) # 设置 Y 轴显示范围
    #         speed.set_ylim([min(speed_Y) - 10, max(speed_Y) + 10])
    #         torque.set_ylim([min(torque_Y) - 10, max(torque_Y) + 10])
    #
    #         map.pause(0.01)  # 刷新时间，单位 s
    #
    #         while (time.time() - start) < (t / n):
    #             time.sleep(0.0001)
    #         bit_wideth1 = 1 / (time.time() - start) / 2  # 时刻监控在 t>n * bit_time 情况下单条指令发送的时间
    #         j += 1
    #     r += 1

    '''关闭角度、转速、力矩实时反馈'''
    # dr.disable_angle_speed_torque_state(id_num=id_num)

    '''多个关节角度、转速、力矩实时反馈'''
    # id_list = [1, 2, 3, 4, 5, 6, 7]
    # n = len(id_list)
    # for i in range(n):
    #     dr.set_state_feedback_rate_ms(id_num=id_list[i], n_ms=n*2)  # 设置状态反馈间隔时间 n_ms，注意过小会影响密集指令发送
    # dr.enable_angle_speed_torque_state(id_num=0) # 全部开启状态反馈
    # dr.disable_angle_speed_torque_state(id_num=8) # 关闭不需要反馈的关节
    # N = 1000
    # j = 0
    # for i in range(N):
    #     angle_speed_torques = dr.angle_speed_torque_states(id_list=id_list)  # 获取实时角度、转速、力矩信息
    #     if angle_speed_torques is None:
    #         print(1)
    #         pass
    #     else:
    #         j += 1
    #         print(angle_speed_torques)
    # print(j/N) # 准确率
    # dr.disable_angle_speed_torque_state(id_num=0)

    '''同时读取总线上所有一体化关节当前角度、转速和力矩'''
    # j = 0
    # N = 1000
    # for i in range(N):
    #     angle_speed_torque_list = dr.get_angle_speed_torque_all([1, 2, 3, 4, 5, 6, 7, 8])
    #     if angle_speed_torque_list is None:
    #         pass
    #     else:
    #         print(angle_speed_torque_list)
    #         j+=1
    # print(j/N) # 准确率

    '''回读控制环位置增益 P、积分增益 I函数、转速增益 D'''
    # dr.get_pid(id_num=id_num)

    '''回读配置参数'''
    # print("总线电压为：", dr.read_property(id_num=id_num, property='dr.voltage'))
    # print("总线电流为：", dr.read_property(id_num=id_num, property='dr.i'))
    # print("CAN 通信波特率为：", dr.read_property(id_num=id_num, property='dr.can.config.baud_rate'))
    # print("状态反馈使能状态为：", dr.read_property(id_num=id_num, property='dr.can.enable_state_feedback'))
    # print("当前控制状态为：", dr.read_property(id_num=id_num, property='dr.current_state'))
    # print("CAN ID 号为：", dr.read_property(id_num=id_num, property='dr.config.can_id'))
    # print("状态反馈时间间隔为：", dr.read_property(id_num=id_num, property='dr.config.state_feedback_rate_ms'))
    # print("角度限制属性使能状态为：", dr.read_property(id_num=id_num, property='dr.config.enable_angle_limit'))
    # print("最小角度限制属性为：", dr.read_property(id_num=id_num, property='dr.config.angle_min'))
    # print("最大角度限制属性为：", dr.read_property(id_num=id_num, property='dr.config.angle_max'))
    # print("减速比为：", dr.read_property(id_num=id_num, property='dr.config.gear_ratio'))
    # print("堵转保护电流为：", dr.read_property(id_num=id_num, property='dr.config.stall_current_limit'))
    # print("碰撞检测使能状态为：", dr.read_property(id_num=id_num, property='dr.config.enable_crash_detect'))
    # print("碰撞检测灵敏度为：", dr.read_property(id_num=id_num, property='dr.config.crash_detect_sensitivity'))
    # print("转速限制使能状态为：", dr.read_property(id_num=id_num, property='dr.controller.config.enable_speed_limit'))
    # print("位置增益为：", dr.read_property(id_num=id_num, property='dr.controller.config.angle_gain'))
    # print("转速增益为：", dr.read_property(id_num=id_num, property='dr.controller.config.speed_gain'))
    # print("积分增益为：", dr.read_property(id_num=id_num, property='dr.controller.config.speed_integrator_gain'))
    # print("最大转速限制为：", dr.read_property(id_num=id_num, property='dr.controller.config.speed_limit'))
    # print("超速容忍度为：", dr.read_property(id_num=id_num, property='dr.controller.config.speed_limit_tolerance'))
    # print("转动惯量为：", dr.read_property(id_num=id_num, property='dr.controller.config.inertia'))
    # print("位置输入带宽为：", dr.read_property(id_num=id_num, property='dr.controller.config.input_filter_bandwidth'))
    # print("电机极对数为：", dr.read_property(id_num=id_num, property='dr.motor.config.pole_pairs'))
    # print("电机相电感为：", dr.read_property(id_num=id_num, property='dr.motor.config.phase_inductance'))
    # print("电机相电阻为：", dr.read_property(id_num=id_num, property='dr.motor.config.phase_resistance'))
    # print("电机力矩常数为：", dr.read_property(id_num=id_num, property='dr.motor.config.torque_constant'))
    # print("最大电流限制为：", dr.read_property(id_num=id_num, property='dr.motor.config.current_limit'))
    # print("过流容忍度为：", dr.read_property(id_num=id_num, property='dr.motor.config.current_limit_margin'))
    # print("最大力矩限制为：", dr.read_property(id_num=id_num, property='dr.motor.config.torque_limit'))
    # print("电流控制带宽为：", dr.read_property(id_num=id_num, property='dr.motor.config.current_control_bandwidth'))
    # print("q 轴电流为：", dr.read_property(id_num=id_num, property='dr.motor.Iq_measured'))
    # print("d 轴电流为：", dr.read_property(id_num=id_num, property='dr.motor.Id_measured'))
    # print("电路板温度为：", dr.read_property(id_num=id_num, property='dr.board_temperature'))
    # print("电路板温度保护使能状态为：", dr.read_property(id_num=id_num, property='dr.board_temperature.config.enabled'))
    # print("电路板温度保护下限为：", dr.read_property(id_num=id_num, property='dr.board_temperature.config.temp_limit_lower'))
    # print("电路板温度保护上限为：", dr.read_property(id_num=id_num, property='dr.board_temperature.config.temp_limit_upper'))
    # print("电机温度为：", dr.read_property(id_num=id_num, property='dr.motor_temperature'))
    # print("电机温度保护使能状态为：", dr.read_property(id_num=id_num, property='dr.motor_temperature.config.enabled'))
    # print("电机板温度保护下限为：", dr.read_property(id_num=id_num, property='dr.motor_temperature.config.temp_limit_lower'))
    # print("电机板温度保护上限为：", dr.read_property(id_num=id_num, property='dr.motor_temperature.config.temp_limit_upper'))
    # print("角度为：", dr.read_property(id_num=id_num, property='dr.output_shaft.angle'))
    # print("转速为：", dr.read_property(id_num=id_num, property='dr.output_shaft.speed'))
    # print("力矩为：", dr.read_property(id_num=id_num, property='dr.output_shaft.torque'))
    # print("临时最小角度限制为：", dr.read_property(id_num=id_num, property='dr.output_shaft.angle_min'))
    # print("临时最大角度限制为：", dr.read_property(id_num=id_num, property='dr.output_shaft.angle_max'))
    # print("临时角度限制使能状态为：", dr.read_property(id_num=id_num, property='dr.output_shaft.enable_angle_limit'))

    '''设置本次运行期间关节角度限位'''
    # dr.set_angle_range(id_num=id_num, angle_min=-181, angle_max=181)
    # dr.set_angle(id_num=id_num, angle=-200, speed=10, param=10, mode=1)
    # dr.position_done(id_num=id_num)
    # print(dr.get_angle(id_num=id_num))

    '''关闭本次运行期间的关节角度限位'''
    # dr.disable_angle_range(id_num=id_num)
    # dr.set_angle(id_num=id_num, angle=-200, speed=10, param=10, mode=1)
    # dr.position_done(id_num=id_num)
    # print(dr.get_angle(id_num=id_num))

    '''设置关节限位属性'''
    # dr.set_angle_range_config(id_num=id_num, angle_min=-181, angle_max=181)
    # dr.set_angle(id_num=id_num, angle=200, speed=10, param=10, mode=1)
    # dr.position_done(id_num=id_num)
    # print(dr.get_angle(id_num=id_num))

    '''关闭关节限位属性'''
    # dr.disable_angle_range_config(id_num=id_num)
    # dr.set_angle(id_num=id_num, angle=0, speed=10, param=10, mode=1)
    # dr.position_done(id_num=id_num)
    # print(dr.get_angle(id_num=id_num))

    '''设置本次运行期间转速限制'''
    # dr.set_angle(id_num=id_num, angle=0, speed=20, param=10, mode=1)
    # dr.set_speed_limit(id_num=id_num, speed_limit=1) # 运行在运动控制指令之后

    '''设置本次运行期间力矩限制'''
    # dr.set_angle(id_num=id_num, angle=-600, speed=10, param=10, mode=1)
    # dr.set_torque_limit(id_num=id_num, torque_limit=2) # 运行在运动控制指令之后

    '''设置本次运行期间自适应转速限制'''
    # dr.set_angle(id_num=id_num, angle=-600, speed=10, param=10, mode=1)
    # dr.set_speed_adaptive(id_num=0, speed_adaptive=15)

    '''设置本次运行期间自适应力矩限制'''
    # dr.set_angle(id_num=id_num, angle=600, speed=10, param=10, mode=1)
    # dr.set_torque_adaptive(id_num=0, torque_adaptive=8)

    '''设置本次运行期间位置增益 P、积分增益 I、转速增益 D'''
    # dr.set_pid(id_num=id_num, P=25, I=20, D=20)

    '''设置关节待机或闭环控制'''
    # dr.set_mode(id_num=id_num, mode=1)

    '''写入关节配置参数'''
    # dr.write_property(id_num=id_num, property='dr.config.state_feedback_rate_ms', value=2000) # 设置角度、转速、力矩实时反馈时间间隔，单位 ms
    # dr.enable_angle_speed_torque_state(id_num=id_num)
    # i = 0
    # while i < 10:
    #     angle_speed_torque = dr.angle_speed_torque_state(id_num=id_num, n=1)
    #     print(angle_speed_torque)
    #     i += 1
    # dr.disable_angle_speed_torque_state(id_num=id_num)

    '''保存关节配置参数'''
    # dr.save_config(id_num=id_num)

    '''打印关节错误'''
    # dr.dump_error(id_num=id_num)

    '''重启一体化关节'''
    # dr.reboot(id_num=id_num)

    '''恢复出厂设置，不改变关节 ID 号'''
    # dr.init_config(id_num=id_num)

    '''擦除配置参数，重新标定并恢复出厂设置，关节 ID 变为 0'''
    # dr.erase_config(id_num=id_num)

if __name__ == '__main__':
    try:
        motor_control()
    except Exception as e:  # 捕获可能的异常
        print(f"发生错误: {e}")

