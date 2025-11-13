# -*- coding=utf-8 -*-
# !/usr/bin/env python3
N = 1

def plus_plus(n=0):
    global N
    if n == 0:
        N = 1
    else:
        N = N + 1
    return N

property_addresss = {
    #     #
    'dr.voltage': 00000 + plus_plus(0), # 只读
    'dr.i': 00000 + plus_plus(1), # 只读
    # config. #
    # can. #
    'can.error': 20000 + plus_plus(0), # 只读
    # can.config. #
    'dr.can.config.baud_rate': 21000 + plus_plus(0), # CAN波特率
    # can. #
    'dr.can.enable_state_feedback': 22000 + plus_plus(0), # 是否开启CAN实时状态反馈
    # dr. #
    'dr.error': 30000 + plus_plus(0), # 只读
    'dr.current_state': 30000 + plus_plus(1), # 只读
    'dr.requested_state': 30000 + plus_plus(1), # 设置运行状态
    'dr.version_date': 30000 + plus_plus(1), # 版本日期为 231207 及以后版本可用
    # dr.config. #
    'dr.config.can_id': 31000 + plus_plus(0), # CAN总线节点 ID 号
    'dr.config.state_feedback_rate_ms': 31000 + plus_plus(1), # CAN总线实时状态反馈时间间隔，默认为 2，单位 ms，当总线中不同 ID 号关节数量为 n 时，需将该值设置为 2n
    'dr.config.product_model': 31000 + plus_plus(1), # 产品型号（只读）
    # dr.config. #
    'dr.config.enable_angle_limit': 31200 + plus_plus(0), # 是否开启角度限制属性，默认不开启
    'dr.config.angle_min': 31200 + plus_plus(1), # 最小角度限位 默认为-180°
    'dr.config.angle_max': 31200 + plus_plus(1),# 最大角度限位 默认为180°
    'dr.config.gear_ratio': 31200 + plus_plus(1), # 只读 减速比
    'dr.config.stall_current_limit': 31200 + plus_plus(1), # 堵转电流
    'dr.config.enable_crash_detect': 31200 + plus_plus(1), # 是否开启碰撞检测，默认开启
    'dr.config.crash_detect_sensitivity': 31200 + plus_plus(1), # 碰撞检测灵敏度，该值需大于 0，越小越灵敏
    'dr.config.enable_encoder_circular_limit': 31200 + plus_plus(1), # 是否开启多圈计数角度限制，超出该限制关节将无法记住重启前的角度，若关闭该限制则务必将编码器供电线拔掉，即默认系统不需要关机后零点位置
    # dr.controller. #
    'dr.controller.error': 32000 + plus_plus(0), # 只读
    'dr.controller.position_done': 32000 + plus_plus(1), # 只读
    'dr.controller.position_precision': 32000 + plus_plus(1), # 判定运动到位的定位精度，版本号为 231207 及以后版本可用
    # dr.controller.config. #
    'dr.controller.config.enable_speed_limit': 32100 + plus_plus(0),# 是否启用转速限制，默认启用
    'dr.controller.config.angle_gain': 32100 + plus_plus(1), # 位置增益
    'dr.controller.config.speed_gain': 32100 + plus_plus(1), # 转速增益
    'dr.controller.config.speed_integrator_gain': 32100 + plus_plus(1), # 转速积分增益
    'dr.controller.config.speed_limit': 32100 + plus_plus(1), # 最大限制转速，此处转速为电机转速，输出端需要考虑减速比
    'dr.controller.config.speed_limit_tolerance': 32100 + plus_plus(1), # 超出最大转速的忍耐度，比如设定为 1.2 时表示当转速超过设定的最大转速值的 1.2 倍才会触发 超速错误
    'dr.controller.config.inertia': 32100 + plus_plus(1), # 负载转动惯量 默认为0
    'dr.controller.config.input_filter_bandwidth': 32100 + plus_plus(1), # 输入滤波带宽，反应外部控制信号输入的快慢
    # dr.motor. #
    'dr.motor.error': 33000 + plus_plus(0), # 只读
    # dr.motor.config. #
    'dr.motor.config.pole_pairs': 33100 + plus_plus(0), # 只读
    'dr.motor.config.phase_inductance': 33100 + plus_plus(1), # 只读
    'dr.motor.config.phase_resistance': 33100 + plus_plus(1), # 只读
    'dr.motor.config.torque_constant': 33100 + plus_plus(1), # 只读
    'dr.motor.config.current_limit': 33100 + plus_plus(1), # 最大电流限制
    'dr.motor.config.current_limit_margin': 33100 + plus_plus(1), # 最大电流限制忍耐度，如：此值设置为 3 表示当关节电流超过限制电流 3A 时停止关节并报错
    'dr.motor.config.torque_limit': 33100 + plus_plus(1), # 最大力矩限制，该限制值对象是电机，输出端需乘减速比
    'dr.motor.config.current_control_bandwidth': 33100 + plus_plus(1), # 电流控制带宽
    # dr.motor.current_control. #
    'dr.motor.Iq_measured': 33200 + plus_plus(0), # 只读 FOC Q轴电流
    'dr.motor.Id_measured': 33200 + plus_plus(1), # 只读 FOC D轴电流
    # dr.encoder. #
    'dr.encoder.error': 34000 + plus_plus(0), # 只读
    'dr.encoder.abs_output': 34000 + plus_plus(1), # 只读
    'dr.encoder.config.pos_zero': 34000 + plus_plus(1),
    'dr.encoder.abs_angle_power_on': 34000 + plus_plus(1),
    'dr.encoder.abs_turns_power_on': 34000 + plus_plus(1),
    'dr.encoder.sign_turns': 34000 + plus_plus(1),

    'dr.encoder.pos_zero_output': 34100 + plus_plus(0), # 只读 设置零点后输出轴编码器零位值（中空关节专属）
    'dr.encoder.abs_pos_power_on': 34100 + plus_plus(1), # 只读 刚启动时读取到的输出轴编码器值（中空关节专属）
    'dr.encoder.first_half_T': 34100 + plus_plus(1), # 中空关节绝对位置误差的前半个周期长度（中空关节专属）
    'dr.encoder.first_half_M': 34100 + plus_plus(1), # 中空关节绝对位置误差的前半个周期幅值（中空关节专属）
    'dr.encoder.second_half_M': 34100 + plus_plus(1), # 中空关节绝对位置误差的后半个周期幅值（中空关节专属）
    'dr.encoder.intercept': 34100 + plus_plus(1), # 中空关节绝对位置误差截距（中空关节专属）
    'dr.encoder.first_half_up': 34100 + plus_plus(1), # 中空关节绝对位置误差的前半个周期幅值是否向上，是则为 1 否则为 0（中空关节专属）
    # dr.board_temperature. #
    'dr.board_temperature.error': 36000 + plus_plus(0), # 只读
    'dr.board_temperature': 36000 + plus_plus(1), #只读
    # dr.board_temperature.config. #
    'dr.board_temperature.config.enabled': 36100 + plus_plus(0), # 是否开启驱动板温度保护，默认开启
    'dr.board_temperature.config.temp_limit_lower': 36100 + plus_plus(1), # 驱动板温度保护下限，即开始进行保护的温度
    'dr.board_temperature.config.temp_limit_upper': 36100 + plus_plus(1), # 驱动板温度保护上限，即完全保护待机的温度
    # dr.motor_temperature. #
    'dr.motor_temperature.error': 37000 + plus_plus(0), # 只读
    'dr.motor_temperature': 37000 + plus_plus(1), #只读
    # dr.motor_temperature.config. #
    'dr.motor_temperature.config.enabled': 37100 + plus_plus(0), # 是否开启电机温度保护，默认开启
    'dr.motor_temperature.config.temp_limit_lower': 37100 + plus_plus(1), # 电机温度保护下限，即开始进行保护的温度，一般不宜超过80℃
    'dr.motor_temperature.config.temp_limit_upper': 37100 + plus_plus(1), # 电机温度保护上限，即完全保护待机的温度
    # dr.output_shaft. #
    'dr.output_shaft.angle': 38000 + plus_plus(0), # 只读
    'dr.output_shaft.speed': 38000 + plus_plus(1), # 只读
    'dr.output_shaft.torque': 38000 + plus_plus(1), # 只读
    'dr.output_shaft.angle_min': 38000 + plus_plus(1), # 输出轴最小临时角度限位，默认为-180°
    'dr.output_shaft.angle_max': 38000 + plus_plus(1), # 输出轴最大临时角度限位，默认为180°
    'dr.output_shaft.enable_angle_limit': 38000 + plus_plus(1), # 是否开启输出端临时角度限位，默认不开启
}

property_type = {
    #     #
    'dr.voltage': 'f', # 只读
    'dr.i': 'f', # 只读
    # can. #
    'can.error': 'u32',
    # can.config. #
    'dr.can.config.baud_rate': 'u32',
    # can. #
    'dr.can.enable_state_feedback': 'u32',
    # dr. #
    'dr.error': 'u32',
    'dr.current_state': 'u32',
    'dr.requested_state': 'u32',
    'dr.version_date': 'u32', # 版本日期
    # dr.config. #
    'dr.config.can_id': 'u32',
    'dr.config.state_feedback_rate_ms': 'u32',
    'dr.config.product_model': 'u32',
    'dr.config.enable_angle_limit': 'u32',
    'dr.config.angle_min': 'f',
    'dr.config.angle_max': 'f',
    'dr.config.gear_ratio': 'f',
    'dr.config.stall_current_limit': 'f',
    'dr.config.enable_crash_detect': 'u32',
    'dr.config.crash_detect_sensitivity': 'f',
    'dr.config.enable_encoder_circular_limit': 'u32',
    # dr.controller. #
    'dr.controller.error': 'u32',
    'dr.controller.position_done': 'u32',
    'dr.controller.position_precision': 'f', # 判定运动到位的定位精度
    # dr.controller.config. #
    'dr.controller.config.enable_gain_scheduling': 'u32',
    'dr.controller.config.enable_speed_limit': 'u32',
    'dr.controller.config.angle_gain': 'f',
    'dr.controller.config.speed_gain': 'f',
    'dr.controller.config.speed_integrator_gain': 'f',
    'dr.controller.config.speed_limit': 'f',
    'dr.controller.config.speed_limit_tolerance': 'f',
    'dr.controller.config.inertia': 'f',
    'dr.controller.config.input_filter_bandwidth': 'f',
    # dr.motor. #
    'dr.motor.error': 'u32',
    # dr.motor.config. #
    'dr.motor.config.pole_pairs': 's32',
    'dr.motor.config.phase_inductance': 'f',
    'dr.motor.config.phase_resistance': 'f',
    'dr.motor.config.torque_constant': 'f',
    'dr.motor.config.current_limit': 'f',
    'dr.motor.config.current_limit_margin': 'f',
    'dr.motor.config.torque_limit': 'f',
    'dr.motor.config.current_control_bandwidth': 'f',
    # dr.motor.current_control. #
    'dr.motor.Iq_measured': 'f',
    'dr.motor.Id_measured': 'f',
    # dr.encoder. #
    'dr.encoder.error': 'u32',
    'dr.encoder.abs_output': 's32',
    'dr.encoder.config.pos_zero': 'u32',
    'dr.encoder.abs_angle_power_on': 'u32',
    'dr.encoder.abs_turns_power_on': 's32',
    'dr.encoder.sign_turns': 'u32',

    'dr.encoder.pos_zero_output': 's32',
    'dr.encoder.abs_pos_power_on': 's32',
    'dr.encoder.first_half_T': 'f',
    'dr.encoder.first_half_M': 'f',
    'dr.encoder.second_half_M': 'f',
    'dr.encoder.intercept': 'f',
    'dr.encoder.first_half_up': 'u32',
    # dr.board_temperature. #
    'dr.board_temperature.error': 'u32',
    'dr.board_temperature': 'f',
    # dr.board_temperature.config. #
    'dr.board_temperature.config.enabled': 'u32',
    'dr.board_temperature.config.temp_limit_lower': 'f',
    'dr.board_temperature.config.temp_limit_upper': 'f',
    # dr.motor_temperature. #
    'dr.motor_temperature.error': 'u32',
    'dr.motor_temperature': 'f',
    # dr.motor_temperature.config. #
    'dr.motor_temperature.config.enabled': 'u32',
    'dr.motor_temperature.config.temp_limit_lower': 'f',
    'dr.motor_temperature.config.temp_limit_upper': 'f',
    # dr.output_shaft. #
    'dr.output_shaft.angle': 'f',
    'dr.output_shaft.speed': 'f',
    'dr.output_shaft.torque': 'f',
    'dr.output_shaft.angle_min': 'f',
    'dr.output_shaft.angle_max': 'f',
    'dr.output_shaft.enable_angle_limit': 'u32',
}


# 通过属性编码找到名称
def value_find_key(value=1):
    dict_temp = property_addresss
    if value in dict_temp.values():
        return list(dict_temp.keys())[list(dict_temp.values()).index(value)]
    else:
        return 0


# 通过名称找到属性编码
def key_find_value(key=''):
    if 'dr' in key:
        temp = key.split('.', 1)[-1]
        key = 'dr.' + temp
    dict_temp = property_addresss
    if key in dict_temp.keys():
        return dict_temp[key]
    else:
        return 0
