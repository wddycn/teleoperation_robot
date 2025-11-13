import pygame
import time

# 初始化 pygame
pygame.init()
pygame.joystick.init()

# 检测手柄
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("❌ 没检测到任何手柄，请先插入再运行！")
    exit()
else:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"✅ 检测到手柄: {joystick.get_name()}")
    print(f"按钮数量: {joystick.get_numbuttons()}")
    print(f"轴数量: {joystick.get_numaxes()}")
    print(f"方向帽数量: {joystick.get_numhats()}")
    print("-" * 40)
    print("🎮 请操作手柄（按键、摇杆、方向键），Ctrl+C 退出\n")

# 主循环
try:
    while True:
        pygame.event.pump()  # 刷新事件队列

        # 打印轴的值
        for i in range(joystick.get_numaxes()):
            axis_val = joystick.get_axis(i)
            if abs(axis_val) > 0.1:  # 加一个阈值避免太多 0
                print(f"Axis {i}: {axis_val:.3f}")

        # 打印按钮状态
        for i in range(joystick.get_numbuttons()):
            if joystick.get_button(i):
                print(f"Button {i} pressed")

        # 打印方向帽状态（一般是十字键）
        for i in range(joystick.get_numhats()):
            hat_val = joystick.get_hat(i)
            if hat_val != (0, 0):
                print(f"Hat {i}: {hat_val}")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n👋 退出调试。")
finally:
    pygame.quit()
