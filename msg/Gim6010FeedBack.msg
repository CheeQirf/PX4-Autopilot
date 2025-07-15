# 4 个 GIM6010-8 电机的反馈
uint64 timestamp
float32[4] position      # 电机位置，单位：圈 (rev)
float32[4] velocity      # 电机速度，单位：圈/秒 (rev/s)
float32[4] torque        # 电机扭矩，单位：牛·米 (Nm)
#uint8[4] axis_state      # 电机状态（例如，8 表示闭环控制）
#uint32[4] axis_error     # 错误码
