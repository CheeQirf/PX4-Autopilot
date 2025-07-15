# 4 个 GIM6010-8 电机的命令设定点
uint64 timestamp
float32[4] position      # 目标位置，单位：圈 (rev)
int16[4] velocity      # 目标速度，单位：圈/秒 (rev/s)
int16[4] torque        # 目标扭矩，单位：牛·米 (Nm)

