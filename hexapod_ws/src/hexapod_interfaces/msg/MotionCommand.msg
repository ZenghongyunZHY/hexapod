uint8 TELEOP=1
uint8 WALK_CYCLES=2
uint8 STOP=3

uint8 kind
# 单一命令接收节点为命令单调递增编号。
uint64 sequence
# 机器人系统时钟下的命令时刻。
builtin_interfaces/Time stamp
float64 vx_m_s
float64 vy_m_s
float64 wz_rad_s
# 仅 WALK_CYCLES 使用。收脚不计入此计数。
uint32 cycle_count
