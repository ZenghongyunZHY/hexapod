uint64 plan_id
# 本计划所基于的 StandingReference 就绪时刻。
builtin_interfaces/Time standing_stamp
# 希望控制板开始执行首段的机器人系统时钟时刻。
builtin_interfaces/Time motion_start
# 首段之前的名义参考，按物理舵机 ID - 1 排列。
uint16[18] initial_positions
ServoSegment[] segments
