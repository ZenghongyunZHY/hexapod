# 相对整批动作起点的段起点。
uint32 start_offset_ms
# 该段 18 轴共用的动作时长。
uint16 move_duration_ms
# 段终点目标，按物理舵机 ID - 1 排列。
uint16[18] positions
# 一个行走完整周期的末段。原地收脚阶段不标记。
bool walking_cycle_end
