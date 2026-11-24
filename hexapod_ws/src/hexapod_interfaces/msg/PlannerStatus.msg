uint64 command_sequence
uint64 plan_id
string state
string reason
# 规划器最后接收的站姿状态时刻，用于跨话题关联本轮就绪。
builtin_interfaces/Time standing_stamp
# accepted 时，从状态发出起允许等待执行完成的毫秒数，已含反馈余量。
uint64 completion_timeout_ms
