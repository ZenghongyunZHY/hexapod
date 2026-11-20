uint8 ACCEPTED=1
uint8 COMPLETED=2
uint8 REJECTED=3
uint8 FAILED=4
uint8 INTERRUPTED=5

uint64 plan_id
uint8 state
# 已完成的前缀段数。INTERRUPTED 只允许在已确认的计划边界。
uint32 completed_segments
string reason
