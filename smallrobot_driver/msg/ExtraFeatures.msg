# 额外功能   比SmallrobotRobot的PWM结构多出Tag数据
uint32  Tag          # 功能标签
uint32  PSC          # PRC，预分频器系数
uint32  ARR          # ARR，自动重载寄存器
uint32  CCR1         # CCR1，捕获/比较寄存器1，只使用通道1
int32   PluseCount   # 执行脉冲的数量，只有此位不为0时有效,可为负数
int32[] Position   # Position，用于当前角度的设置，即LOCATION_SETTING所使用