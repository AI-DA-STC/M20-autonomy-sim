uint16 voltage             # 总电压, 单位10mV
int16  current             # 总电流, 单位10mA
uint16 remaining_capacity  # 剩余容量, 单位10mAH
uint16 nominal_capacity    # 标称容量, 单位10mAH
uint16 cycles              # 循环次数
uint16 production_date     # 生产日期
uint16 balanced_low        # 均衡低
uint16 balanced_high       # 均衡高
uint16 protected_state     # 保护状态

uint8 software_version     # 软件版本
uint8 battery_level        # 剩余容量百分比
uint8 mos_state            # Mos状态
uint8 battery_quantity     # 电池串数
uint8 battery_ntc          # NTC个数
float32[] battery_temperature  # 电池温度序列
char[32] battery_serialnum #电池序列号
