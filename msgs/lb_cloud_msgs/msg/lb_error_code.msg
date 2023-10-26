bool success
uint16 error_code
string error_msg

uint16 ERROR_UNDEFINED               = 0  # 0x00 # 未定义的错误
uint16 ERROR_SCENE_INVALID           = 1  # 0x01 # 场景信息不合法，比如错误的字符等
uint16 ERROR_MAP_UNFOUND             = 2  # 0x02 # (激光雷达模式)当前场景没有地图信息
uint16 ERROR_TRAJ_UNFOUND            = 3  # 0x03 # 当前场景没有轨迹信息
uint16 ERROR_TARGET_MODE_UNMATCH     = 4  # 0x04 # 当前指定终点的形式与当前导航模式不匹配。如果当前导航模式为激光雷达，但是给定的终点是绝对值坐标。
uint16 ERROR_TARGET_POSE_INVALID     = 5  # 0x05 # 目标位置无效，比如距离轨迹过远，数值非法等
uint16 ERROR_VERSION_UNMATCH         = 6  # 0x06 # 协议版本不匹配
uint16 ERROR_VIN_UNMATCH             = 7  # 0x07 # 车辆VIN不匹配
uint16 ERROR_TYPE_INVALID            = 8  # 0x08 # 指令类型非法, 未定义该指令type类型
uint16 ERROR_VEHICLE_UNFUNCTION      = 9  # 0x09 # 导航系统未知故障
uint16 ERROR_SCENE_LOADED            = 16 # 0x10 # 场景已经加载，不能重复加载
uint16 ERROR_SCENE_UNLOADED          = 17 # 0x11 # 场景已经卸载，不能重复卸载

uint16 ERROR_CMD_UNCOMPLETE          = 26 # 0x1a # 指令不完整，比如缺少某些字段
uint16 ERROR_CMD_VALUE_UNDEFINED     = 27 # 0x1b # 指令cmd非法, 未定义
uint16 ERROR_RECORD_IS_ON            = 28 # 0x1c # 已经是录制轨迹状态, 不能重复设置录制轨迹
uint16 ERROR_RECORD_IS_OFF           = 29 # 0x1d # 已经是结束录制状态, 不能重复结束