# 灵足时代 EL05 运控驱动

使用灵足私有 CAN 2.0 扩展帧协议，电机默认波特率 1 Mbps。通过 CMake 的
`H7_CANx_EXTENDED` 启用对应总线。电机端须配置为私有协议和运控模式。
驱动由应用显式创建静态实例并配置总线、电机 ID 和主机 ID，不绑定具体机构。

1. 调用 `Init(bus, motor_id, master_id)`，检查返回值。
2. 调用 `RequestEnabled(true)` 提交使能。等待首帧时保留请求，默认 100 ms 内
   收到无故障且处于运行状态的反馈后，`GetSnapshot().ready` 才为真。
3. 就绪后周期调用 `SetMotion(position_rad, velocity_rad_s, kp, kd, torque_nm)`。
   上层负责机构的速度、力矩及位置限幅。
4. 停止时调用 `RequestEnabled(false)`。使能确认超时、掉线或故障会锁定输出，
   由驱动覆盖待发送命令为停止帧。应用恢复前须显式请求失能，再请求使能。
   返回成功仅表示接受软件请求。已进入硬件 FIFO 的帧无法撤回。
   总线完全断开时停止指令也无法送达，实机须核对电机自身通信超时参数。

首版支持运控、使能、失能，以及角度、角速度、力矩、温度和状态反馈。
位置为协议反馈值，未做多圈展开。采用官方 SDK 的 EL05 映射
（±12.57 rad、±50 rad/s、±6 N·m），与旧手册示例中的 ±5.5 N·m 不同。
上板前核对目标电机固件量程。

[官方 SDK 协议汇总](https://github.com/RobStride/EDULITE_A3/blob/main/el_a3_sdk/docs/%E7%94%B5%E6%9C%BA%E9%80%9A%E4%BF%A1%E5%8D%8F%E8%AE%AE%E6%B1%87%E6%80%BB.md)
