# 上游可靠性修复回归

在仓库根目录执行：

```sh
python3 Tests/Reliability/run.py
python3 Tests/CAN/run.py
cmake --build build/SingleBoard
```

`run.py` 从当前生产文件提取完整目标函数体，使用主机 HAL/对象替身进行故障注入，
以 C++17、警告视为错误和 UBSan 编译运行；生成文件放在 `build/host-reliability`。
这验证函数级分支及状态转换，不替代完整外设驱动、中断时序或固件集成验证。

覆盖：

- UART：恢复期间再次报错、Abort/Start 失败、原中断屏蔽状态保留。
- TIM5：回绕前后、溢出 ISR 尚未执行、HAL 清标志到计数更新窗口、中断状态保留。
- 电源/加热：首帧缺失、100 ms 新鲜度边界、欠压停热/清积分、周期触发和 ADC 错误恢复。
- Flash：ISR 不提交轮询、任务一次性消费、提交失败、超时取消待续接。
- 任务：三个板型分别模拟每个必要任务创建失败，以及全部成功；这是主机检查，不是其他板型固件构建。
- CAN：独立套件编译完整 BSP/Daemon/VESC/EL05 源码，覆盖五种扩展帧开关组合，
  包括单路 Bus-Off 不阻塞其余路、恢复间隔独立、硬件仍 Bus-Off 时禁止发送、主动 INIT 不被重启。

来源为 MermaidFAR/H7_BSP 的 `ebfb320`、`db0c69d`、`6e7c44c`、`1aa5d25`、
`179c779`、`6ff9d2d`；本地手动适配，未整体合并上游。

实机仍需验证 CAN 断线恢复、电压变化与加热输出、Flash 读写及实际中断时序。
