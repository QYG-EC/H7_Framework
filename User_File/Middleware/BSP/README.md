# H7_BSP 板级支持层

本文说明 `User_File/Middleware/BSP` 的职责、资源模型、公开接口与扩展规则。BSP 位于
STM32 HAL/CubeMX 生成层之上，为 Device 和 System 提供稳定的板级通信接口。机器人
业务逻辑不应直接写在 BSP 中；Application 的结构和接入方式见
[Application 文档](../../Application/README.md)。

## 1. 分层位置

```text
Application        机器人控制目标、状态机和模块协作
    ↓
Device / System    设备协议、状态、在线检测、姿态与时间服务
    ↓
Middleware/BSP     总线资源、DMA 缓冲、收发队列和 HAL 回调
    ↓
Core + HAL         CubeMX 生成的句柄、中断入口和外设初始化
    ↓
STM32H723 / MC02   CAN、SPI、UART、USB、OSPI、ADC、GPIO
```

BSP 的目标是把“使用哪个 HAL 句柄、缓冲区放在哪里、回调如何分发、发送失败如何
保留”收敛到一处。Device 负责理解数据协议；Application 负责决定机器人做什么。

## 2. 目录与组件

| 目录 | 主要职责 | 关键入口 |
| --- | --- | --- |
| `CAN` | 三路 FDCAN 接收分发、命令 FIFO、周期 Latest-Value 槽 | `BSP_CAN_ConfigInit`、`CAN_Tx_Submit`、`CAN_Tx_Perform` |
| `SPI` | SPI 管理对象、片选、DMA 事务、错误与超时现场 | `SPI_Init`、`SPI_Transmit_Data`、`SPI_Transmit_Receive_Data` |
| `UART` | Receive-to-Idle DMA、双缓冲、专用 TX 缓冲、错误恢复 | `UART_Init`、`UART_Transmit_Data` |
| `USB` | USB CDC 接收双缓冲与发送接口 | `USB_Init`、`USB_Transmit_Data` |
| `OSPI` | OSPI 命令、收发缓冲和自动轮询回调 | `OSPI_Init`、`OSPI_Command*` |
| `ADC` | ADC DMA 采样管理对象 | `ADC_Init` |

CubeMX 生成的 GPIO、DMA、FDCAN、SPI、UART、TIM、USB 和 OSPI 初始化代码仍位于
`Core` 及 `cmake/stm32cubemx`。BSP 不取代 CubeMX，而是在生成句柄之上增加工程约束。

## 3. 生命周期

### 3.1 启动阶段

1. `main.c` 完成 MPU、Cache、HAL、时钟和 CubeMX 外设初始化。
2. `System_Init()` 初始化时间戳、UART、SPI、OSPI 和板载设备。
3. `osKernelInitialize()` 后，`MX_FREERTOS_Init()` 调用 `BSP_CAN_ConfigInit()`，创建
   CAN 使用的 RTOS 资源并建立任务。
4. 调度器启动后，专用任务处理 CAN 发送、BMI088 解算、在线检测、周期服务和遥测。

必须依赖 RTOS Queue 的初始化不能提前到 `osKernelInitialize()` 之前。静态对象和不依赖
内核的设备配置则由 `System_Init()` 按依赖顺序完成。

### 3.2 回调阶段

HAL 中断回调先进入 BSP 或 `System/callback`：

- CAN 根据 `(FDCAN handle, 帧类型, CAN ID)` 找到注册回调和 `context`。
- SPI/OSPI/UART/USB 根据管理对象交付本次缓冲区和有效长度。
- BMI088 的 SPI/EXTI 回调只推进采集状态机并设置线程标志，VQF 计算留在任务上下文。

中断回调必须短小，不得阻塞、等待 Mutex、执行大规模控制计算或调用只能用于任务的
RTOS API。传入回调的接收缓冲通常只在回调期间有效，需要跨上下文保留时由上层复制。

## 4. CAN 设计

### 4.1 接收

```cpp
bool BSP_CAN_RegisterCallback(uint32_t can_id,
                              FDCAN_HandleTypeDef *hfdcan,
                              CAN_RxCallback_t callback,
                              void *context);
```

原接口继续只注册标准帧；`BSP_CAN_RegisterCallbackEx(id, id_type, bus, callback, context)`
支持指定 `CAN_ID_STANDARD` / `CAN_ID_EXTENDED`。匹配键包含总线、帧类型与完整 ID，
不同总线或不同帧类型可使用相同数值 ID，不会互相分发。回调运行在接收中断中。

CMake 的 `H7_CAN1_EXTENDED`、`H7_CAN2_EXTENDED`、`H7_CAN3_EXTENDED` 独立控制各路，
默认均为 `OFF`，只在构建/初始化时生效；不存在运行中自动切换。关闭的总线拒绝扩展帧
注册和发送，硬件全局过滤也拒绝扩展帧。`BSP_CAN_ExtendedEnabled(bus)` 可查询实际配置。

开启后，该路标准和扩展 Classic 数据帧共享现有 RX FIFO0，软件仅交付已注册 ID。
远程帧和 CAN FD 不交付设备。使用全局扩展帧放行，无需增加过滤器、FIFO 或修改 IOC/Message RAM；
波特率沿用现有外设配置。ISR 每次仅处理进入时已有的帧，避免持续流量形成无界循环。

注册表容量仍为全 BSP 共 16 项，周期槽共 32 项；容量共享不改变按总线隔离的路由。
多 ID 设备初始化失败可用 `BSP_CAN_UnregisterCallbackEx()` 回滚自己成功注册的项；
该接口核对 context，不移除其他设备注册。

### 4.2 发送通道

CAN 刻意保留两种语义，不应按消息频率混用：

| 接口 | 语义 | 适合内容 |
| --- | --- | --- |
| `CAN_Tx_Submit` | FIFO，每次成功提交都保留 | 使能、失能、复位、回零、模式切换 |
| `CAN_Tx_Perform` | 同一 `(总线, 帧类型, ID)` 只保留最新帧 | 电流、速度、位置等周期控制量 |

发送消息应零初始化，新增 `id_type` 字段为零时仍发送标准帧，旧设备调用无需修改。
标准 ID 限制为 11 位，扩展 ID 为 29 位；扩展帧不可仅根据 ID 数值大小推断。

`CAN_Tx_PerformOwned(message, owner)` 用 `(总线, 帧类型, owner)` 固定一个周期槽，
允许 VESC 等设备切换命令 ID 时原位覆盖旧目标。owner 必须静态存活，且其发送 ID 由设备
独占，不与普通周期槽或 FIFO 混用。版本核对与非阻塞 HAL 提交在同一短临界区执行，
防止安全覆盖后再提交旧快照；已经进入硬件 FIFO 的帧无法撤回。

两个接口都会复制 `Struct_CAN_Tx_Msg`，调用返回后可以复用局部变量。返回 `false` 表示
参数无效、资源尚未建立或容量不足，Device 必须保留失败状态或决定重试，不能把“写入
软件通道成功”等同于“电机已经执行”。

`Can_Tx_Task` 每 1 ms 依次调用 `BSP_CAN_SendAsync()` 和 `BSP_CAN_SendPer()`。
FDCAN1/2/3 各有静态 FIFO 和 pending 帧，每次轮询每条总线最多尝试一帧；同总线
pending 先于队列后续帧重试。某条总线暂时不可写只阻塞本总线的后续离散命令，
其他总线和周期槽仍独立处理。软件接受、HAL 写入硬件 FIFO、总线发送、对端收到、
设备执行并非同一事件。

## 5. SPI、UART、USB 与 OSPI

### 5.1 SPI

每个 SPI 管理对象保存句柄、回调、事务状态、片选、收发缓冲、时间戳和诊断计数。
SPI1～SPI5 使用 DMA；当前 SPI6 路径保留短超时阻塞发送。一次事务未结束时再次提交会
失败并增加 Busy 统计，调用方不得覆盖正在使用的缓冲。

SPI2 额外保留 timeout 快照，包括 SPI/DMA 寄存器、HAL 状态、传输计数和 NVIC 状态，
用于分析 BMI088 采集链路的偶发超时。

### 5.2 UART

UART 接收采用 `HAL_UARTEx_ReceiveToIdle_DMA` 和双缓冲，回调交付的是本次 DMA chunk，
可能是半帧或多帧。BSP 不负责协议拼帧、粘包和重同步，Device parser 必须处理。
DMA TX 会先复制
到管理对象的专用发送缓冲；接口返回后调用方可以立即复用原数据。发送在途时返回
`HAL_BUSY`，不会覆盖现有帧。

错误中断只设置恢复标志；`TIM1msTask` 调用
`UART_TIM_1ms_Recover_PeriodElapsedCallback()` 在任务上下文清理 DMA，并每 10 ms
重试待恢复的接收通道。就绪缓冲只保证在当前回调期间有效。

当前管理对象覆盖 UART5、UART7、USART1/2/3/6/10。其他 UART 若要使用 DMA 路径，必须
先补齐管理对象、CubeMX DMA 配置和回调映射。

### 5.3 USB CDC

USB 提供接收双缓冲、接收时间戳和非拥有型回调。`Transport_Task` 初始化 USB Device，
并通过 EricTool 周期输出调试数据。USB 发送结果需要向上传递，忙状态不能被当成成功。

### 5.4 OSPI

OSPI 管理对象持有命令缓冲、自动轮询时间戳及完成回调。当前 W25Q64JV 使用 OSPI2；
`TIM1msTask` 负责自动轮询超时检查，避免 Busy 状态永久锁死。

### 5.5 ADC

采样数量约定与失败行为由 [bsp_adc.h](ADC/bsp_adc.h) 定义。`ADC_Init()` 在调用 HAL 前
检查采样数量是否位于缓冲容量内；数量按采样元素计，不是字节数。

## 6. DMA 与内存约束

STM32H7 的 DMA 可达性与 Cache 一致性属于 BSP 接口契约，而不是可选优化。

| 区域 | 地址 | 大小 | 用途 |
| --- | --- | --- | --- |
| DTCMRAM | `0x20000000` | 128 KiB | CPU 高频数据与部分 FreeRTOS heap；DMA1/2 不可访问 |
| RAM_DMA | `0x24000000` | 64 KiB | `.dma_buffer`；共享、不可缓存的 DMA 缓冲 |
| RAM_D1 | `0x24010000` | 256 KiB | 普通 D1 数据与部分 heap |
| RAM_D2 / D3 | `0x30000000` / `0x38000000` | 32 / 16 KiB | 其他 SRAM，使用前确认 DMA 可达性 |

UART、OSPI 等 DMA 管理对象放入 `.dma_buffer`。修改链接脚本、MPU 属性或缓冲区位置时，
必须同时核对 `h7_memory.ld`、`main.c`/IOC 的 MPU 设置以及外设所用 DMA 控制器。

## 7. 并发与所有权

- BSP 管理对象由对应总线实现拥有，上层只通过公开接口访问。
- ISR 与任务共享字段使用短临界区或经过核对的单生产者/单消费者协议。`volatile` 只限制编译器优化，不能保证多字段快照一致或代替同步。
- CAN 异步发送函数只允许同一发送任务调用，不支持重入。
- UART DMA 发送每个端口同一时刻只有一帧在途；不提供隐式排队。
- SPI 事务通过 `Transaction_Active` 串行化；上层必须处理提交失败。
- BSP 不通过 Message Center 发送底层字节帧；Message Center 用于系统状态和应用消息。

## 8. 添加或修改 BSP 组件

1. 先在 IOC 中确认外设、DMA、IRQ 优先级、引脚和时钟。
2. 定义管理对象、缓冲所有权、并发模型和错误返回，避免只包装一层 HAL 名称。
3. 对通信外设提供明确的初始化、提交和回调接口；回调传递有效长度与用户上下文。
4. DMA 缓冲放入正确内存段，并验证对齐、生命周期、Cache 与 DMA 可达性。
5. 在 `System_Init()` 或 RTOS 初始化阶段按依赖接入，不能依赖静态构造访问硬件。
6. 更新根 CMake 源文件/include、本文档和架构图。
7. 验证成功、Busy、容量耗尽、HAL 启动失败、错误恢复和重复初始化路径。

## 9. 禁止事项

- 不在 Application 中直接绕过 BSP 调用 HAL 收发函数。
- 不在中断回调中阻塞、动态分配或执行完整控制环。
- 不把 DTCM 地址交给 DMA1/DMA2。
- 不在 DMA 正在使用时复用或覆盖缓冲区。
- 不忽略 `bool` / HAL 状态返回值。
- 不把设备协议解析、电机状态机或机器人业务逻辑塞进 BSP。

## 10. 相关文档

- [框架总览](../../../README.md)
- [Application](../../Application/README.md)
- [Message Center](../../System/MessageCenter/README.md)
- [交互式架构图](../../../Assets/Architecture/H7_BSP.html)

## 上游可靠性修复的运行约定

本地选择性移植上游 H7_BSP `ebfb320`、`db0c69d`、`6e7c44c`、`1aa5d25`、
`179c779`、`6ff9d2d` 的修复，保留本地板型调度、CAN 分路队列和设备使能语义。

- UART 恢复在开始时于短临界区消费 `Rx_Restart_Pending`；恢复期间再次收到错误，
  新请求保留到下一轮服务。恢复失败仍保持待恢复状态。
- TIM5 时间戳读取与更新中断均保护溢出计数/硬件标志的一致性，恢复调用前的 PRIMASK。
  继续采用 1 MHz 计数、3600 s 溢出周期，避免 HAL 清标志后被中断读取到旧的软件计数。
- `ADC_TIM_10ms_PeriodElapsedCallback()` 由现有 1 ms 调度表每 10 ms 调用，
  针对板载 ADC1 单通道、软件触发、循环 DMA 配置更新电源采样并尝试错误恢复。
  `Sample_Valid` 和 `Sample_Timestamp` 标记最新完成采样；电压读取在临界区获取一致快照，
  初始化未完成、ADC 错误或样本超过 100 ms 时返回 0 V。
  BMI088 加热在电压无效或低于 10 V 时输出零并清除温控积分；正常时沿用原温控计算。
- W25Q64JV 的 RX/TX 完成 ISR 仅登记 `Auto_Polling_Pending`，现有 1 ms 服务负责续接。
  OSPI 命令及句柄等待超时限制为 1 ms；传输失败或忙超时清除待续接状态并累计错误。
  软件提交成功仍不等于 Flash 已完成操作。
- `BSP_CAN_Recover()` 仅由 CAN 发送任务调用，各路独立维护至少 100 ms 的恢复尝试间隔。
  仅 Bus-Off 且 INIT 置位时清除 INIT 以请求硬件恢复；单独 INIT 不触发恢复。
  Bus-Off 或 INIT 未解除期间不向该路硬件提交报文，软件 FIFO/周期目标继续遵循原有语义，
  其他总线照常发送。该服务不清除设备掉线保护，也不主动重新使能电机。

主机故障注入与验证边界见 [可靠性回归](../../../Tests/Reliability/README.md)。
