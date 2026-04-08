# Cboard Suction Gripper Test (STM32F407 + HAL + FreeRTOS)

本项目基于 STM32F407IIH6，使用 STM32 HAL + FreeRTOS 实现吸盘机械臂控制框架。
当前版本已完成多功能线程化框架搭建，包含：

- 多路舵机控制线程（USART1 + DMA + STS 协议帧）
- CAN 电机控制线程（与现有机械臂/泵控制并行）
- 多路电磁阀控制线程（GPIO 电平控制）
- 灯带控制线程（WS2812/SK6812 框架入口，当前为占位实现）
- 心跳监控线程（任务超时告警 + LED 报警）

## 1. 目录说明

- Core: CubeMX 生成的 HAL 初始化、ISR、中间件入口。
- bsp: 板级驱动封装（CAN、USART、LED、Solenoid、LED Strip）。
- hardware_device: 硬件设备层（DM/DJI/BM 电机、遥控器、舵机协议等）。
- application: 业务算法、全局变量、FreeRTOS 任务层。

## 2. 线程架构

FreeRTOS 线程由 Core/Src/freertos.c 统一创建。

- arm_control_task: 机械臂逆解与关节闭环控制（2ms）
- pump_control_task: 泵电机速度控制（2ms）
- motor_can_task: CAN 电机统一调度入口（2ms，框架占位）
- servo_control_task: 8 路舵机批量控制（20ms）
- solenoid_control_task: 多路电磁阀状态下发（2ms）
- led_strip_task: 灯带效果刷新（默认 30ms）
- heartbeat_monitor_task: 心跳监控与告警（20ms）
- led_indicate_task: 基础板载心跳灯（500ms）
- usartr_debug_task: 串口调试输出（4ms）

## 3. 功能块实现状态

### 3.1 多路舵机（USART1 + DMA）

- 文件：hardware_device/Servo/servo_controller.c
- 已实现：8 路舵机目标缓存、STS SyncWrite 帧打包、USART1 DMA 异步下发。
- 已实现：USART1 RX DMA 使能检测与重启入口。
- 说明：此实现优先保证任务框架打通，后续可替换为完整 STS 库 API 调用。

### 3.2 CAN 电机

- 现有 arm_control_task 与 pump_control_task 保持原控制逻辑。
- 新增 motor_can_task 作为统一 CAN 调度扩展位，便于后续加入限幅、仲裁和分组发送。

### 3.3 多路电磁阀（GPIO）

- 文件：bsp/solenoid/bsp_solenoid.c
- 默认映射：
	- 通道 0 -> GPIOH PIN12
	- 通道 1 -> GPIOH PIN10
- 上电默认安全态：全部关闭（RESET）。

### 3.4 灯带（WS2812/SK6812）

- 文件：bsp/led/bsp_led_strip.c
- 当前状态：任务与接口已就绪，DMA/PWM 实际编码发送为占位。
- 后续需要：在 CubeMX 中新增 TIM PWM + DMA 资源，再将占位逻辑替换为 WS2812 脉宽编码发送。

### 3.5 心跳监控

- 文件：application/RTOS_TASK/heartbeat_monitor_task.c
- 机制：各线程周期调用 heartbeat_kick() 更新时间戳；监控线程按阈值判断超时。
- 超时动作：串口告警 + 系统报警标志置位（不自动复位，不强制停机）。

## 4. 共享变量层

文件：application/variables/variables.h / variables.c

新增状态：

- g_servo_state: 8 路舵机目标位置/速度/加速度与 ID 映射
- g_solenoid_state: 多路阀门命令态与反馈态
- g_led_strip_state: 灯带效果参数（模式、亮度、刷新周期）
- g_task_heartbeat_ms[]: 全线程心跳时间戳
- g_system_alarm_active: 告警标志

## 5. 编译

使用 CMake + Ninja（Debug）：

```bash
cmake -S . -B build/Debug
cmake --build build/Debug -j 8
```

若提示 arm-none-eabi-gcc 未找到，请先配置 ARM GCC 工具链路径。

## 6. 已知事项

- USART1 目前承载舵机 DMA 发送，建议调试日志优先走 USART6。
- WS2812 实际 PWM+DMA 波形输出尚未启用 TIM 外设，请在下一阶段补全 CubeMX 配置。
- 新增 motor_can_task 当前为调度框架入口，具体电机策略需按业务继续实现。