# Cboard Suction Gripper Test

基于 STM32F407IIH6 的吸盘机械臂控制工程，使用 STM32 HAL + FreeRTOS。

当前 README 按仓库现有代码整理，重点描述已经接入构建的模块与任务。

## 1. 工程概览

- MCU: STM32F407IIH6
- 框架: STM32 HAL + CMSIS-RTOS (FreeRTOS)
- 构建系统: CMake + Ninja
- 主要入口:
  - `Core/Src/main.c`
  - `Core/Src/freertos.c`
  - `CMakeLists.txt`

启动流程（简化）：

1. `main()` 完成 HAL 与外设初始化（GPIO/DMA/CAN/UART）。
2. 执行 `can_filter_init()`、`usart1_tx_dma_init()`。
3. 调用 `MX_FREERTOS_Init()` 创建线程。
4. `osKernelStart()` 启动调度。

## 2. 目录结构（按当前代码）

- `Core/`: CubeMX 生成的初始化代码、中断、FreeRTOS 入口。
- `application/Algorithm/`: 通用算法（PID、user_lib）。
- `application/Gripper/`: 机械臂/云台/气动/命令解析/吸附检测逻辑。
- `application/RTOS_TASK/`: 业务任务线程实现。
- `application/variables/`: 全局共享状态与结构体实例。
- `bsp/can`, `bsp/usart`, `bsp/led`, `bsp/virtual_serial_port`: 板级驱动。
- `hardware_device/`: 电机、舵机、遥控器设备层代码。

## 3. FreeRTOS 任务（当前已创建）

由 `Core/Src/freertos.c` 中 `MX_FREERTOS_Init()` 创建：

- `defaultTask` (`StartDefaultTask`)
- `RobotArm_TASK` (`arm_control_task`)
- `LED_TASK` (`led_indicate_task`)
- `Pump_control` (`pump_control_task`)
- `debug_TASK` (`usartr_debug_task`)
- `data_update_TAS` (`update_task`)

各任务当前周期（来自任务实现）：

- `arm_control_task`: `osDelay(4)`，约 4ms
- `pump_control_task`: `osDelay(4)`，约 4ms
- `usartr_debug_task`: `osDelay(4)`，约 4ms
- `update_task`: `osDelay(20)`，约 20ms
- `led_indicate_task`: `osDelay(500)`，约 500ms

## 4. 关键功能模块

### 4.1 机械臂控制

- 文件：
  - `application/Gripper/arm_control.c`
  - `application/Gripper/Planar_Robot_Arm.c`
  - `application/RTOS_TASK/arm_control_task.c`
- 说明：`arm_control_task` 中执行 `planar_robot_arm_all_init()` 与 `planar_arm_control_loop()`。

### 4.2 泵电机控制（M3508）

- 文件：
  - `application/Gripper/pneumatic_control.c`
  - `application/RTOS_TASK/pump_control_task.c`
- 说明：任务初始化 `pump_M3508_init()`，循环调用 `pump_speed_set(3000.0f)`。

### 4.3 串口调试输出

- 文件：
  - `application/RTOS_TASK/usart_debug_task.c`
  - `bsp/usart/bsp_usart.c`
- 说明：当前主要通过 `huart6` 输出调试信息。

### 4.4 上下位机命令与反馈

- 文件：
  - `application/Gripper/command_decode.c`
  - `bsp/virtual_serial_port/virtual_serial_port.c`
- 说明：包含 AA 帧协议、CRC8、反馈帧发送与命令解析状态机。

### 4.5 物块吸附检测（微动开关/光电）

- 文件：
  - `application/Gripper/block_inspect.c`
  - `application/Gripper/block_inspect.h`
- 说明：
  - EXTI 回调中只记录事件；
  - 在 `update_task` 中通过 `block_inspect_process()` 做延时去抖后采样；
  - 当前已接入 `PB12`（`GPIO_PIN_12`）一路。

### 4.6 LED 与灯带

- 文件：
  - `bsp/led/bsp_led.c`
  - `bsp/led/bsp_led_strip.c`
- 说明：`bsp_led_strip.c` 目前是服务框架与占位逻辑，未接入实际 WS2812 PWM+DMA 波形输出。

## 5. 全局变量

- 文件：`application/variables/variables.h`、`application/variables/variables.c`
- 当前包含：
  - 机械臂实例：`Arm_LF/Arm_RF/Arm_LB/Arm_RB`
  - 电机数据：`DMmotor_4340[]`、`DMmotor_4310[]`、`DJI_motor_3508`
  - 云台实例：`Gimbal`
  - 电磁阀状态：`solenoid_state`
  - 心跳数组与报警标志：`g_task_heartbeat_ms[]`、`g_system_alarm_active`

注意：`heartbeat_kick()` 与 `heartbeat_get_age_ms()` 在 `variables.c` 中目前为注释状态（未启用实现）。

## 6. 构建与下载

### 6.1 CMake 构建

```bash
cmake -S . -B build/Debug
cmake --build build/Debug -j 8
```

### 6.2 VS Code 任务

工作区已定义任务：

- `CMake Build Debug`
- `Flash STM32 (DAPLink/OpenOCD)`

## 7. 当前状态说明（避免误解）

以下能力在历史文档中出现过，但按当前仓库代码并未以独立任务完整启用：

- 独立 `motor_can_task`
- 独立 `servo_control_task`
- 独立 `solenoid_control_task`
- 独立 `heartbeat_monitor_task`

当前实现以现有 6 个 FreeRTOS 任务为主，部分扩展功能仍保留为结构体/接口或注释代码，便于后续继续开发。
