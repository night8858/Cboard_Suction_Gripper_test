# Cboard Suction Gripper Test

基于 STM32F407IIH6 的C板控制工程框架，使用 STM32 HAL + FreeRTOS。

当前 README 按仓库现有代码整理，重点描述已经接入构建的模块与任务。

## 1. 工程概览

- MCU: STM32F407IIH6
- 框架: STM32 HAL + CMSIS-RTOS (FreeRTOS)
- 构建系统: CMake + Ninja
- 开发环境: VS Code + openocd + DAPLink

## 1.1 调试构建
- 目标: `build/Debug`
- 任务: `CMake Build Debug`

## 2. 目录结构（按当前代码）
此三处是核心部分
- `Core/`: CubeMX 生成的初始化代码、中断、FreeRTOS 入口。。
- `bsp/`: 板级驱动。
- `hardware_device/`: 电机、舵机、遥控器等设备层代码。

## 3. 使用和构建流程（当前已创建）


## 5. 构建与下载

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


4.30 任务：构建完善的运动空间，限制和运动位置要在安全范围内，避免碰撞和过载。并且尝试移植代码到上位机中，融入到toe_dog里。