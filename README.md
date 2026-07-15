# Cboard Suction Gripper Test

基于 STM32F407IIH6、STM32 HAL 和 FreeRTOS 的双臂吸盘控制工程。

## 开发环境

本工程使用 CMake + Ninja 构建，通过 OpenOCD 支持 DAPLink 和 ST-Link
烧录/调试。仓库不保存任何用户目录或工具版本路径，以下命令必须能从
Linux 的 `PATH` 中直接找到：

```bash
cmake --version
ninja --version
arm-none-eabi-gcc --version
arm-none-eabi-gdb --version
openocd --version
```

最低要求：CMake 3.22、Ninja、GNU Arm Embedded Toolchain（包含 GDB）和
OpenOCD。Ubuntu/Debian 可先安装基础软件包：

```bash
sudo apt update
sudo apt install cmake ninja-build gcc-arm-none-eabi openocd
```

如果发行版软件包没有提供 `arm-none-eabi-gdb`，请安装 Arm GNU Toolchain
完整版，并将其 `bin` 目录加入 PATH。例如：

```bash
export PATH="/opt/arm-gnu-toolchain/bin:$PATH"
```

需要永久生效时，将对应的 `export PATH=...` 写入 `~/.profile`，然后重新
登录。也可以只设置 `STM32_TOOLCHAIN_PATH`，其值必须是包含
`arm-none-eabi-gcc` 的 `bin` 目录；但 VS Code 调试所需的 GDB 仍应位于 PATH。

## VS Code

打开仓库后，按扩展推荐提示安装：

- CMake Tools
- C/C++
- Cortex-Debug

随后通过 `Terminal → Run Task` 使用以下任务：

- `CMake Build Debug`：配置并编译 Debug 固件。
- `Flash STM32 (DAPLink)`：编译后使用 CMSIS-DAP/DAPLink 烧录。
- `Flash STM32 (ST-Link)`：编译后使用 ST-Link 烧录。

烧录任务统一使用：

```text
build/Debug/Cboard_Suction_Gripper_test.elf
```

按 `F5` 调试时，可选择 `STM32 Debug (DAPLink)` 或
`STM32 Debug (ST-Link)`；两种配置都会先执行 Debug 构建并运行到 `main`。

## 命令行构建与烧录

Debug 构建：

```bash
cmake --preset Debug
cmake --build --preset Debug
```

DAPLink 烧录：

```bash
openocd -f .vscode/openocd-daplink.cfg \
  -c "program build/Debug/Cboard_Suction_Gripper_test.elf verify reset exit"
```

ST-Link 烧录：

```bash
openocd -f .vscode/openocd-stlink.cfg \
  -c "program build/Debug/Cboard_Suction_Gripper_test.elf verify reset exit"
```

## 下载器连接与权限

默认使用 SWDIO、SWCLK、GND 和 NRST，OpenOCD 速率为 4 MHz。DAPLink 配置
匹配当前 `0x0d28:0x0204` 设备；ST-Link 配置使用 OpenOCD 自带的标准接口。

若 OpenOCD 报 USB 权限错误，请确认发行版已经安装 OpenOCD 的 udev 规则，
并将当前用户加入 `plugdev` 组：

```bash
sudo usermod -aG plugdev "$USER"
sudo udevadm control --reload-rules
sudo udevadm trigger
```

重新登录并重新插拔下载器后再烧录。若连接不稳定，可将对应 OpenOCD 配置中
的 `adapter speed 4000` 降为 `1000`。

## 目录概览

- `Core/`：CubeMX 生成的初始化代码、中断与 FreeRTOS 入口。
- `application/`：机械臂动作、运动学、任务和控制逻辑。
- `bsp/`：板级外设接口。
- `hardware_device/`：电机、舵机及其他硬件设备驱动。
- `tests/`：可在主机运行的核心逻辑回归测试。
