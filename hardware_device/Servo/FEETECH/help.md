### FEETECH STS UART 绑定帮助文档

#### 1. 目标
本项目现已支持选择 FEETECH STS 驱动 TX/RX 所使用的 UART 接口。

该功能包含两个层级：

- SCSLib 层：`SCS_SetUART()`
- 应用舵机控制器层：`servo_controller_init(UART_HandleTypeDef *huart)`

#### 2. 公共 API

**SCSLib API (FEETECH/SCSLib)**

- `void SCS_SetUART(UART_HandleTypeDef *huart);`
- `UART_HandleTypeDef *SCS_GetUART(void);`

这些函数声明在 `SCSLib/SCSerail.h` 中，并由 `SCSLib/SCServo.h` 包含。

**舵机控制器 API**

- `void servo_controller_bind_uart(UART_HandleTypeDef *huart);`
- `void servo_controller_init(UART_HandleTypeDef *huart);`

这些函数声明在 `hardware_device/Servo/servo_controller.h` 中。

#### 3. UART 切换示例

**使用 USART1 (默认)**

```c
#include "usart.h"
#include "servo_controller.h"

servo_controller_init(&huart1);
```

**使用 USART3**

```c
#include "usart.h"
#include "servo_controller.h"

servo_controller_init(&huart3);
```

**直接绑定 SCSLib UART**

```c
#include "SCServo.h"
#include "usart.h"

SCS_SetUART(&huart3);
WritePosEx(1, 2048, 300, 40);
```

#### 4. RTOS 任务中的推荐初始化顺序

1. 确保在调度器启动之前调用 `MX_USARTx_UART_Init()`。
2. 在舵机任务入口处，调用一次 `servo_controller_init(&huartx)`。
3. 然后运行周期性的 `servo_controller_task_once()`。
4. 保持一个任务独占一个 FEETECH 总线/UART，以避免帧冲突。

#### 5. 注意事项与风险

- `servo_controller` 的 TX 完成和 RX 完成回调函数会根据绑定的 UART 进行过滤。
- 如果同一个 UART 也被用于调试日志，则需要处理仲裁问题。
- `usart_tx_dma_init(huart)` 仅启用 UART 的 DMAT 位；DMA 流的分配仍取决于 CubeMX 的配置。
- 出于协议可靠性的考虑，`SCSerail.c` 中的 SCSLib 底层读写使用的是阻塞式 HAL UART。