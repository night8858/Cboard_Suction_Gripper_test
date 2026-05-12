#include "i2c_interface.h"
#include "main.h"
#include "stm32f4xx_hal.h"

/// 这里是 I2C 接口的实现文件，提供与小 R 模块通信的底层函数。

// I2C设备地址
const uint8_t I2C_SLAVE_ADDR = 0x20;

// 全局变量，用于处理I2C通信
uint8_t i2c_rx_buffer[32];
size_t i2c_rx_index = 0;
uint8_t lastReceivedCmdId = 0;


int xiao_R_I2C_init(I2C_HandleTypeDef *hi2c)
{
    // 重置接收缓冲区
    i2c_rx_index = 0;
     return 0;
}

//========================================================================
// 函数: int i2c_mem_read(uint16_t mem_addr, uint8_t* buffer, size_t len)
// 功能: 使用mem方式从I2C设备读取数据
//========================================================================
int i2c_mem_read(uint16_t mem_addr, uint8_t* buffer, size_t len) {
    // 使用HAL库的mem读取功能，从指定内存地址读取数据
    if (HAL_I2C_Mem_Read(&hi2c1, I2C_SLAVE_ADDR << 1, mem_addr,
                         I2C_MEMADD_SIZE_8BIT, buffer, len, 100) == HAL_OK) {
        return 0; // 成功
    }
    return -1; // 失败
}

//========================================================================
// 函数: int i2c_mem_write(uint16_t mem_addr, uint8_t* data, size_t len)
// 功能: 使用mem方式向I2C设备写入数据
//========================================================================
int i2c_mem_write(uint16_t mem_addr, uint8_t* data, size_t len) {
    // 使用HAL库的mem写入功能，向指定内存地址写入数据
    if (HAL_I2C_Mem_Write(&hi2c1, I2C_SLAVE_ADDR << 1, mem_addr,
                          I2C_MEMADD_SIZE_8BIT, data, len, 100) == HAL_OK) {
        return 0; // 成功
    }
    return -1; // 失败
}

//========================================================================
// 函数: int i2c_comm_read(uint8_t* buffer, size_t len)
// 功能: 从I2C设备读取数据（兼容旧接口）
//========================================================================
int i2c_comm_read(uint8_t* buffer, size_t len) {
    // 调用mem读取函数，从地址0开始读取
    return i2c_mem_read(0, buffer, len);
}

//========================================================================
// 函数: int i2c_comm_write(uint8_t* data, size_t len)
// 功能: 向I2C设备写入数据（兼容旧接口）
//========================================================================
int i2c_comm_write(uint8_t* data, size_t len) {
    // 调用mem写入函数，从地址0开始写入
    return i2c_mem_write(0, data, len);
}


//========================================================================
// 函数: void processI2CCommunication()
// 功能: 处理I2C通信
//========================================================================
void processI2CCommunication(void) {
    const size_t READ_LEN = 6; // 从0字节开始读取6个数据
    uint8_t rxBuf[READ_LEN];

    // 使用mem方式从0字节开始读取6个数据
    if (i2c_mem_read(0, rxBuf, READ_LEN) == 0) {
        // 检查0字节的最低有效位是否为有效标志位
        if (rxBuf[0] & 0x01) { // 有效标志位为1
            log_print_buf(rxBuf, READ_LEN);

            // 从第0字节开始解析6个字节，0字节+AA+55+类型+ID+FB
            // 协议格式：0字节(标志位) + 0xAA + 0x55 + 类型 + ID + 0xFB
            if (rxBuf[1] == 0xAA && rxBuf[2] == 0x55 && rxBuf[5] == 0xFB) {
                uint8_t cmd_type = rxBuf[3];  // 第3字节是指令类型
                uint8_t cmd_id = rxBuf[4];    // 第4字节是指令ID
                
                printf("解析指令类型: 0x%02X, 指令ID: 0x%02X\r\n", cmd_type, cmd_id);
                printf("指令类型: 0x%02X, 指令ID: %d\r\n", cmd_type, cmd_id);

                // 使用协议库查找对应指令
                ProtocolResult result = find_command_by_type_and_id(cmd_type, cmd_id);
                if (result.success) {
                    printf("指令: %s\r\n", result.command);
                    printf("响应: %s\r\n", result.response);
                    
                    // 记录最后收到的指令ID
                    lastReceivedCmdId = result.cmd_id;
                    
                    printf("等待用户操作... (输入 'play <ID>' 执行对应)\r\n");
                    printf("输入 'help' 查看所有命令\r\n");
                } else {
                    printf("未找到对应指令\r\n");
                }
            } else {
                printf("协议格式不正确: 0xAA 0x55 ... 0xFB\r\n");
            }
        } else {
            // 无效数据，有效标志位为0，则不处理也不打印
        }
    } else {
        // 读取失败
        printf("I2C读取失败\r\n");
    }
}
