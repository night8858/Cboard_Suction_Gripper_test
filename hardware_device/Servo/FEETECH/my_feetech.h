#ifndef MY_FEETECH_H
#define MY_FEETECH_H

#include <stdbool.h>
#include <stdint.h>

/**
 * FTSTS_MAX_PARAM_LEN：应答帧参数字段的最大字节数。
 * 普通读操作最多 16 字节，该值提供充足缓冲。
 */
#define FTSTS_MAX_PARAM_LEN 16U

/**
 * 飞特舵机常用寄存器地址枚举（部分）。
 * 完整寄存器表请参考对应型号手册。
 */
enum FTSTS_servo_address {
    current_position = 0x38,  /**< 当前位置（2字节，小端，范围 0~4095） */
    current_speed    = 0x3A,  /**< 当前速度（2字节） */
    current_load     = 0x3C,  /**< 当前负载（2字节） */
};

/**
 * @brief 飞特舵机应答帧解析结果。
 *
 * 由 FTSTS_get_last_frame() 或 servo_get_position() 内部填充。
 * timestamp_ms = 0 表示该 ID 从未收到有效应答。
 */
typedef struct {
    uint8_t  id;                              /**< 舵机 ID */
    uint8_t  length;                          /**< 帧 Length 字段（含 ERROR + CS） */
    uint8_t  error;                           /**< ERROR / 状态字节（0=正常） */
    uint8_t  parameter_len;                   /**< 有效参数字节数 */
    uint8_t  parameter[FTSTS_MAX_PARAM_LEN];  /**< 参数数据 */
    uint8_t  checksum;                        /**< 接收到的校验码 */
    uint32_t timestamp_ms;                    /**< 帧接收时的 HAL_GetTick() 时刻（ms） */
} ftsts_rx_frame_t;

/**
 * @brief 飞特舵机运动指令数据包（供上层批量控制使用）。
 */
typedef struct {
    uint8_t id;            /**< 舵机 ID */
    float   angle;         /**< 目标角度（度） */
    float   speed;         /**< 运动速度 */
    int     back_position; /**< 反馈位置（步进值） */
    int     back_speed;    /**< 反馈速度 */
} FTservo_data_t;

/* =========================================================================
 * 公共接口
 * ========================================================================= */

/**
 * @brief 初始化帧缓存，清零所有历史应答记录。
 *        在系统上电初始化阶段调用一次，传输层由 SCS_SetUART() 负责。
 */
void FTSTS_init(void);

/**
 * @brief 发送 READ DATA 指令（不等待应答）。
 * @param id        舵机 ID
 * @param addr      寄存器起始地址
 * @param data_len  请求读取的字节数
 */
void FTSTS_read_data(uint8_t id, uint8_t addr, uint8_t data_len);

/**
 * @brief 查询帧缓存，获取指定舵机最近一次有效应答帧。
 * @param id        舵机 ID
 * @param out_frame 输出缓冲区（非 NULL）
 * @return          true=找到有效缓存，false=从未收到该 ID 的应答
 */
bool FTSTS_get_last_frame(uint8_t id, ftsts_rx_frame_t *out_frame);

/**
 * @brief 查询指定舵机当前位置（完整收发流程）。
 * @param id  舵机 ID
 * @return    >=0 位置步进值（0~4095）；-1 超时；<-1 舵机报错
 */
int servo_get_position(int id);

/**
 * @brief 清除舵机 EPROM 写入锁（寄存器 0x30 写 0x00）。
 *        写入配置参数前须先调用此函数解锁。
 */
void FTSTS_clear_writelock(int id);

/**
 * @brief 向广播地址写入新 ID（总线上须仅连接一个舵机）。
 * @param id  新 ID 值（0x00~0xFE）
 */
void FTSTS_setID(int id);

/**
 * @brief 设置指定舵机最大力矩（固定写入 700，寄存器 0x10）。
 */
void FTSTS_setMAX_F(int id);

/**
 * @brief 驱动指定舵机运动到目标角度。
 * @param id    舵机 ID
 * @param angle 目标角度（度，FT 系列 0~300°）
 * @param speed 运动速度（0=最大速度）
 */
void FTSTS_servo_write_pos(uint8_t id, float angle, float speed);

/**
 * @brief FT 系列同步写位置（预留接口，暂未实现）。
 */
void FTSTS_servo_Syncwrite_pos(uint8_t id, float angle, float time, float speed);

#endif /* MY_FEETECH_H */
