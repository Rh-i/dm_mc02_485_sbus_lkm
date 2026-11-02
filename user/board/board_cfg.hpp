/**
 * @file board_cfg.hpp
 * @brief 板级实例配置（dm_mc02_lkm / DM-MC02）
 *
 * @details 由旧工程的 `User/Bsp/bsp_cfg.hpp` + `User/Device/device_cfg.hpp` 合并而来：
 *          - 外设实例改成 C++20 `inline` 变量，定义只在这里出现一次
 *          - 旧 `UartMode::TxRx` → `UartDir::TxRx`，旧 `uart_detail::DmaTransport/ItTransport`
 *            → `uart_mode::dma/it`
 *          - 旧 `DjiMotorGroupT`（CAN3 上 2 台 GM6020）按新库拆成两个 `DjiMotor<Motor6020, ...>`
 *          - 旧 `DeviceMotor` 的 `.uart` / `pid_print()` 新库没有，波形改用 `pid_frame()`
 *
 *          【硬规矩】
 *            - 串口 / SPI 用 DMA 时必须放 `.dma_buffer`
 *            - 只能用库里已有的 `BspUartT` 组合：`<dma,128,128>`、`<it,128,128>`、`<it,128,32>`；
 *              `BspCanT` 只有 `<16,16>`
 *            - 每个实例都要在 `board_instances.hpp` 里出现，否则中断不会分发
 */

#ifndef __BOARD_CFG_HPP__
#define __BOARD_CFG_HPP__

// CubeMX 句柄
#include "fdcan.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

#include "bsp_can.hpp"
#include "bsp_dwt.hpp"
#include "bsp_gpio.hpp"
#include "bsp_key.hpp"
#include "bsp_pwm.hpp"
#include "bsp_uart.hpp"

#include "device_buzzer.hpp"
#include "device_emmv5_group.hpp"
#include "device_r9ds.hpp"
#include "device_servo.hpp"
#include "dji_motor.hpp"


/** @brief 板级初始化：各实例 init() */
void board_init(void);


/* ==================== 外设：计时 ==================== */

inline BspDwt bsp_dwt {}; ///< 内核 CYCCNT 计时，CPU 频率由 init() 从 RCC 读出


/* ==================== 外设：CAN（3 路，队列各 16 帧） ==================== */

inline BspCanT<16, 16> bsp_can1({.hfdcan = &hfdcan1, .name = "CAN1"});
inline BspCanT<16, 16> bsp_can2({.hfdcan = &hfdcan2, .name = "CAN2"});
inline BspCanT<16, 16> bsp_can3({.hfdcan = &hfdcan3, .name = "CAN3"}); ///< DJI 电机挂这里


/* ==================== 外设：UART ==================== */

// DMA 收发（放 .dma_buffer：DMA1/DMA2 访问不到 DTCM）
inline __attribute__((section(".dma_buffer.bsp_uart1"))) BspUartT<uart_mode::dma, 128, 128> bsp_uart1(
    {.huart = &huart1, .mode = UartDir::TxRx, .baudrate = 115200U}); ///< USART1

inline __attribute__((section(".dma_buffer.bsp_uart2"))) BspUartT<uart_mode::dma, 128, 128> bsp_uart2(
    {.huart = &huart2, .mode = UartDir::TxRx, .baudrate = 921600U}); ///< USART2（RS485，Emm_V5）

inline __attribute__((section(".dma_buffer.bsp_uart3"))) BspUartT<uart_mode::dma, 128, 128> bsp_uart3(
    {.huart = &huart3, .mode = UartDir::TxRx, .baudrate = 115200U}); ///< USART3（调试打印）

inline __attribute__((section(".dma_buffer.bsp_uart4"))) BspUartT<uart_mode::dma, 128, 128> bsp_uart4(
    {.huart = &huart4, .mode = UartDir::TxRx, .baudrate = 115200U}); ///< UART4

inline __attribute__((section(".dma_buffer.bsp_uart7"))) BspUartT<uart_mode::dma, 128, 128> bsp_uart7(
    {.huart = &huart7, .mode = UartDir::TxRx, .baudrate = 921600U}); ///< UART7

inline __attribute__((section(".dma_buffer.bsp_uart8"))) BspUartT<uart_mode::it, 128, 128> bsp_uart8(
    {.huart = &huart8, .mode = UartDir::TxRx, .baudrate = 921600U}); ///< UART8（中断收发）

inline __attribute__((section(".dma_buffer.bsp_uart9"))) BspUartT<uart_mode::dma, 128, 128> bsp_uart9(
    {.huart = &huart9, .mode = UartDir::TxRx, .baudrate = 921600U}); ///< UART9

inline __attribute__((section(".dma_buffer.bsp_uart10"))) BspUartT<uart_mode::dma, 128, 128> bsp_uart10(
    {.huart = &huart10, .mode = UartDir::TxRx, .baudrate = 921600U}); ///< USART10

// UART5 = SBUS：只收不发，中断接收（库里有 <it,128,32> 的显式实例化）
inline BspUartT<uart_mode::it, 128, 32> bsp_uart_it5(
    {.huart = &huart5, .mode = UartDir::RxOnly, .baudrate = 100000U}); ///< R9DS 接收机

/** @brief 调试打印：默认走 USART3 */
#define print_uart(...) bsp_uart3.printf(__VA_ARGS__)


/* ==================== 外设：GPIO / 按键 ==================== */

inline BspGpio power_24v_2({.port = POWER_24V_2_GPIO_Port, .pin = POWER_24V_2_Pin}); ///< PC13
inline BspGpio power_24v_1({.port = POWER_24V_1_GPIO_Port, .pin = POWER_24V_1_Pin}); ///< PC14
inline BspGpio power_5v({.port = POWER_5V_GPIO_Port, .pin = POWER_5V_Pin});          ///< PC15

inline BspGpio gyro_acc_cs({.port = GYRO_ACC_CS_GPIO_Port, .pin = GYRO_ACC_CS_Pin});   ///< PC0
inline BspGpio gyro_gyro_cs({.port = GYRO_GYRO_CS_GPIO_Port, .pin = GYRO_GYRO_CS_Pin}); ///< PC3
inline BspGpio spi1_cs({.port = SPI1_CS_GPIO_Port, .pin = SPI1_CS_Pin});               ///< PE15

inline BspGpio btb_pa5({.port = BTB_PA5_GPIO_Port, .pin = BTB_PA5_Pin});
inline BspGpio btb_pe14({.port = BTB_PE14_GPIO_Port, .pin = BTB_PE14_Pin});
inline BspGpio btb_pd10({.port = BTB_PD10_GPIO_Port, .pin = BTB_PD10_Pin});

inline BspKey key_user({.port = KEY_GPIO_Port, .pin = KEY_Pin, .active_low = true,
                        .debounce_cnt = 1U, .long_press_cnt = 5U}); ///< PA15，低有效


/* ==================== 外设：PWM ==================== */

// 频率 = timer_clk_hz / (prescaler + 1) / (period + 1)；同一定时器的两个通道必须同频
inline BspPwm bsp_pwm1({.htim = &htim1, .channel = TIM_CHANNEL_3, .timer_clk_hz = 275000000UL,
                        .prescaler = 274U, .period = 19999U}); ///< PE13 舵机 50 Hz
inline BspPwm bsp_pwm2({.htim = &htim1, .channel = TIM_CHANNEL_1, .timer_clk_hz = 275000000UL,
                        .prescaler = 274U, .period = 19999U}); ///< PE9  舵机 50 Hz
inline BspPwm bsp_pwm3({.htim = &htim2, .channel = TIM_CHANNEL_3, .timer_clk_hz = 275000000UL,
                        .prescaler = 274U, .period = 19999U}); ///< PA2  舵机 50 Hz
inline BspPwm bsp_pwm4({.htim = &htim2, .channel = TIM_CHANNEL_1, .timer_clk_hz = 275000000UL,
                        .prescaler = 274U, .period = 19999U}); ///< PA0  舵机 50 Hz
inline BspPwm bsp_pwm_gyro({.htim = &htim3, .channel = TIM_CHANNEL_4, .timer_clk_hz = 275000000UL,
                            .prescaler = 23U, .period = 9999U}); ///< PB1 陀螺仪
inline BspPwm bsp_pwm_buzzer({.htim = &htim12, .channel = TIM_CHANNEL_2, .timer_clk_hz = 275000000UL,
                              .prescaler = 23U, .period = 1999U}); ///< PB15 无源蜂鸣器


/* ==================== 设备实例 ==================== */

inline DeviceBuzzer buzzer(bsp_pwm_buzzer);

/**
 * CAN3 上的两台 GM6020（旧 `DjiMotorGroupT` 拆开）
 *   旧：反馈 0x205（电调 ID 1）→ 0x1FE 槽 0；反馈 0x20A（电调 ID 6）→ 0x2FE 槽 1
 *   新：`DjiMotor<Motor6020, CanT>(can, 电调ID)`，发送帧由 CanTxNode 自动聚合
 */
inline DjiMotor<Motor6020, decltype(bsp_can3)> gm6020_1(bsp_can3, 1U);
inline DjiMotor<Motor6020, decltype(bsp_can3)> gm6020_6(bsp_can3, 6U);

/// R9DS 遥控接收机：UART5（SBUS，中断接收，不占 DMA）
inline DeviceR9dsT<decltype(bsp_uart_it5)> r9ds(bsp_uart_it5);

/// Emm_V5 多电机组：一条 485 总线，地址 1/2 为 CoreXY 两轴，3 为旋转轴
inline DeviceEmmv5GroupT<decltype(bsp_uart2)> emm_group({.uart = bsp_uart2, .addr = {1, 2, 3}});

/// 舵机：4 路排针 PWM 已配成 50 Hz / 20 ms、1 计数 = 1 µs
inline DeviceServo servo1({.pwm = &bsp_pwm1, .pulse_min_us = 500.0f, .pulse_max_us = 2500.0f,
                           .range_deg = 270.0f});
inline DeviceServo servo2({.pwm = &bsp_pwm2, .pulse_min_us = 500.0f, .pulse_max_us = 2500.0f,
                           .range_deg = 180.0f, .max_speed_dps = 300.0f, .max_accel_dps2 = 1500.0f});
inline DeviceServo servo3({.pwm = &bsp_pwm3, .pulse_min_us = 500.0f, .pulse_max_us = 2500.0f,
                           .range_deg = 180.0f, .max_speed_dps = 300.0f, .max_accel_dps2 = 1500.0f});
inline DeviceServo servo4({.pwm = &bsp_pwm4, .pulse_min_us = 500.0f, .pulse_max_us = 2500.0f,
                           .range_deg = 180.0f, .max_speed_dps = 300.0f, .max_accel_dps2 = 1500.0f});

#endif // __BOARD_CFG_HPP__
