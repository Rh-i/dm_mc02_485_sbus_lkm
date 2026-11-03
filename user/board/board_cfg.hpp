/**
 * @file board_cfg.hpp
 * @brief 板级实例配置（dm_mc02_485+sbus_lkm / DM-MC02）
 *
 * @details 由旧工程的 `User/Bsp/bsp_cfg.hpp` + `User/Device/device_cfg.hpp` 合并而来：
 *          - 外设实例改成 C++20 `inline` 变量，定义只在这里出现一次
 *          - 旧 `UartMode::TxRx` → `UartDir::TxRx`，旧 `uart_detail::DmaTransport/ItTransport`
 *            → `uart_mode::dma/it`
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
#include "bsp_gpio.hpp"
#include "bsp_key.hpp"
#include "bsp_pwm.hpp"
#include "bsp_spi.hpp"
#include "bsp_uart.hpp"

#include "device_bmi088.hpp"
#include "device_buzzer.hpp"
#include "device_emmv5_group.hpp"
#include "device_servo.hpp"
// #include "device_r9ds.hpp" // R9DS 暂时不用（随 r9ds 实例一起注释）


/** @brief 板级初始化：各实例 init() */
void board_init(void);


/* ==================== 外设：计时 ==================== */

// bsp_dwt 由库内部初始化：实例定义在 stm32-framework/code/service/system.cpp，
// 初始化在 code/service/system.cpp 的 system_init() 里。工程中直接调用就行
// （如 bsp_dwt.get_delta_s(&last)），不要再定义实例、不要再调 init()。

/* ==================== */


/* ==================== 外设：CAN（3 路，队列各 16 帧） ==================== */

inline BspCanT<16, 16> bsp_can1({.hfdcan = &hfdcan1, .name = "CAN1"});
inline BspCanT<16, 16> bsp_can2({.hfdcan = &hfdcan2, .name = "CAN2"});
inline BspCanT<16, 16> bsp_can3({.hfdcan = &hfdcan3, .name = "CAN3"}); ///< DJI 电机挂这里

/* ==================== */


/* ==================== 外设：UART ==================== */

// DMA 收发（放 .dma_buffer：DMA1/DMA2 访问不到 DTCM）
inline __attribute__((section(".dma_buffer.bsp_uart1"))) BspUartT<uart_mode::dma, 128, 128> bsp_uart1(
  {.huart    = &huart1,
   .mode     = UartDir::TxRx,
   .baudrate = 115200U}); ///< USART1

inline __attribute__((section(".dma_buffer.bsp_uart2"))) BspUartT<uart_mode::dma, 128, 128> bsp_uart2(
  {.huart    = &huart2,
   .mode     = UartDir::TxRx,
   .baudrate = 921600U}); ///< USART2（RS485，Emm_V5）

// USART3 = RS485-3：PD8/PD9 + PB14 硬件 DE，CubeMX 没配 DMA，只能用中断收发
inline BspUartT<uart_mode::it, 128, 128> bsp_uart3(
  {.huart    = &huart3,
   .mode     = UartDir::TxRx,
   .baudrate = 115200U}); ///< USART3（RS485-3）

inline __attribute__((section(".dma_buffer.bsp_uart4"))) BspUartT<uart_mode::dma, 128, 128> bsp_uart4(
  {.huart    = &huart4,
   .mode     = UartDir::TxRx,
   .baudrate = 115200U}); ///< UART4

inline __attribute__((section(".dma_buffer.bsp_uart7"))) BspUartT<uart_mode::dma, 128, 128> bsp_uart7(
  {.huart    = &huart7,
   .mode     = UartDir::TxRx,
   .baudrate = 921600U}); ///< UART7

inline __attribute__((section(".dma_buffer.bsp_uart8"))) BspUartT<uart_mode::it, 128, 128> bsp_uart8(
  {.huart    = &huart8,
   .mode     = UartDir::TxRx,
   .baudrate = 921600U}); ///< UART8（中断收发）

inline __attribute__((section(".dma_buffer.bsp_uart9"))) BspUartT<uart_mode::dma, 128, 128> bsp_uart9(
  {.huart    = &huart9,
   .mode     = UartDir::TxRx,
   .baudrate = 921600U}); ///< UART9

inline __attribute__((section(".dma_buffer.bsp_uart10"))) BspUartT<uart_mode::dma, 128, 128> bsp_uart10(
  {.huart    = &huart10,
   .mode     = UartDir::TxRx,
   .baudrate = 921600U}); ///< USART10

// UART5 = SBUS：只收不发，中断接收（库里有 <it,128,32> 的显式实例化）
inline BspUartT<uart_mode::it, 128, 32> bsp_uart_it5(
  {.huart    = &huart5,
   .mode     = UartDir::RxOnly,
   .baudrate = 100000U}); ///< R9DS 接收机

/** @brief 调试打印：默认走 USART1 */
#define print_uart(...) bsp_uart1.printf(__VA_ARGS__)

/* ==================== */


/* ==================== 外设：SPI ==================== */

// SPI1：原 LCD 接口，CubeMX 配成 2LINES_TXONLY（只引出 SCK/MOSI），无 DMA、无中断。
// 注意：库的传输接口固定走 HAL_SPI_TransmitReceive，TXONLY 模式下只能发不能收；
//       要读回数据得在 CubeMX 改成 2LINES 并接上 MISO。
inline BspSpi bsp_spi1(
  {.hspi          = &hspi1,
   .use_dma       = false,
   .dma_threshold = 16U});

// SPI2：陀螺仪 BMI088，4.3 MHz / CPOL=1 / CPHA=2Edge；CubeMX 没配 DMA，走轮询。
// 本版陀螺仪不启用：bsp_spi2 与挂在它上面的片选 / 设备都只实例化、不 init()。
inline BspSpi bsp_spi2(
  {.hspi          = &hspi2,
   .use_dma       = false,
   .dma_threshold = 16U});

// 片选：加速度计读寄存器时地址后要跟 1 个无效字节，陀螺仪不跟
inline BspSpiDevice bmi088_acc_spi(
  bsp_spi2,
  {.cs_port    = GYRO_ACC_CS_GPIO_Port,
   .cs_pin     = GYRO_ACC_CS_Pin,
   .read_flag  = 0x80U,
   .read_dummy = 1U});
inline BspSpiDevice bmi088_gyro_spi(
  bsp_spi2,
  {.cs_port    = GYRO_GYRO_CS_GPIO_Port,
   .cs_pin     = GYRO_GYRO_CS_Pin,
   .read_flag  = 0x80U,
   .read_dummy = 0U});

/// BMI088 六轴 IMU：SPI2 上的两颗芯片（自动刷新 + VQF 姿态）。本版不 init()，
/// 既不会建刷新任务，也不会注册数据就绪中断。
inline DeviceBmi088 imu_bmi088(
  bmi088_acc_spi,
  bmi088_gyro_spi,
  {.acc_range  = DeviceBmi088::AccRange::G24,
   .gyro_range = DeviceBmi088::GyroRange::DPS2000});

/* ==================== */


/* ==================== 外设：GPIO / 按键 ==================== */

inline BspGpio power_24v_2({.port = POWER_24V_2_GPIO_Port, .pin = POWER_24V_2_Pin}); ///< PC13
inline BspGpio power_24v_1({.port = POWER_24V_1_GPIO_Port, .pin = POWER_24V_1_Pin}); ///< PC14
inline BspGpio power_5v({.port = POWER_5V_GPIO_Port, .pin = POWER_5V_Pin});          ///< PC15

inline BspGpio gyro_acc_cs({.port = GYRO_ACC_CS_GPIO_Port, .pin = GYRO_ACC_CS_Pin});    ///< PC0
inline BspGpio gyro_gyro_cs({.port = GYRO_GYRO_CS_GPIO_Port, .pin = GYRO_GYRO_CS_Pin}); ///< PC3
inline BspGpio spi1_cs({.port = SPI1_CS_GPIO_Port, .pin = SPI1_CS_Pin});                ///< PE15

inline BspGpio btb_pa5({.port = BTB_PA5_GPIO_Port, .pin = BTB_PA5_Pin});
inline BspGpio btb_pe14({.port = BTB_PE14_GPIO_Port, .pin = BTB_PE14_Pin});
inline BspGpio btb_pd10({.port = BTB_PD10_GPIO_Port, .pin = BTB_PD10_Pin});

inline BspKey key_user(
  {.port           = KEY_GPIO_Port,
   .pin            = KEY_Pin,
   .active_low     = true,
   .debounce_cnt   = 1U,
   .long_press_cnt = 5U}); ///< PA15，低有效

/* ==================== */


/* ==================== 外设：PWM ==================== */

// 频率 = timer_clk_hz / (prescaler + 1) / (period + 1)；同一定时器的两个通道必须同频
inline BspPwm bsp_pwm1(
  {.htim         = &htim1,
   .channel      = TIM_CHANNEL_3,
   .timer_clk_hz = 275000000UL,
   .prescaler    = 274U,
   .period       = 19999U}); ///< PE13 舵机 50 Hz
inline BspPwm bsp_pwm2(
  {.htim         = &htim1,
   .channel      = TIM_CHANNEL_1,
   .timer_clk_hz = 275000000UL,
   .prescaler    = 274U,
   .period       = 19999U}); ///< PE9  舵机 50 Hz
inline BspPwm bsp_pwm3(
  {.htim         = &htim2,
   .channel      = TIM_CHANNEL_3,
   .timer_clk_hz = 275000000UL,
   .prescaler    = 274U,
   .period       = 19999U}); ///< PA2  舵机 50 Hz
inline BspPwm bsp_pwm4(
  {.htim         = &htim2,
   .channel      = TIM_CHANNEL_1,
   .timer_clk_hz = 275000000UL,
   .prescaler    = 274U,
   .period       = 19999U}); ///< PA0  舵机 50 Hz
inline BspPwm bsp_pwm_gyro(
  {.htim         = &htim3,
   .channel      = TIM_CHANNEL_4,
   .timer_clk_hz = 275000000UL,
   .prescaler    = 23U,
   .period       = 9999U}); ///< PB1 陀螺仪
inline BspPwm bsp_pwm_buzzer(
  {.htim         = &htim12,
   .channel      = TIM_CHANNEL_2,
   .timer_clk_hz = 275000000UL,
   .prescaler    = 23U,
   .period       = 1999U}); ///< PB15 无源蜂鸣器

/* ==================== */


/* ==================== 设备实例 ==================== */

inline DeviceBuzzer buzzer(bsp_pwm_buzzer);

// R9DS 遥控接收机：UART5（SBUS，中断接收，不占 DMA）—— 暂时不用，连同板级 init 一起注释
// inline DeviceR9dsT<decltype(bsp_uart_it5)> r9ds(bsp_uart_it5);

/// Emm_V5 多电机组：一条 485 总线，地址 1/2 为 CoreXY 两轴，3 为旋转轴
inline DeviceEmmv5GroupT<decltype(bsp_uart2)> emm_group({.uart = bsp_uart2, .addr = {1, 2, 3}});

/// 舵机：4 路排针 PWM 已配成 50 Hz / 20 ms、1 计数 = 1 µs
inline DeviceServo servo1(
  {.pwm          = &bsp_pwm1,
   .pulse_min_us = 500.0f,
   .pulse_max_us = 2500.0f,
   .range_deg    = 270.0f});
inline DeviceServo servo2(
  {.pwm            = &bsp_pwm2,
   .pulse_min_us   = 500.0f,
   .pulse_max_us   = 2500.0f,
   .range_deg      = 180.0f,
   .max_speed_dps  = 300.0f,
   .max_accel_dps2 = 1500.0f});
inline DeviceServo servo3(
  {.pwm            = &bsp_pwm3,
   .pulse_min_us   = 500.0f,
   .pulse_max_us   = 2500.0f,
   .range_deg      = 180.0f,
   .max_speed_dps  = 300.0f,
   .max_accel_dps2 = 1500.0f});
inline DeviceServo servo4(
  {.pwm            = &bsp_pwm4,
   .pulse_min_us   = 500.0f,
   .pulse_max_us   = 2500.0f,
   .range_deg      = 180.0f,
   .max_speed_dps  = 300.0f,
   .max_accel_dps2 = 1500.0f});

/* ==================== */

#endif // __BOARD_CFG_HPP__
