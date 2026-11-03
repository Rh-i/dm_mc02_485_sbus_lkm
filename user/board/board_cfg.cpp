/**
 * @file board_cfg.cpp
 * @brief board_init()：把 board_cfg.hpp 里的实例逐个 init()（dm_mc02_485+sbus_lkm）
 *
 * @details 旧工程的 `bsp_init()` + `device_init()` 合并到这里：
 *          DWT → CAN → UART → PWM/GPIO/按键 → 设备（Emm_V5 / 舵机）。
 *          设备要放在对应外设 init() 之后（它们要注册 CanRxNode / CanTxNode 或用已初始化的串口）。
 *
 *          【失败就地停机】每行 configASSERT，fault_record 会记下行号，print_uart 能打出来。
 */

#include "board_cfg.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep (configASSERT)


void board_init(void)
{
  // 计时基准：sys_task 用它算 10 ms 周期内的耗时，必须最先起来
  configASSERT(bsp_dwt.init() == Status::OK);

  // CAN：DJI 电机挂 CAN3
  configASSERT(bsp_can1.init() == Status::OK);
  configASSERT(bsp_can2.init() == Status::OK);
  configASSERT(bsp_can3.init() == Status::OK);

  // 串口：DMA 收发 + UART5 中断收（SBUS）
  configASSERT(bsp_uart1.init() == Status::OK);
  configASSERT(bsp_uart2.init() == Status::OK);
  configASSERT(bsp_uart3.init() == Status::OK);
  configASSERT(bsp_uart4.init() == Status::OK);
  configASSERT(bsp_uart7.init() == Status::OK);
  configASSERT(bsp_uart8.init() == Status::OK);
  configASSERT(bsp_uart9.init() == Status::OK);
  configASSERT(bsp_uart10.init() == Status::OK);
  configASSERT(bsp_uart_it5.init() == Status::OK);

  // PWM 与按键
  configASSERT(bsp_pwm1.init() == Status::OK);
  configASSERT(bsp_pwm2.init() == Status::OK);
  configASSERT(bsp_pwm3.init() == Status::OK);
  configASSERT(bsp_pwm4.init() == Status::OK);
  configASSERT(bsp_pwm_gyro.init() == Status::OK);
  configASSERT(bsp_pwm_buzzer.init() == Status::OK);
  configASSERT(key_user.init() == Status::OK);

  // 设备：必须在对应外设之后（注册 CAN 节点 / 用已初始化的串口）
  // configASSERT(r9ds.init() == Status::OK); // R9DS 暂时不用
  configASSERT(emm_group.init() == Status::OK);
  configASSERT(servo1.init() == Status::OK);
  configASSERT(servo2.init() == Status::OK);
  configASSERT(servo3.init() == Status::OK);
  configASSERT(servo4.init() == Status::OK);
}
