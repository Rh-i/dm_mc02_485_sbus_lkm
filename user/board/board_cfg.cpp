/**
 * @file board_cfg.cpp
 * @brief board_init()：把 board_cfg.hpp 里的实例逐个 init()（dm_mc02_485_sbus_lkm）
 *
 * @details 旧工程的 `bsp_init()` + `device_init()` 合并到这里：
 *          CAN → UART → SPI → PWM/GPIO/按键 → 设备（Emm_V5 / 舵机）。
 *          计时基准 `bsp_dwt` 已由库的 `system_init()` 初始化，这里不再处理。
 *          设备要放在对应外设 init() 之后（它们要注册 CanRxNode / CanTxNode 或用已初始化的串口）。
 *
 *          【失败就地停机】每行 configASSERT，fault_record 会记下行号，print_uart 能打出来。
 */

#include "board_cfg.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep (configASSERT)


void board_init(void)
{
  // 计时基准bsp_dwt已经被库内定义，并且由 system_init()内部 init()，这里不再重复初始化

  // CAN
  configASSERT(bsp_can1.init() == Status::OK);
  configASSERT(bsp_can2.init() == Status::OK);
  configASSERT(bsp_can3.init() == Status::OK);

  // 串口
  configASSERT(bsp_uart1.init() == Status::OK);
  configASSERT(bsp_uart2.init() == Status::OK);
  configASSERT(bsp_uart3.init() == Status::OK);
  configASSERT(bsp_uart4.init() == Status::OK);
  configASSERT(bsp_uart7.init() == Status::OK);
  configASSERT(bsp_uart8.init() == Status::OK);
  configASSERT(bsp_uart9.init() == Status::OK);
  configASSERT(bsp_uart10.init() == Status::OK);
  configASSERT(bsp_uart_it5.init() == Status::OK);

  // SPI
  configASSERT(bsp_spi1.init() == Status::OK);

  // 陀螺仪 SPI 暂不启用（启用时按 总线 → 两个片选 → 设备 的顺序取消注释）
  // configASSERT(bsp_spi2.init() == Status::OK);
  // configASSERT(bmi088_acc_spi.init() == Status::OK);
  // configASSERT(bmi088_gyro_spi.init() == Status::OK);
  // configASSERT(imu_bmi088.init() == Status::OK);

  // PWM与按键
  configASSERT(bsp_pwm1.init() == Status::OK);
  configASSERT(bsp_pwm2.init() == Status::OK);
  configASSERT(bsp_pwm3.init() == Status::OK);
  configASSERT(bsp_pwm4.init() == Status::OK);
  configASSERT(bsp_pwm_gyro.init() == Status::OK);
  configASSERT(bsp_pwm_buzzer.init() == Status::OK);
  configASSERT(key_user.init() == Status::OK);

  // 设备：必须在对应外设之后
  // configASSERT(r9ds.init() == Status::OK); // R9DS 暂时不用
  configASSERT(emm_group.init() == Status::OK);
  configASSERT(servo1.init() == Status::OK);
  configASSERT(servo2.init() == Status::OK);
  configASSERT(servo3.init() == Status::OK);
  configASSERT(servo4.init() == Status::OK);
}
