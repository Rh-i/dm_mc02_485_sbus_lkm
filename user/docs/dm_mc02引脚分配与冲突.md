# 引脚分配与冲突

## 本工程采用的引脚冲突分配方案

| 引脚      | 冲突双方                      | 采用   | 放弃 |
| --------- | ----------------------------- | ------ | ---- |
| PB8 / PB9 | UART4 RX/TX ↔ I2C1 SCL/SDA   | UART4  | I2C1 |
| PD8 / PD9 | USART3（RS485-3）↔ SPI3 复用 | USART3 | SPI3 |

> 结论：**RS485-3（USART3）与 UART5 SBUS 同时使用**，SPI3 不启用；UART4 占用 PB8/PB9，因此 I2C1 不可复用。
> 保留 SWD（PA13/PA14）后 JTAG 关闭，PA15 才能当普通按键；同理 SPI1 占用 PB3，SWO 不可用。

## 当前启用情况

以 `user/board/board_cfg.cpp` 的 `board_init()` 与 `user/app/` 的实际使用为准。

| 类别        | 实例                                                                                                        | 状态             | 用途                                     |
| ----------- | ----------------------------------------------------------------------------------------------------------- | ---------------- | ---------------------------------------- |
| 计时        | `bsp_dwt`                                                                                                 | 已启用（库内初始化） | 内核 CYCCNT 计时基准，实例与 `init()` 都在 `system_init()` 里 |
| 串口        | `bsp_uart1`                                                                                               | 已启用           | USART1：控制台输入 + `print_uart` 打印 |
| 串口        | `bsp_uart2`                                                                                               | 已启用           | RS485-2，Emm_V5 电机组（地址 1/2/3）     |
| 串口        | `bsp_uart3`、`bsp_uart4`、`bsp_uart7`、`bsp_uart8`、`bsp_uart9`、`bsp_uart10`、`bsp_uart_it5` | 仅初始化         |                                          |
| PWM         | `bsp_pwm1~4` + `servo1~4`                                                                               | 仅初始化         |                                          |
| PWM         | `bsp_pwm_buzzer` + `buzzer`                                                                             | 已启用           | `key_task` 提示音                      |
| PWM         | `bsp_pwm_gyro`                                                                                            | 仅初始化         | 陀螺仪温控                               |
| GPIO / 按键 | `power_24v_1/2`、`power_5v`、`btb_pa5/pe14/pd10`、`spi1_cs`、`key_user`                           | 已实例化         | `key_user` 由 `key_task` 使用        |
| SPI         | `bsp_spi1`                                                                                                | 仅初始化         |                                          |
| SPI         | `bsp_spi2` + `bmi088_*` + `imu_bmi088`                                                                | **未启用** |                                          |
| CAN         | `bsp_can1/2/3`                                                                                            | 仅初始化         |                                          |
| 设备        | `r9ds`（UART5 SBUS）                                                                                      | **未启用** |                                          |

> 本版实际在用的只有：1 路 RS485（Emm_V5）、1 路调试/命令串口、按键 + 蜂鸣器、4 路舵机 PWM。
> CAN 与陀螺仪都没有接入设备。

## 已配置但未实例化的外设

| 外设               | 引脚                      | CubeMX                                 | 代码   | 原因                     |
| ------------------ | ------------------------- | -------------------------------------- | ------ | ------------------------ |
| I2C2               | PB10 / PB11               | 已初始化（`MX_I2C2_Init`）           | 无实例 | 库暂无 `bsp_i2c` 驱动  |
| OCTOSPI2（W25Q64） | PB2/PE11/PD11/PB0/PA3/PA1 | 已初始化（`MX_OCTOSPI2_Init`）       | 无实例 | 库暂无 `bsp_ospi` 驱动 |
| USB_OTG_HS         | PA11 / PA12               | 已初始化（`MX_USB_OTG_HS_PCD_Init`） | 无实例 | 库暂无 USB 驱动          |

> 这三个外设的引脚与时钟已由 CubeMX 配好，句柄分别是 `hi2c2`、`hospi2`、`hpcd_USB_OTG_HS`，
> 后续加驱动时直接在 `board_cfg.hpp` 里绑句柄、在 `board_instances.hpp` 里登记即可。
