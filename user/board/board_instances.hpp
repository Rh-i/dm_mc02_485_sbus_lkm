/**
 * @file board_instances.hpp
 * @brief 板级实例清单（dm_mc02_485_sbus_lkm）：库用 X-macro 按这里的名字展开
 *
 * @details 库没有运行时登记表：HAL 回调、`sys_task` 巡检、`can_rx` / `can_tx` 都靠这组宏
 *          在编译期知道「板子上有哪几个实例」。只写名字，类型由 `decltype` 推。
 *
 *          【加实例的完整步骤】
 *            1. `board_cfg.hpp` 里定义 `inline` 实例
 *            2. 本文件对应宏里加一项（续行用反斜杠，**最后一项后面不加逗号**）
 *            3. `board_cfg.cpp` 的 `board_init()` 里加 `configASSERT`
 *            4. CubeMX 里把对应中断打开
 *
 *          【忘了同步不会报错】：那个实例的回调不会被分发、`sys_task` 也不巡检它，
 *          表现是「数据收不到」而不是「编译失败」。
 */

#ifndef __BOARD_INSTANCES_HPP__
#define __BOARD_INSTANCES_HPP__

#include "board_cfg.hpp" // IWYU pragma: keep

/**
 * @brief CAN 实例列表（3 路全用，CAN3 挂 DJI 电机）
 *
 * 列了几个就要求每个都已 `init()`：`sys_task` 的 Bus-Off 恢复和 `can_rx_task` 的分发
 * 都会直接访问它们。
 */
#define BSP_CAN_INSTANCES(X) \
  X(bsp_can1)                \
  X(bsp_can2)                \
  X(bsp_can3)

/**
 * @brief UART 实例列表
 *
 * 用途：① HAL 的 UART 回调按句柄找到对应实例；② `sys_task` 每 10 ms 巡检收发健康度。
 * `bsp_uart_it5`（UART5 / SBUS）虽然是只收，也要列进来——巡检看的是「有没有断流」。
 */
#define BSP_UART_INSTANCES(X) \
  X(bsp_uart1)                \
  X(bsp_uart2)                \
  X(bsp_uart3)                \
  X(bsp_uart4)                \
  X(bsp_uart7)                \
  X(bsp_uart8)                \
  X(bsp_uart9)                \
  X(bsp_uart10)               \
  X(bsp_uart_it5)

/**
 * @brief SPI 实例列表
 *
 * `bsp_spi1`：原 LCD 接口（TXONLY、纯轮询）。
 * `bsp_spi2`：陀螺仪 BMI088 总线；当前**没有** `init()`，列在这里只是为了让库的
 *             HAL SPI 回调能按句柄找到它（不 init 时回调本身是安全的空操作）。
 */
#define BSP_SPI_INSTANCES(X) \
  X(bsp_spi1)                \
  X(bsp_spi2)

#endif // __BOARD_INSTANCES_HPP__
