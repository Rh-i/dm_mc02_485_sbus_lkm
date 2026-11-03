/**
 * @file corexy_rx_binary.cpp
 * @author Rh
 * @brief 二进制口收包任务：读 UART7 → `corexy_link.feed_binary()`
 *
 * @note 帧格式由 `corexy_link` 的 `parse_binary_frame()` 决定（当前沿用 A5 5A 帧，待定稿）。
 * @note 非活动源时字节照读、只丢不解析，防止接收缓冲溢出。
 */

#include "app_task.hpp"

#include "corexy_link.hpp"
#include "module_cfg.hpp"
#include "system.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"

#include <stddef.h>
#include <stdint.h>


extern "C" void corexy_rx_binary(void *argument)
{
  (void)argument;
  sys_state.wait_running();

  uint8_t buf[32];

  for (;;)
  {
    size_t n = 0U;

    if ((corexy_link.binary_port()->receive(buf, sizeof(buf), 10U, &n) == Status::OK) && (n > 0U))
    {
      corexy_link.feed_binary(buf, n);
    }
  }
}
