/**
 * @file corexy_rx_console.cpp
 * @author Rh
 * @brief 终端口收包任务：读 USART1 → `corexy_link.feed_console()`
 *
 * @note 只搬字节：解析与投递都在 `corexy_link`；非活动源时字节照读、只丢不解析。
 */

#include "app_task.hpp"

#include "corexy_link.hpp"
#include "module_cfg.hpp"
#include "system.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"

#include <stddef.h>
#include <stdint.h>


extern "C" void corexy_rx_console(void *argument)
{
  (void)argument;
  sys_state.wait_running();

  uint8_t buf[32];

  for (;;)
  {
    size_t n = 0U;

    if ((corexy_link.console_port()->receive(buf, sizeof(buf), 10U, &n) == Status::OK) && (n > 0U))
    {
      corexy_link.feed_console(buf, n);
    }
  }
}
