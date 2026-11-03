/**
 * @file remote_ctrl_task.cpp
 * @author Rh
 * @brief 遥控任务占位：把遥控器数据翻译成 `CxyCmd` 后交给 `corexy_link`
 *
 * @note 只有活动源是 `Rc` 时 `submit()` 才接受；否则返回 `BAD_ARG`，可直接忽略。
 * @note 接入遥控器时只改本文件的采样与映射，`corexy` / `corexy_link` 都不用动。
 */

#include "app_task.hpp"

#include "corexy_link.hpp"
#include "module_cfg.hpp"
#include "system.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"

#include <stdint.h>


namespace
{
constexpr uint32_t PERIOD_MS = 20U; ///< 遥控采样周期
}

extern "C" void remote_ctrl_task(void *argument)
{
  (void)argument;
  sys_state.wait_running();

  for (;;)
  {
    // 示例：活动源为 Rc 时下发一条绝对移动（接入遥控器后按实际摇杆值替换）
    // (void)corexy_link.submit(cxy_cmd::move_abs(CxySource::Rc, x_mm, y_mm));

    vTaskDelay(pdMS_TO_TICKS(PERIOD_MS));
  }
}
