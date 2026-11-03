/**
 * @file corexy_io_task.cpp
 * @author Rh
 * @brief CoreXY 执行任务：独占电机总线，执行队列命令并按来源上报
 *
 * @note 上电：清零 → 使能 → 复核原点 → 上报 BOOT；之后取队列命令执行，再 10 ms 推一拍状态机。
 * @note 上报格式由 `corexy_link` 按命令来源决定（终端 / 二进制 / 遥控不回）。
 * @note 切源与活动源超时都在这里统一处理：先 `stop()` 再继续，避免两台抢命令。
 */

#include "app_task.hpp"

#include "board_cfg.hpp" // emm_group（诊断命令用）
#include "corexy_link.hpp"
#include "module_cfg.hpp"
#include "system.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"

#include <stdint.h>


namespace
{
///< 事件码（与二进制协议一致）
constexpr uint8_t EVT_BOOT  = 0x01U;
constexpr uint8_t EVT_DONE  = 0x02U;
constexpr uint8_t EVT_FAULT = 0x03U;

///< 位置与状态一行（终端用）
void print_status()
{
  const CorexyModule::Point p = corexy.position();

  corexy_link.console_port()->printf("[CXY] pos=(%.2f, %.2f) c1=%ld c2=%ld state=%u seg=%u/%u err=%u origin=%u\r\n", p.x, p.y,
                                     static_cast<long>(corexy.raw_counts(0U)), static_cast<long>(corexy.raw_counts(1U)),
                                     static_cast<unsigned>(corexy.state()), static_cast<unsigned>(corexy.segment_index()),
                                     static_cast<unsigned>(corexy.segment_count()), static_cast<unsigned>(corexy.last_error()),
                                     corexy.origin_verified() ? 1U : 0U);
}

///< 最近一帧下发内容与两台在线状态（诊断“命令到底发出去没”）
void print_dump()
{
  size_t         len = 0U;
  const uint8_t *f   = emm_group.last_multi_frame(len);

  corexy_link.console_port()->printf("[CXY] mmcl(%u):", static_cast<unsigned>(len));

  for (size_t i = 0U; i < len; i++)
  {
    corexy_link.console_port()->printf(" %02X", f[i]);
  }

  corexy_link.console_port()->printf("\r\n[CXY] online 1=%u 2=%u\r\n", emm_group.is_online(1U) ? 1U : 0U,
                                     emm_group.is_online(2U) ? 1U : 0U);
}

///< 执行一条命令
void run(const CxyCmd &cmd)
{
  Status st = Status::OK;

  switch (cmd.type)
  {
    case CxyCmd::Type::Enable:
      st = corexy.enable(cmd.p0 != 0); // 静默 + 两台 + 帧间隔都在模块里
      break;

    case CxyCmd::Type::MoveAbs:
      st = corexy.move_to(CorexyModule::Point(cmd.a, cmd.b));
      break;

    case CxyCmd::Type::MoveRel:
      st = corexy.move_by(CorexyModule::Point(cmd.a, cmd.b));
      break;

    case CxyCmd::Type::Zero:
      st = corexy.zero_here();
      break;

    case CxyCmd::Type::Stop:
      st = corexy.stop();
      break;

    case CxyCmd::Type::SetSpeed:
      st = corexy.set_speed(static_cast<uint16_t>(cmd.p0), static_cast<uint8_t>(cmd.p1));
      break;

    case CxyCmd::Type::QuickParam:
      // 单电机诊断帧：绕过模块直发驱动器
      st = emm_group.set_quick_position_params(cmd.addr, static_cast<uint16_t>(cmd.p0), static_cast<uint8_t>(cmd.p1),
                                               (cmd.mode != 0U) ? emmv5::PositionMode::Absolute : emmv5::PositionMode::Relative);
      break;

    case CxyCmd::Type::QuickPos:
      st = emm_group.set_quick_position(cmd.addr, cmd.p0);
      break;

    case CxyCmd::Type::Limits:
    {
      const CorexyModule::Rect th    = corexy.limits_theory();
      const CorexyModule::Rect sf    = corexy.limits_safe();
      const float              t[4]  = {th.x0, th.y0, th.x1, th.y1};
      const float              s[4]  = {sf.x0, sf.y0, sf.x1, sf.y1};
      corexy_link.reply_limits(Status::OK, t, s);
      return;
    }

    case CxyCmd::Type::Status:
      print_status();
      return;

    case CxyCmd::Type::Dump:
      print_dump();
      return;
  }

  const CorexyModule::Point p = corexy.position();
  corexy_link.reply(cmd, st, p.x, p.y);
}

} // namespace


extern "C" void corexy_io_task(void *argument)
{
  (void)argument;
  sys_state.wait_running();
  vTaskDelay(pdMS_TO_TICKS(500U));

  // 上电：清零 → 使能 → 等几个轮询周期复核原点
  (void)corexy.zero_here();
  (void)corexy.enable(true);

  for (uint32_t i = 0U; i < 30U; i++)
  {
    corexy.update(10U);
  }

  corexy_link.event(EVT_BOOT, corexy.origin_verified() ? Status::OK : Status::NOT_INIT, corexy.position().x, corexy.position().y);

  CxySource           last_src = corexy_link.active();
  CorexyModule::State last     = corexy.state();

  for (;;)
  {
    // 切源：旧源的运动不能继续；超时：活动源没新指令就停（0 = 关闭）
    if (corexy_link.active() != last_src)
    {
      (void)corexy.stop();
      last_src = corexy_link.active();
    }

    if (corexy_link.timed_out())
    {
      (void)corexy.stop();
      corexy_link.reset_timeout();
    }

    CxyCmd cmd;

    while (corexy_link.take(cmd))
    {
      run(cmd);
    }

    corexy.update(10U); // 收字节 + 推进状态机（内部等最多 10 ms）

    const CorexyModule::State now = corexy.state();

    if (now != last)
    {
      const CorexyModule::Point p = corexy.position();

      if (now == CorexyModule::State::Fault)
      {
        corexy_link.event(EVT_FAULT, corexy.last_error(), p.x, p.y);
      }
      else if ((now == CorexyModule::State::Idle) && (last == CorexyModule::State::Moving))
      {
        corexy_link.event(EVT_DONE, Status::OK, p.x, p.y);
      }

      last = now;
    }
  }
}
