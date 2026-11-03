/**
 * @file corexy_link.hpp
 * @brief CoreXY 三源入口（终端 / 二进制 / 遥控）：命令归一、源仲裁、上报
 *
 * @details
 * 【它是什么】
 *   一条命令队列 + 三个输入源 + 按来源格式化应答。`corexy_io_task` 从这里取命令执行，
 *   其它任务只往里投递，不碰电机总线。
 *
 * 【三个源】
 *   | 源 | 入口 | 端口 | 说明 |
 *   | --- | --- | --- | --- |
 *   | `Console` | `feed_console()` | `bsp_uart1`（USART1，115200） | ASCII 行命令，行尾 CR / LF / `;` |
 *   | `Binary`  | `feed_binary()`  | `bsp_uart7`（UART7，921600） | 帧命令，**格式待定**（当前 A5 5A 帧） |
 *   | `Rc`      | `submit()`       | 无 | app 任务直接投递 |
 *
 * 【最短用法】
 *   @code{.cpp}
 *   // 1) 上电：module_cfg.cpp 已实例化 corexy_link，module_init() 里已 init()，这里不用管
 *
 *   // 2) 终端（默认源，USART1 115200 8N1，行尾 CR / LF / ;）
 *   //    首次必须 home：清零并把当前位置记为原点；之后 update() 连续 2 拍读到 (0,0) 才算复核通过，
 *   //    未复核前 m / j 一律返回 NOT_INIT。
 *   home
 *   en 1                  // 使能两台（en 0 失能）
 *   spd 500 250           // 基准 rpm（1..5000）/ acc（0..255）；两轴按脉冲比例分摊，长轴 = 设定值
 *   m 100 50              // 绝对移动 (100,50) mm：先 45° 到 (50,50)，到位后再走轴向
 *   j 10 0                // 相对当前位置 +10 mm
 *   g 2 20                // 八方向单步：0=+X 1=+X+Y 2=+Y 3=-X+Y 4=-X 5=-X-Y 6=-Y 7=+X-Y
 *   pos                   // 打印 pos / c1 / c2 / state / seg / err / origin
 *   lim                   // 打印理论限位与安全限位
 *   dump                  // 打印最近一帧下发内容与两台在线状态
 *   stop                  // 立即停（逐台 + 广播）；进 Fault 后只有它能清除
 *   qp 1 rel 500 250      // 诊断：单台快速位置参数（0xF1，绕过模块）
 *   q 1 1600              // 诊断：单台 +1600 脉冲（80 脉冲/mm → 20 mm）
 *   src 1                 // 切源：0 终端 / 1 二进制 / 2 遥控（只有当前活动源能切）
 *   //    回显：每条命令回一行 `[CXY] cmd 0x02 -> st=0`；移动结束另报 `[CXY] done ...`，
 *   //    越界 / 丢命令报 `[CXY] fault st=<原因>`（st 取值见 Status）。
 *
 *   // 3) 遥控（app 任务里）
 *   // 3.1 先启用 + 切源，否则 submit() 一律 BAD_ARG
 *   corexy_link.set_enabled(CxySource::Rc, true);
 *   corexy_link.select(CxySource::Rc);
 *   // 3.2 投递命令：不阻塞，进 8 深队列，由 corexy_io_task 执行
 *   corexy_link.submit(cxy_cmd::move_abs(CxySource::Rc, 50.0f, 30.0f)); // 绝对 mm
 *   corexy_link.submit(cxy_cmd::move_rel(CxySource::Rc, 0.0f, -5.0f));  // 相对位移
 *   corexy_link.submit(cxy_cmd::enable(CxySource::Rc, true));
 *   corexy_link.submit(cxy_cmd::speed(CxySource::Rc, 500, 250));
 *   corexy_link.submit(cxy_cmd::zero(CxySource::Rc));                   // 原点 = 当前位置（需复核）
 *   corexy_link.submit(cxy_cmd::stop(CxySource::Rc));
 *   // 3.3 返回值：OK / BAD_ARG（非活动源或未启用）/ FULL（队列满，本次丢弃）
 *   // 3.4 Rc 不回包，需要状态就自查：
 *   const CorexyModule::Point p = corexy.position();
 *   const bool busy = (corexy.state() == CorexyModule::State::Moving);
 *   // 3.5 建议把 CMD_TIMEOUT_MS 设成 200 ms：遥控掉线时 io 任务自动 stop()。
 *
 *   // 4) 运行期开关（不删任务，只决定「解析 / 接收」）
 *   corexy_link.set_enabled(CxySource::Console, false);
 *   //    - 关掉的源：feed_* 只丢字节不解析，submit() 拒绝，也不能 select() 过去
 *   //    - 关的是当前活动源 → active 退回 None、队列清空，io 任务下一拍 stop()
 *   //    - 重新打开不会自动成为活动源，要显式 select()
 *   corexy_link.set_enabled(CxySource::Console, true);
 *   corexy_link.select(CxySource::Console);
 *   @endcode
 *
 * 【约束】
 *   - `take()` 只允许 `corexy_io_task` 调（队列唯一消费者）。
 *   - `submit()` 队列满返回 `FULL`；源不匹配 / 未启用返回 `BAD_ARG`，调用方自行决定重试。
 *   - 本层不碰电机、不阻塞等电机，只做“字节 ↔ 命令”的转换与格式化。
 *   - 可调项（上电默认开关 / 默认源 / 超时 / 队列深度）在下面的 `cxy_link` 命名空间。
 */

#ifndef __MODULE_COREXY_LINK_HPP__
#define __MODULE_COREXY_LINK_HPP__

#include "bsp_uart.hpp"
#include "status.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "queue.h"    // IWYU pragma: keep
#include "task.h"     // IWYU pragma: keep

#include <stddef.h>
#include <stdint.h>


/* ==================== 类型 ==================== */

/** @brief 命令来源（也是仲裁单位） */
enum class CxySource : uint8_t
{
  None,    ///< 无（初始 / 关闭）
  Console, ///< 串口终端
  Binary,  ///< 串口二进制
  Rc,      ///< 遥控（app 任务直接调用）
};

/** @brief 归一化命令：三个源都产出这个结构 */
struct CxyCmd
{
  /** @brief 命令类型；前 9 个与二进制命令码一一对应 */
  enum class Type : uint8_t
  {
    Enable     = 0x01, ///< p0 = 1 使能 / 0 失能
    MoveAbs    = 0x02, ///< a,b = 目标 X/Y（mm）
    MoveRel    = 0x03, ///< a,b = 增量 X/Y（mm）
    Zero       = 0x04, ///< 当前位置记为原点
    Stop       = 0x05, ///< 立即停止并清故障
    SetSpeed   = 0x06, ///< p0 = rpm，p1 = acc
    QuickParam = 0x07, ///< addr，p0 = rpm，p1 = acc，mode = 0 相对 / 1 绝对
    QuickPos   = 0x08, ///< addr，p0 = 脉冲
    Limits     = 0x09, ///< 查询理论 / 安全限位
    Status     = 0x0A, ///< 查位置与状态（仅终端）
    Dump       = 0x0B, ///< 打印诊断（仅终端）
  };

  Type      type = Type::Stop;
  float     a    = 0.0f;            ///< X（mm）
  float     b    = 0.0f;            ///< Y（mm）
  int32_t   p0   = 0;               ///< 开关 / rpm / 脉冲
  int32_t   p1   = 0;               ///< 加速度
  uint8_t   addr = 0U;              ///< 电机地址
  uint8_t   mode = 0U;              ///< 快速位置：0 相对 / 1 绝对
  CxySource from = CxySource::None; ///< 来源，决定应答回哪
};

/** @brief 命令构造辅助（遥控侧直接拼一条命令时用，避免到处写字段） */
namespace cxy_cmd
{
inline CxyCmd move_abs(CxySource from, float x, float y)
{
  return CxyCmd {CxyCmd::Type::MoveAbs, x, y, 0, 0, 0U, 0U, from};
}

inline CxyCmd move_rel(CxySource from, float dx, float dy)
{
  return CxyCmd {CxyCmd::Type::MoveRel, dx, dy, 0, 0, 0U, 0U, from};
}

inline CxyCmd enable(CxySource from, bool on)
{
  return CxyCmd {CxyCmd::Type::Enable, 0.0f, 0.0f, on ? 1 : 0, 0, 0U, 0U, from};
}

inline CxyCmd stop(CxySource from)
{
  return CxyCmd {CxyCmd::Type::Stop, 0.0f, 0.0f, 0, 0, 0U, 0U, from};
}

inline CxyCmd speed(CxySource from, uint16_t rpm, uint8_t acc)
{
  return CxyCmd {CxyCmd::Type::SetSpeed, 0.0f, 0.0f, static_cast<int32_t>(rpm), static_cast<int32_t>(acc), 0U, 0U, from};
}

inline CxyCmd zero(CxySource from)
{
  return CxyCmd {CxyCmd::Type::Zero, 0.0f, 0.0f, 0, 0, 0U, 0U, from};
}
} // namespace cxy_cmd

/* ==================== */


/* ==================== 配置 ==================== */

/** @brief 三源入口的可调项（改这里就够，不另建 cfg 文件） */
namespace cxy_link
{
// 下面三个开关只是**上电默认值**，运行期用 `CorexyLink::set_enabled()` 改
constexpr bool      CONSOLE_ENABLE = true;               ///< 终端口上电默认启用
constexpr bool      BINARY_ENABLE  = false;              ///< 二进制口上电默认启用
constexpr bool      RC_ENABLE      = false;              ///< 遥控源上电默认启用
constexpr CxySource DEFAULT_SOURCE = CxySource::Console; ///< 上电默认生效的源（默认源必须已启用，否则为 None）
constexpr uint32_t  CMD_TIMEOUT_MS = 0U;                 ///< 活动源无指令超时（0 = 关闭自动停机）
constexpr uint16_t  QUEUE_DEPTH    = 8U;                 ///< 命令队列深度
constexpr size_t    LINE_MAX       = 64U;                ///< 终端单行上限
constexpr size_t    PAYLOAD_MAX    = 24U;                ///< 二进制请求负载上限
} // namespace cxy_link

/* ==================== */

/**
 * @brief 三源入口：收字节 → 归一化命令 → 队列；以及按来源回格式化的应答
 *
 * @note 两个串口各带一套解析状态（`_line` / `_rx`），互不共享；非活动源只丢不解析。
 * @note 报文格式待定的二进制帧集中在 `parse_binary_frame()`，改协议只动那一处。
 */
class CorexyLink
{
public:
  /* ==================== 类型 ==================== */

  ///< 本工程串口实例类型（DMA 收发、128/128 缓冲）
  using Uart = BspUartT<uart_mode::dma, 128, 128>;

  /** @brief 端口绑定（不拥有） */
  struct Config
  {
    Uart    *console        = nullptr;                  ///< 终端口
    Uart    *binary         = nullptr;                  ///< 二进制口
    uint32_t cmd_timeout_ms = cxy_link::CMD_TIMEOUT_MS; ///< 0 = 不自动停机
  };

  /* ==================== */
  /* ==================== 构造与仲裁 ==================== */

  /** @brief 构造函数（只做赋值） */
  explicit CorexyLink(const Config &cfg);

  /**
   * @brief 建命令队列，按 `cxy_link::DEFAULT_SOURCE` 选中初始源
   * @return OK / IO_ERROR（队列创建失败）
   */
  Status init();

  /**
   * @brief 切换活动源：清空队列、复位两套解析状态
   * @note 运动中的停机由 `corexy_io_task` 检测到源变化后执行（这里不碰电机）
   */
  Status select(CxySource s);

  /** @brief 当前活动源 */
  CxySource active() const
  {
    return _active;
  }

  /**
   * @brief 运行期启用 / 关闭某个源（app 层可随时调）
   * @note 关掉当前活动源时：活动源退回 `None`、清队列、复位解析器；
   *       `corexy_io_task` 检测到源变化后会 `stop()`。
   * @return OK / BAD_ARG（源非法，或启用但对应端口未绑）
   */
  Status set_enabled(CxySource s, bool on);

  ///< 该源当前是否启用
  bool enabled(CxySource s) const;

  /* ==================== */
  /* ==================== 命令通道 ==================== */

  /**
   * @brief 投递一条命令（源不匹配直接拒绝）
   * @return OK / BAD_ARG（不是活动源或未 init）/ FULL（队列满）
   */
  Status submit(const CxyCmd &cmd);

  ///< io 任务取一条命令；没有则返回 false
  bool take(CxyCmd &out);

  ///< 活动源是否已超过 `cmd_timeout_ms` 没有新命令（0 = 永不超时）
  bool timed_out() const;

  ///< 超时停机后重新计时，避免重复停机
  void reset_timeout();

  /* ==================== */
  /* ==================== 串口侧入口 ==================== */

  ///< 终端口：按行解析，行结束符 CR / LF / `;`
  void feed_console(const uint8_t *data, size_t size);

  /**
   * @brief 二进制口：按帧解析
   * @note **帧格式待定**，当前沿用 `A5 5A | CMD | LEN | PAYLOAD | SUM`；
   *       约定好后只改 `parse_binary_frame()` 与 `reply()` / `event()`。
   */
  void feed_binary(const uint8_t *data, size_t size);

  /* ==================== */
  /* ==================== 上报 ==================== */

  ///< 回一条命令的结果：Binary 发应答帧，Console 打一行，Rc 不回
  void reply(const CxyCmd &cmd, Status st, float x, float y);

  ///< 限位应答（只有 Binary 需要负载，Console 打一行）
  void reply_limits(Status st, const float theory[4], const float safe[4]);

  ///< 事件（BOOT / DONE / FAULT）
  void event(uint8_t code, Status st, float x, float y);

  ///< 直接往终端口打印（帮助、诊断等）
  void print_console(const char *text);

  /* ==================== */
  /* ==================== 端口 ==================== */

  Uart *console_port() const
  {
    return _cfg.console;
  }

  Uart *binary_port() const
  {
    return _cfg.binary;
  }

  /* ==================== */
private:
  /* ==================== 解析实现 ==================== */

  ///< 一行终端命令（已去掉行尾）
  void parse_line(const char *text);

  ///< 一帧完整二进制请求
  void parse_binary_frame(uint8_t cmd, const uint8_t *payload, size_t len);

  ///< 切源后复位解析状态
  void reset_parsers();

  ///< 终端提示
  void print_help();

  ///< 命令码（二进制用）
  static uint8_t code_of(CxyCmd::Type t)
  {
    return static_cast<uint8_t>(t);
  }

  ///< 源 → `_enabled` 下标（None 返回 3）
  static uint8_t slot_of(CxySource s)
  {
    return static_cast<uint8_t>(s);
  }

  /* ==================== */
  /* ==================== 成员 ==================== */

  Config        _cfg;                        ///< 端口绑定
  QueueHandle_t _queue    = nullptr;         ///< 命令队列
  CxySource     _active   = CxySource::None; ///< 当前活动源
  TickType_t    _last_cmd = 0;               ///< 最近一次收到命令的时刻（超时用）
  bool          _enabled[4] = {false, false, false, false}; ///< 按 CxySource 下标（None 占 0）

  char   _line[cxy_link::LINE_MAX] = {}; ///< 终端行缓冲
  size_t _line_len                 = 0U;

  uint8_t _rx[cxy_link::PAYLOAD_MAX + 5U] = {}; ///< 二进制帧缓冲（头2+CMD+LEN+负载+SUM）
  size_t  _rx_len                         = 0U;

  /* ==================== */
};

#endif // __MODULE_COREXY_LINK_HPP__
