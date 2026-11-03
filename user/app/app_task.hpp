/**
 * @file app_task.hpp
 * @author Rh
 * @brief 应用层任务声明 —— 除默认任务外的业务任务
 * @version 0.1
 * @date 2026-08-14
 *
 * @copyright Copyright (c) 2026
 *
 * @note 任务函数均以 extern "C" 声明（FreeRTOS 以 C 方式调用）。
 *       任务实现位于 User/App/task/，由 app_init() 统一创建。
 */

#ifndef __APP_TASK_HPP__
#define __APP_TASK_HPP__

/**
 * @brief CoreXY 任务：两个收包口（终端 / 二进制）、一个执行任务、一个遥控占位
 *
 * @note 三个输入源共用 `corexy_link` 的命令队列，同一时刻只有活动源有效；
 *       开关与默认源在 `module/corexy_link.hpp` 的 `cxy_link` 命名空间里。
 * @note 执行侧（`corexy_io_task`）与上报也跟随活动源切换。
 */


#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @brief 按键任务：200 ms 轮询 key_user，按事件驱动蜂鸣器提示
   * @param argument 任务参数（未使用，NULL）
   * @note 轮询周期与 key_user 的消抖 / 长按配置对应（debounce 1 → 200 ms，
   *       long_press 5 → 1 s）；beep 为阻塞调用，实际周期会多出一个响铃时长。
   * @note 由 app_init() 创建，不应由业务代码直接调用。
   */
  void key_task(void *argument);

  /**
   * @brief 终端口收包任务：读终端串口 → `corexy_link.feed_console()`
   * @param argument 任务参数（未使用，NULL）
   * @note 只搬字节；行解析、源仲裁、投递都在 `corexy_link`。
   * @note 由 app_init() 创建，不应由业务代码直接调用。
   */
  void corexy_rx_console(void *argument);

  /**
   * @brief 二进制口收包任务：读二进制串口 → `corexy_link.feed_binary()`
   * @param argument 任务参数（未使用，NULL）
   * @note 帧格式由 `corexy_link` 统一处理（当前沿用 A5 5A 帧，待定稿）。
   * @note 由 app_init() 创建，不应由业务代码直接调用。
   */
  void corexy_rx_binary(void *argument);

  /**
   * @brief CoreXY 执行任务：独占电机总线，取队列命令执行并按来源上报
   * @param argument 任务参数（未使用，NULL）
   * @note 上电完成停止/清零/使能 + 原点复核，之后 10 ms 循环：取命令 + `corexy.update()` +
   *       状态变化上报。切源与活动源超时也在这里停机。
   * @note 本任务同时负责 `emm_group.update()`（这条总线唯一的接收消费者）。
   * @note 由 app_init() 创建，不应由业务代码直接调用。
   */
  void corexy_io_task(void *argument);

  /**
   * @brief 遥控任务：把遥控器数据翻成 `CxyCmd` 交给 `corexy_link`
   * @param argument 任务参数（未使用，NULL）
   * @note 当前为占位；只有活动源是 `Rc` 时 `submit()` 才接受。
   */
  void remote_ctrl_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif // __APP_TASK_HPP__
