#include "app_main.h"
#include "FreeRTOS.h" // IWYU pragma: keep
#include "main.h"     // IWYU pragma: keep
#include "stdio.h"
#include "task.h"


/* BSP */
#include "board_cfg.hpp"

/* Device */
#include "board_cfg.hpp" // IWYU pragma: keep

/* Module */
#include "module_cfg.hpp"

/* Service */
#include "system.hpp"

/* 任务声明 */
#include "app_task.hpp"

/* CoreXY 任务间命令队列 */
#include "corexy_protocol.hpp"

/* 测试任务声明（开关见 app_test.hpp） */
#include "app_test.hpp" // IWYU pragma: keep


/**
 * @brief FreeRTOS后的相关初始化
 *
 * @note 在main.c的MX_FREERTOS_INIT函数中调用
 *       用于创建FreeRTOS任务和初始化外设驱动
 *
 *       初始化顺序：
 *         1. service_init()  —— Service 层（系统状态标志）
 *         2. bsp_init()      —— BSP 层外设初始化
 *         3. device_init()   —— 设备层初始化
 *         4. 创建各任务      —— 任务入口先等运行态
 *         5. sys_state.complete_init() —— 置运行位
 */
void app_init(void)
{
  /* 1. 库入口：建 sys_state，创建 sys_task（10 ms）与 can_rx_task（1 ms）。
   *    调度器还没启动，所以这里只是「创建」，真正运行要等 vTaskStartScheduler()。 */
  system_init();

  /* 2. 板级初始化：外设 init()、设备 init()、注册 CanRxNode / CanTxNode。
   *    每一路都带 configASSERT，失败就地停机，fault_record 记下行号。 */
  board_init();

  /* 3. 模块层：设备之后（CoreXY 绑定 emm_group），只做配置校验 */
  module_init();

  /* 4. 放行：板级有发送节点才创建 can_tx_task；随后置运行位。
   *    业务任务的入口第一句都是 sys_state.wait_running()，在这之后才会继续。 */
  system_start();

#if APP_TEST_CAN_RECOVERY_ENABLED
  can_recovery_test_init();
  configASSERT(xTaskCreate(can_recovery_test_task, "can_fault", 512, NULL, tskIDLE_PRIORITY + 5, NULL) == pdPASS);
#endif

#if APP_TEST_ONLINE_CHECK_ENABLED
  configASSERT(xTaskCreate(online_check_test_task, "online_test", 256, NULL, tskIDLE_PRIORITY + 3, NULL) == pdPASS);
#endif

#if APP_TEST_R9DS_ENABLED
  configASSERT(xTaskCreate(r9ds_test_task, "r9ds", 512, NULL, tskIDLE_PRIORITY + 5, NULL) == pdPASS);
#endif

#if APP_TEST_SERVO_ENABLED
  configASSERT(xTaskCreate(servo_test_task, "servo", 512, NULL, tskIDLE_PRIORITY + 5, NULL) == pdPASS);
#endif

#if APP_TEST_EMMV5_GROUP_ENABLED
  configASSERT(xTaskCreate(emmv5_group_test_task, "emm_grp", 512, NULL, tskIDLE_PRIORITY + 5, NULL) == pdPASS);
#endif

#if APP_TEST_EMMV5_POS_PROBE_ENABLED
  configASSERT(xTaskCreate(emmv5_pos_probe_test_task, "emm_pos", 512, NULL, tskIDLE_PRIORITY + 5, NULL) == pdPASS);
#endif

#if APP_TEST_UART_LOOPBACK_ENABLED
  configASSERT(xTaskCreate(uart_loopback_test_task, "uart_loop", 768, NULL, tskIDLE_PRIORITY + 5, NULL) == pdPASS);
#endif

#if APP_TEST_UART_STRESS_ENABLED
  configASSERT(xTaskCreate(uart_stress_test_task, "uart_strs", 768, NULL, tskIDLE_PRIORITY + 5, NULL) == pdPASS);
#endif

  /* 按键任务：200 ms 轮询（短按/长按发不同提示音，优先级 +5） */
  configASSERT(xTaskCreate(key_task, "key", 256, NULL, tskIDLE_PRIORITY + 5, NULL) == pdPASS);

  /* CoreXY：输入源二选一（都占 USART1），执行侧统一走 corexy_io_task */
  configASSERT(corexy_cmd_init());
  configASSERT(xTaskCreate(corexy_io_task, "cxy_io", 512, NULL, tskIDLE_PRIORITY + 5, NULL) == pdPASS);

#if APP_COREXY_PROTOCOL
  configASSERT(xTaskCreate(corexy_task, "cxy_pro", 512, NULL, tskIDLE_PRIORITY + 4, NULL) == pdPASS);
#else
  configASSERT(xTaskCreate(corexy_console_task, "cxy_con", 512, NULL, tskIDLE_PRIORITY + 4, NULL) == pdPASS);
#endif

}


/**
 * @brief 默认任务，这个原本命名为_start_default_task。但是每次开FreeRTOS这个里面，默认是这个名字
 *
 * @note 保留这个名字，但是其他任务要类似：_start_default_task
 * @param argument 任务参数
 */
extern "C" void StartDefaultTask(void *argument)
{
  for (;;)
  {

    vTaskDelay(100);
  }
}
