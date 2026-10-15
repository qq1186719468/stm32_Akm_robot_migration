/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V5.0
修改时间：2022-05-05

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V5.0
Update：2022-05-05

All rights reserved
***********************************************/
#include "system.h"
#include "guo_log.h"
#include "akm_watchdog.h"

//Task priority    //任务优先级
#define START_TASK_PRIO	1

//Task stack size //任务堆栈大小	
#define START_STK_SIZE 	256  

//Task handle     //任务句柄
TaskHandle_t StartTask_Handler;

/*
 * 保留各任务句柄供MonitorTask查询任务状态和栈水位。
 * 这些句柄只用于诊断，不改变原任务的优先级、周期或控制关系。
 */
static TaskHandle_t s_balance_task_handle;
static TaskHandle_t s_mpu6050_task_handle;
static TaskHandle_t s_show_task_handle;
static TaskHandle_t s_led_task_handle;
static TaskHandle_t s_ps2_task_handle;
static TaskHandle_t s_data_task_handle;
static TaskHandle_t s_uart3_dma_task_handle;
static TaskHandle_t s_log_task_handle;
static TaskHandle_t s_watchdog_task_handle;
static TaskHandle_t s_monitor_task_handle;

//Task function   //任务函数
void start_task(void *pvParameters);

//Main function //主函数
int main(void)
 { 
  systemInit(); //Hardware initialization //硬件初始化
  /* 读取上一次复位来源；默认IWDG关闭，不会改变原有启动流程。 */
  AkmWatchdog_CaptureResetCause();
	
	//Create the start task //创建开始任务
	xTaskCreate((TaskFunction_t )start_task,            //Task function   //任务函数
							(const char*    )"start_task",          //Task name       //任务名称
							(uint16_t       )START_STK_SIZE,        //Task stack size //任务堆栈大小
							(void*          )NULL,                  //Arguments passed to the task function //传递给任务函数的参数
							(UBaseType_t    )START_TASK_PRIO,       //Task priority   //任务优先级
							(TaskHandle_t*  )&StartTask_Handler);   //Task handle     //任务句柄    					
	vTaskStartScheduler();  //Enables task scheduling //开启任务调度	
}
 
//Start task task function //开始任务任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL(); //Enter the critical area //进入临界区
	
    //Create the task //创建任务
   	xTaskCreate(Balance_task,  "Balance_task",  BALANCE_STK_SIZE,  NULL, BALANCE_TASK_PRIO,  &s_balance_task_handle);	//Vehicle motion control task //小车运动控制任务
	  xTaskCreate(MPU6050_task, "MPU6050_task", MPU6050_STK_SIZE, NULL, MPU6050_TASK_PRIO, &s_mpu6050_task_handle);	//IMU data read task //IMU数据读取任务 
    xTaskCreate(show_task,     "show_task",     SHOW_STK_SIZE,     NULL, SHOW_TASK_PRIO,     &s_show_task_handle); //The OLED display displays tasks //OLED显示屏显示任务
    xTaskCreate(led_task,      "led_task",      LED_STK_SIZE,      NULL, LED_TASK_PRIO,      &s_led_task_handle);	//LED light flashing task //LED灯闪烁任务
    xTaskCreate(pstwo_task,    "PSTWO_task",    PS2_STK_SIZE,      NULL, PS2_TASK_PRIO,      &s_ps2_task_handle);	//Read the PS2 controller task //读取PS2手柄任务
    xTaskCreate(data_task,     "DATA_task",     DATA_STK_SIZE,     NULL, DATA_TASK_PRIO,     &s_data_task_handle);	//Usartx3, Usartx1 and CAN send data task //串口3、串口1、CAN发送数据任务
	#if AKM_UART3_DMA_ENABLE
	    /* DMA通信任务只负责协议接收和发布目标，不直接计算电机PWM。 */
	    xTaskCreate(UART3_DMA_CommTask, "UART3_DMA", UART3_DMA_TASK_STK_SIZE,
	                NULL, UART3_DMA_TASK_PRIO, &s_uart3_dma_task_handle);
	#endif
	#if AKM_LOG_ENABLE
	    /* 日志任务低优先级运行；控制任务只投递固定长度事件，不等待串口。 */
	    GuoLog_Init();
	    xTaskCreate(GuoLogTask, "AkmLog", AKM_LOG_TASK_STK_SIZE,
	                NULL, AKM_LOG_TASK_PRIO, &s_log_task_handle);
	#endif
    /* 看门狗任务负责心跳诊断；健康门控通过后启动并刷新硬件IWDG。 */
	    xTaskCreate(AkmWatchdogTask, "AkmWdg", AKM_IWDG_TASK_STK_SIZE,
	                NULL, AKM_IWDG_TASK_PRIO, &s_watchdog_task_handle);
	#if AKM_MONITOR_ENABLE
	    /* 低优先级只读监控任务每秒生成一次一致状态快照。 */
	    xTaskCreate(AkmMonitorTask, "AkmMonitor", AKM_MONITOR_TASK_STK_SIZE,
	                NULL, AKM_MONITOR_TASK_PRIO, &s_monitor_task_handle);
	#endif
	    /*
	     * 所有xTaskCreate完成并取得有效句柄后再统一注册。
	     * 此时start_task仍处于临界区，MonitorTask不会与注册过程并发执行。
	     */
	    {
	        akm_monitor_task_registration_t info;

	        info.handle = s_balance_task_handle;
	        info.name = "Balance";
	        info.stack_words = BALANCE_STK_SIZE;
	        info.heartbeat_id = AKM_MONITOR_HB_BALANCE;
	        AkmMonitor_RegisterTask(&info);

	        info.handle = s_mpu6050_task_handle;
	        info.name = "MPU6050";
	        info.stack_words = MPU6050_STK_SIZE;
	        info.heartbeat_id = AKM_MONITOR_HB_IMU;
	        AkmMonitor_RegisterTask(&info);

	        info.handle = s_show_task_handle;
	        info.name = "show";
	        info.stack_words = SHOW_STK_SIZE;
	        info.heartbeat_id = AKM_MONITOR_HB_SHOW;
	        AkmMonitor_RegisterTask(&info);

	        info.handle = s_led_task_handle;
	        info.name = "LED";
	        info.stack_words = LED_STK_SIZE;
	        info.heartbeat_id = AKM_MONITOR_HB_LED;
	        AkmMonitor_RegisterTask(&info);

	        info.handle = s_ps2_task_handle;
	        info.name = "PS2";
	        info.stack_words = PS2_STK_SIZE;
	        info.heartbeat_id = AKM_MONITOR_HB_PS2;
	        AkmMonitor_RegisterTask(&info);

	        info.handle = s_data_task_handle;
	        info.name = "DATA";
	        info.stack_words = DATA_STK_SIZE;
	        info.heartbeat_id = AKM_MONITOR_HB_DATA;
	        AkmMonitor_RegisterTask(&info);

	#if AKM_UART3_DMA_ENABLE
	        info.handle = s_uart3_dma_task_handle;
	        info.name = "UART3_DMA";
	        info.stack_words = UART3_DMA_TASK_STK_SIZE;
	        info.heartbeat_id = AKM_MONITOR_HB_UART3_DMA;
	        AkmMonitor_RegisterTask(&info);
	#endif

	#if AKM_LOG_ENABLE
	        info.handle = s_log_task_handle;
	        info.name = "AkmLog";
	        info.stack_words = AKM_LOG_TASK_STK_SIZE;
	        info.heartbeat_id = AKM_MONITOR_HB_LOG;
	        AkmMonitor_RegisterTask(&info);
	#endif

	        info.handle = s_watchdog_task_handle;
	        info.name = "AkmWdg";
	        info.stack_words = AKM_IWDG_TASK_STK_SIZE;
	        info.heartbeat_id = AKM_MONITOR_HB_WATCHDOG;
	        AkmMonitor_RegisterTask(&info);

	#if AKM_MONITOR_ENABLE
	        info.handle = s_monitor_task_handle;
	        info.name = "AkmMonitor";
	        info.stack_words = AKM_MONITOR_TASK_STK_SIZE;
	        info.heartbeat_id = AKM_MONITOR_HB_MONITOR;
	        AkmMonitor_RegisterTask(&info);
	#endif
	    }
	
    vTaskDelete(StartTask_Handler); //Delete the start task //删除开始任务

    taskEXIT_CRITICAL();            //Exit the critical section//退出临界区
}






