# FreeRTOS

## 1、STM32CubeMX配置：

- Tasks and Queues：
  - 设置Task_Init
    - 参数：
      - Task Name
      - Priority
      - Stack Size
      - Entry Function
      - Code Generation Option
      - Parameter
      - Allocation
      - Buffer Name
      - Control Block Name

## 2、Task_Init

- ```c
  /*进入临界区*/
  taskENTER_CRITICAL();//把外部中断都屏蔽
  /***********************
  初始化
  ************************/
  /***********************
  创建任务
  xTaskCreate(函数名,任务名,全局变量数,NULL,优先级，&TaskHandle_xx);
  ************************/    
  HAL_Delay(200);			//延时以使任务完成
  vTaskDelete(NULL);		//只执行一遍的要删除
  taskEXIT_CRITICAL();	//取消外部中断的屏蔽
  /*退出临界区*/
  ```

- ```c
  void StartDefaultTask(void const * argument)
  {
    /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
  	taskENTER_CRITICAL();
      
      //Init
  	
      //Create Thead
  	osThreadDef(ledtTask, taskLed, osPriorityNormal, 0, 128);
      defaultTaskHandle = osThreadCreate(osThread(ledtTask), NULL);
  
  
     vTaskDelete(NULL);
     taskEXIT_CRITICAL();
    
    /* USER CODE END StartDefaultTask */
  }
  ```
  
  ```c
  #include "cmsis_os.h"
  
  void task_name(void const * argument)
  {
      TickType_t xLastWakeTime;
      const TickType_t xFrequency = pdMS_TO_TICKS(1);
      xLastWakeTime = xTaskGetTickCount();
      while (1)
      {
          
          vTaskDelayUntil(&xLastWakeTime, xFrequency);
      }
  }
  ```
  
  

## 3、其他函数

- ```
  vTaskDelay()
  vTaskDelayUntil()
  ```

  