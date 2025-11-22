# TIM（Timer）

## 分类：

- 基本定时器

  - 具有定时功能，可编程控制定时周期，计数器溢出能产生中断、DMA请求

- 通用定时器

  - 具有定时功能
  - 可配置计数器装载模式
  - 具有独立通道进行输入捕获、输出比较、PWM（脉冲宽度调制）输出、单脉冲模式
  - 中断源更多（向上溢出、向下溢出、计数器初始化/启动/终止、输入捕获、输出比较）

- 高级控制定时器*

  - 具有上述功能
  - 可输出嵌入死区时间的互补PWM
  - 允许在指定数目的计数器周期之后更新定时器寄存器的重复计数器
  - 刹车输入信号可以将定时器输出信号置于复位或者一个已知状态
  - 中断源增添了一个刹车信号输入

  ------

  ## 定时器：

  1. 预分频器PSC（TIMx_PSC)
     - 调低时钟信号
  2. CNT计数器（TIMx_CNT）
     - 向上计数
     - 向下计数
     - 中央对齐计数
  3. 自动重装载寄存器（TIMx_ARR）
     - 产生溢出
  4. 捕获/比较寄存器（TIMx_CCRx)
     - 捕获时存储CNT值
     - 输出比较时用来读取CNT值并与CCR比较
  
  
  
  ![image-20231012202831979](C:\Users\13618\AppData\Roaming\Typora\typora-user-images\image-20231012202831979.png)
  
  
  
  实际周期：
  $$
  T_{out}( s) = \frac{（ARR+1) * (PSC+1)}{f(Hz)}
  $$
  PWM占空比：
  $$
  \phi = \frac{CCR}{ARR+1}
  $$
  
  
  ------
  
  ## 函数：
  
  ```c
  HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel) //输入捕获中断启动
      
  HAl_TIM_IC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel) //输入捕获终端中止
      
  HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel) //PWM启动
      
  HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel)  //PWM中止
      
  HAL_TIM_Base_Start(TIM_HandleTypeDef *htim)  //基本定时器启动
      
  HAL_TIM_Base_Stop(TIM_HandleTypeDef *htim)  //基本定时器中止
      
  HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim)  //基本定时器中断启动
      
  HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *htim)  //基本定时器中断中止
      
  ```
  
  ### 中断回调函数：
  
  ```c
  void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  //溢出中断
  
  void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)  //输入捕获中断
  ```
  
  ### 蜂鸣器：
  
  ```c
  //占空比定位50，即CCR为ARR的一半
  //分频1MHz
  //AAR值
  #define L_Do 3822
  #define L_Re 3405
  #define L_Mi 3033
  #define L_Fa 2863
  #define L_So 2551
  #define L_La 2272
  #define L_Xi 2024
  #define M_Do 1911
  #define M_Re 1702
  #define M_Mi 1526
  #define M_Fa 1431
  #define M_So 1275
  #define M_La 1136
  #define M_Xi 1012
  #define H_Do 955
  #define H_Re 851
  #define H_Mi 758
  #define H_Fa 715
  #define H_So 637
  #define H_La 568
  #define H_Xi 506
  ```
  
  

## 呼吸灯：

```c
 /* USER CODE BEGIN 2 */
HAL_TIM_PWM_Start(&htimX,TIM_CHANNEL_X);
 /* USER CODE END 2 */
while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//breathing light	
	for(int k = 0; k<10000 ; k+=20){
		TIMX -> CCRX = k;
		//__HAL_TIM_SetCompare(&htimX,TIM_CHANNEL_X,k);  这两个都是设置CCR
		HAL_Delay(1);
	}
	for(int k=10000 ; k>0 ; k-=20){
		TIMX -> CCRX = k;
		HAL_Delay(1);
	}
  }
  /* USER CODE END 3 */
```

