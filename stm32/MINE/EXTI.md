# EXTI

- **自动停止正在运行的程序并转入处理新情况的程序，处理完毕之后又返回原被暂停的程序继续运行**
- *中断是实现多线程设计的必要条件*

## NVIC（嵌套向量中断控制器）：

- ### 中断优先级配置

  ​	**抢占式优先级**：数字越小优先级越高，能打断其他的中断

  ​	**响应式优先级**：数字越大优先级越高，不能打断其他的中断

  ​	*先判断抢占式优先级大小，再判断响应式优先级大小*，全部相同则比较它们的硬件中断编号，中断编号越小，优先级越高。

  **例如：**① 事件A（1，2） 事件B（1，3）

  ​				由于AB的抢占式优先级相同，因而同时发生事件AB时，根据响应式优先级，会先执行事件B对应的中断；

  ​		   ② 事件A（1，2） 事件B （2，3）

  ​				由于A的抢占式优先级高，若在B执行的过程中发生事件A，则会先执行事件A对应的中断

  

- #### **中断优先级分组**

  |      优先级分组      | 抢占优先级 | 响应优先级 |
  | :------------------: | :--------: | :--------: |
  | NVIC_PriorityGroup_0 |     0      |   0 ~15    |
  | NVIC_PriorityGroup_1 |   0 ~ 1    |   0 ~ 7    |
  | NVIC_PriorityGroup_2 |    0 ~3    |   0 ~ 3    |
  | NVIC_PriorityGroup_3 |   0 ~ 7    |   0 ~ 1    |
  | NVIC_PriorityGroup_4 |   0 ~15    |     0      |

  ​								*一般使用组别4*

  

- ### 读中断请求标志

- ### 清除中断请求标志

- ### 使能中断

- ### 清除中断



## 外部中断：

**中断/事件线:**	

- EXTI0（0，1，2，… ，15）
- F1：0 ~ 4有单独的外部中断，5 ~ 9、10~16为同一个中断
- *同一线路只能选择一种序号*：选了PA0作为中断线就不能再选择PB0；选择了PX5就不能选择PX6

**上升沿触发：**按下按键时触发中断

**下降沿触发：**松开按键时触发中断

**上升下降沿触发：**按下松开都会触发

<img src="C:\Users\13618\AppData\Roaming\Typora\typora-user-images\image-20230929120916673.png" alt="image-20230929120916673" style="zoom:50%;" />

红线为中断模式（是软件上的处理，用代码，常用） 绿线为事件模式（直接用电平信号处理，效率高）

**③或门：**输入有一个是1就会输出1的信号

**④⑥与门：**输入都是1才会输出1的信号  （与事件屏蔽器、中断屏蔽寄存器相接，若屏蔽寄存器发送0，则无法输出）

**挂起请求寄存器：**在硬件层面将中断标志置为1，表示程序在执行中断程序，需要在中断程序中将其清零（往中断标志位写1），否则程序无法响应其它中断，会卡在该中断

**周期性中断**由定时器辅助完成，定时时间到就会发生中断，然后重新计时，与中断标志位无关

## 函数：

```c
void EXTI0_IRQHandler(void)
{
	/* USER CODE BEGIN EXTI0_IRQn 0 */
	/* USER CODE END EXTI0_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
	/* USER CODE BEGIN EXTI0_IRQn 1 */
	/* USER CODE END EXTI0_IRQn 1 */
}
```

```c
void HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin)
{
	if(__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin) != 0x00u )
	{
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
		HAL_GPIO_EXTI_Callback(GPIO_Pin);
	}
}
```

```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	UNUSED(GPIO_Pin);
	if(GPIO_Pin == GPIO_Pin_X)
	{
		/*功能*/
	}
}
```

## 备注：

- 可以通过延时函数实现按键消抖 (需要把EXTIX的抢占优先级调低，否则滴答时钟不能中断)

  ```c
  HAL_Delay(10);	//需要用到系统滴答时钟ISR中断，抢占优先级为0
  if(HAL_GPIO_ReadPin(GPIOX,GPIO_Pin) != GPIO_PIN_RESET) //不是低电平即未松开（按下是高电平）
  {
      return;
  }
  ```

  
