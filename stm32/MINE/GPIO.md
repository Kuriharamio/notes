# GPIO

<img src="C:\Users\13618\AppData\Roaming\Typora\typora-user-images\image-20230929012825169.png" alt="image-20230929012825169" style="zoom:50%;" />

### 八种模式：

- #### 		输入：*输入浮空*	*输入上拉*	*输入下拉*	*模拟输入*

- #### 		输出：*开漏输出*	*推挽输出*	*推挽式复用功能*	*开漏复用功能*	


------

## 输入上拉：

<img src="C:\Users\13618\AppData\Roaming\Typora\typora-user-images\image-20230929013304117.png" alt="image-20230929013304117" style="zoom:50%;" />

- ​	***VDD相当于3.3V，VSS相当于0V***

- ​	**I/O接口无输入时，处于高电平；有输入时取决于输入电平**

- ​	***输入浮空即VSS与VDD都不接通***

- ​	**输入下拉相反	两者都不会影响输入  只影响无输入时的状态**


------

## 模拟输入：

<img src="C:\Users\13618\AppData\Roaming\Typora\typora-user-images\image-20230929014034547.png" alt="image-20230929014034547" style="zoom: 33%;" />

​	***通过输入不同的电压，经由ADC模块转化为不同数值，不止有0和1***

------

## 开漏输出（Output Open Drain)：

<img src="C:\Users\13618\AppData\Roaming\Typora\typora-user-images\image-20230929014945411.png" alt="image-20230929014945411" style="zoom: 50%;" />

- ***MOS是三极管 与上面的VDDVSS类似***

- ***N·MOS接通时，输出低电平；断开时输出浮空状态；无法输出高电平（P·MOS不接通）***

- **可以实现低电压控制高电压***	***I/O接口为高阻态（可视为断路），导线电压由外部电压决定***

- ------

- ## 推挽输出（Output Push Pull)：


<img src="C:\Users\13618\AppData\Roaming\Typora\typora-user-images\image-20230929020114043.png" alt="image-20230929020114043" style="zoom:50%;" />

***与开漏输出类似，但可接通P·MOS，即可输出高电平***

------

## 推挽式复用、开漏式复用：

![image-20230929020423429](C:\Users\13618\AppData\Roaming\Typora\typora-user-images\image-20230929020423429.png)



## 函数：

```C
HAL_GPIO_WritePin(GPIOX , GPIO_Pin , GPIO_State); //设置GPIO电平高低 
```

​		***状态为0（GPIO_PIN_RESET）低电平 或1（GPIO_PIN_SET) 高电平***

```c
HAL_GPIO_TogglePin(GPIOX , GPIO_Pin);			//翻转GPIO电平状态
```

```c
HAL_GPIO_ReadPin(GPIOX , GPIO_Pin);				//读取GPIO电平状态
```

## 按键：



## 备注：

- LED是低电平的时候亮，高电平的时候灭

- 上拉下拉仅存在于输入，输出时无需上拉下拉

- LED配置：![image-20230929022444062](C:\Users\13618\AppData\Roaming\Typora\typora-user-images\image-20230929022444062.png)

- GPIO_PIN_ALL 所有IO

- ```c
  int cnt = 0;
  HAL_GPIO_WritePin(GPIOC , GPIO_PIN_0 << cnt , GPIO_PIN_SET); 	//熄灭所有灯	
  HAL_GPIO_WritePin(GPIOC , GPIO_PIN_0 << cnt , GPIO_PIN_RESET);	//依次点亮灯
  cnt++;
  if(cnt == x) cnt = 0;
  HAL_Delay(100);
  ```

  <img src="C:\Users\13618\AppData\Roaming\Typora\typora-user-images\image-20230929024839087.png" alt="image-20230929024839087" style="zoom:50%;" />

  ​													**依次左移可以实现递增**