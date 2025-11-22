# IIC (Inter-Integrated Circuit) 

## IIC 总线

- **半双工**通信方式
- 支持多设备，多主控（同一时间只能有一个主控）； 而USART只能两个设备进行通信
- IIC串行总线有两根信号线：**双向数据线SDA**、**时钟线SCL**<img src="C:\Users\13618\AppData\Roaming\Typora\typora-user-images\image-20230929191322336.png" alt="image-20230929191322336" style="zoom:50%;" />
- SDA接SDA，SCL接SCL
- 硬件IIC非常复杂且不稳定，故而用**软件模拟**

------

## IIC 数据传输

<img src="C:\Users\13618\AppData\Roaming\Typora\typora-user-images\image-20230929191759903.png" alt="image-20230929191759903" style="zoom:50%;" />

- **起始信号S**：时钟线SCL高电平，数据线SDA下降沿

- **终止信号P：**时钟线SCL高电平，数据线SDA上升沿

  <img src="C:\Users\13618\AppData\Roaming\Typora\typora-user-images\image-20230929192802997.png" alt="image-20230929192802997" style="zoom: 33%;" />

- **应答机制：**

  1. 对于反馈有效应答位ACK的要求是：接收器在第9个时钟脉冲之前的低电平期将SDA拉低，并确保在该时钟的高电平期间为稳定的低电平；

  2. 如果接收器是主控器，则在他收到最后一个字节后会发送一个NACK信号，以通知被控发送器结束数据发送，并释放SDA，以便发送终止信号P

     <img src="C:\Users\13618\AppData\Roaming\Typora\typora-user-images\image-20230929194036868.png" alt="image-20230929194036868" style="zoom: 50%;" />

- 数据在SCL的上升沿到来之前就绪准备好，并在下降沿到来之前必须稳定

- **数据帧格式**

<img src="C:\Users\13618\AppData\Roaming\Typora\typora-user-images\image-20230929201120376.png" alt="image-20230929201120376" style="zoom: 67%;" />

1. *ADDR：从设备地址 （大多数为7位，协议规定再添加一个最低位来表示数据传输方向，共8位）*
2. *1：读	0：写*
3. *ACK：应答	NACK：非应答*
4. *S：起始	P：终止*

- **IIC仲裁机制**

------

## GPIO模拟IIC时序：

```c
void IIC_SDA(uint8_t status)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, status);
}

void IIC_SCL(uint8_t status)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, status);
}

uint8_t IIC_SDA_Read(void)
{
	return HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6);
}

void IIC_Delay(void)
{
	delay_us(10);
}

void IIC_Start(void)
{
	IIC_SDA(1);
	IIC_SCL(1);
	IIC_Delay();
	IIC_SDA(0);
	IIC_Delay();
	IIC_SCL(0);
}

void IIC_Stop(void)
{
	IIC_SCL(0);
	IIC_SDA(0);
	IIC_Delay();
	IIC_SCL(1);
	IIC_Delay();
	IIC_SDA(1);
	IIC_Delay();
}

void IIC_Ack(void)
{
	IIC_SCL(0);
	IIC_Delay();
	IIC_SCL(1);
	IIC_Delay();
	IIC_SCL(0);
}

void IIC_NAck(void)
{
	IIC_SDA(1);
	IIC_Delay();
	IIC_SCL(1);
	IIC_Delay();
	IIC_SCL(0);
	IIC_Delay();
}

int IIC_WaitAck(void)
{

	uint8_t waittime = 0;
	uint8_t rack = 0;
	IIC_SDA(1);
	IIC_Delay();
	IIC_SCL(1);
	IIC_Delay();
	while (IIC_SDA_Read())
	{
		waittime++;
		if (waittime > 250)
		{
			IIC_Stop();
			rack = 1;
			break;
		}
	}
	IIC_SCL(0);
	IIC_Delay();
	return rack;
}


void IIC_WriteBit(uint8_t data)
{
	uint8_t i;
	IIC_SDA(0);
	IIC_SCL(0);
	for(i = 0; i<8 ;i++)
	{
		IIC_SDA((data & 0x80) >> 7);
		IIC_Delay();
		IIC_SCL(1);
		IIC_Delay();
		IIC_SCL(0);
		data <<= 1;
		IIC_Delay();
	}
	IIC_SDA(1);
}

uint8_t IIC_ReadBit(uint8_t ack)
{
	uint8_t i, addr = 0;
	for(i = 0; i<8 ;i++)
	{
		IIC_SCL(0);
		IIC_Delay();
		IIC_SCL(1);
		addr <<= 1;
		if(IIC_SDA_Read())
			addr++;
		IIC_Delay();
	}
	if(ack){
		IIC_Ack();
	IIC_SCL(0);
	}
	else{
		IIC_NAck();
	}
	return addr;
}

uint8_t Write_One_Byte(uint8_t addr, uint8_t data)
{
	IIC_Start();
	IIC_WriteBit(0xA0);
	IIC_WaitAck();
	IIC_WriteBit(addr);
	IIC_WaitAck();
	IIC_WriteBit(data);
	IIC_WaitAck();
	IIC_Stop();
	delay_ms(10);
}

uint8_t Read_One_Byte(uint8_t addr)
{
	uint8_t temp = 0;
	IIC_Start();
	IIC_WriteBit(0xA0);
	IIC_WaitAck();
	IIC_WriteBit(addr);
	IIC_WaitAck();
	IIC_Start();
	IIC_WriteBit(0xA1);
	IIC_WaitAck();
	temp = IIC_ReadBit(0);
	IIC_Stop();
	return temp;
}

/**
 * @brief IIC写字节函数
 * @param addr EEPROM的写入地址
 * @param send_buf  发送缓冲区
 * @param size      发送大小
 */
static HAL_StatusTypeDef EEPROM_IIC_MemWrite(uint8_t addr, uint8_t *send_buf, uint16_t size)
{

	while(size--)
	{
		Write_One_Byte(addr,*send_buf);
		addr++;
		send_buf++;
	}
	return 0;

}

/**
 * @brief IIC读字节函数
 * @param addr EEPROM的读取地址
 * @param send_buf  接收缓冲区
 * @param size      读取数据大小
 */
static HAL_StatusTypeDef EEPROM_IIC_MemRead(uint8_t addr, uint8_t *rev_buf, uint16_t size)
{

	while(size--)
	{
		*rev_buf++= Read_One_Byte(addr++);
	}
	return 0;
}

/****************************************************/
/**
 * @brief EEPROM 写入函数
 * @param addr EEPROM的读取地址
 * @param send_buf  接收缓冲区
 * @param size      读取数据大小
 */
void EEPROM_Read(uint8_t addr, uint8_t *rev_buf, uint16_t size)
{
    if (EEPROM_IIC_MemRead(addr, rev_buf, size) == HAL_OK) {
        ;
    }
}

/**
 * @brief EEPROM 写入函数
 * @param addr EEPROM的读取地址
 * @param send_buf  接收缓冲区
 * @param size      读取数据大小
 */
void EEPROM_Write(uint8_t addr, uint8_t *send_buf, uint16_t size)
{
    if (EEPROM_IIC_MemWrite(addr, send_buf, size) == HAL_OK) {
       ;
    }
}
```

在main.c中：

```c
/* USER CODE BEGIN PV */
uint8_t rx_buf[10] = {0};
uint8_t tx_buf[10] = {0};
/* USER CODE END PV */

/* USER CODE BEGIN 2 */
	__HAL_RCC_GPIOC_CLK_ENABLE();

  printf("-------------Read Test------------------\r\n");
  EEPROM_Read(0, rx_buf, 10);
  for (int i = 0; i < 10; i++) {
      printf("[0x%08x]:0x%02x\r\n", i, rx_buf[i]);
  }
	HAL_Delay(10000);
  printf("-------------Write Test-----------------\r\n");
  for (int i = 0; i < 10; i++) {
      tx_buf[i] = i + 1;
  }
  EEPROM_Write(0, tx_buf, 10);
	  for (int i = 0; i < 10; i++) {
      printf("[0x%08x]:0x%02x\r\n", i, tx_buf[i]);
  }

  printf("-------------Read Test------------------\r\n");
  EEPROM_Read(0, rx_buf, 10);
  for (int i = 0; i < 10; i++) {
      printf("[0x%08x]:0x%02x\r\n", i, rx_buf[i]);
  }

  /* USER CODE END 2 */ 
```

