# RTOS配置

<img src="D:\Study\Note\stm32\MINE\RTOS\RTOS配置.assets\微信截图_20241002203843.png" alt="微信截图_20241002203843" style="zoom:67%;" />

<img src="D:\Study\Note\stm32\MINE\RTOS\RTOS配置.assets\微信截图_20241002203906.png" alt="微信截图_20241002203906" style="zoom:67%;" />

以下是RTOS内核设置的介绍，以及相应的配置建议。

设置选项包括`内核设置`、`内存管理设置`、`钩子函数相关定义`、`运行时和任务统计`、`协程相关定义`、`软件定时器定义`以及`中断嵌套行为配置`。

### 1. 内核设置

- **USE_PREEMPTION**: 启用抢占功能，使得高优先级任务可以中断低优先级任务的执行。
- **CPU_CLOCK_HZ**: CPU时钟频率，通常设置为系统时钟频率，如 `SystemCoreClock`。
- **TICK_RATE_HZ**: 系统滴答频率，设置为 1000 Hz，表示每秒钟产生1000次滴答。
- **MAX_PRIORITIES**: 最大优先级数量，设置为 7，表示系统支持7个不同的优先级。
- **MINIMAL_STACK_SIZE**: 最小任务栈大小，设置为128字（通常是32位系统中的512字节）。
- **MAX_TASK_NAME_LEN**: 任务名称的最大长度，设置为16个字符。
- **USE_16_BIT_TICKS**: 是否使用16位滴答计数器，禁用（适用于大于65535滴答周期的情况）。
- **IDLE_SHOULD_YIELD**: 如果启用，空闲任务将会在每个滴答中让出处理器。
- **USE_MUTEXES**: 启用互斥量，用于保护共享资源。
- **USE_RECURSIVE_MUTEXES**: 启用递归互斥量，允许同一任务多次获得同一互斥量。
- **USE_COUNTING_SEMAPHORES**: 启用计数信号量。
- **QUEUE_REGISTRY_SIZE**: 队列注册表大小，设置为8。
- **USE_APPLICATION_TASK_TAG**: 启用应用程序任务标签，便于任务标识。
- **ENABLE_BACKWARD_COMPATIBILITY**: 启用向后兼容性，确保旧代码的兼容性。
- **USE_PORT_OPTIMISED_TASK_SELECTION**: 启用优化的任务选择，通常可以提高效率。
- **USE_TICKLESS_IDLE**: 禁用滴答空闲（节能模式），若无长时间空闲任务则可以启用。
- **USE_TASK_NOTIFICATIONS**: 启用任务通知机制，以简化任务间通信。
- **RECORD_STACK_HIGH_ADDRESS**: 启用高地址栈记录，便于调试。

### 2. 内存管理设置

- **Memory Allocation**: 使用动态或静态内存分配。
- **TOTAL_HEAP_SIZE**: 堆的总大小，设置为3072字节。
- **Memory Management scheme**: 指定内存管理方案（如`heap_4`）。

### 3. 钩子函数相关定义

- **USE_IDLE_HOOK**: 是否启用空闲任务钩子。
- **USE_TICK_HOOK**: 是否启用滴答钩子。
- **USE_MALLOC_FAILED_HOOK**: 启用内存分配失败钩子。
- **USE_DAEMON_TASK_STARTUP_HOOK**: 启用守护任务启动钩子。
- **CHECK_FOR_STACK_OVERFLOW**: 启用栈溢出检查。

### 4. 运行时和任务统计

- **GENERATE_RUN_TIME_STATS**: 启用运行时间统计生成。
- **USE_TRACE_FACILITY**: 启用追踪功能。
- **USE_STATS_FORMATTING_FUNCTIONS**: 启用统计格式化函数。

### 5. 协程相关定义

- **USE_CO_ROUTINES**: 启用协程支持。
- **MAX_CO_ROUTINE_PRIORITIES**: 设置最大协程优先级。

### 6. 软件定时器定义

- **USE_TIMERS**: 启用软件定时器。
- **TIMER_TASK_PRIORITY**: 定时器任务优先级，设置为2。
- **TIMER_QUEUE_LENGTH**: 定时器队列长度，设置为10。
- **TIMER_TASK_STACK_DEPTH**: 定时器任务栈深度，设置为256字。

### 7. 中断嵌套行为配置

- **LIBRARY_LOWEST_INTERRUPT_PRIORITY**: 设置最低中断优先级，设置为15。
- **LIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY**: 设置最大系统调用中断优先级，设置为5。

### 建议设置

1. **优先级设置**: 如果系统中有多个任务，确保 `MAX_PRIORITIES` 设置足够高（如7或更高），以适应不同优先级的需求。
2. **内存管理**: 根据任务数量和堆使用情况调整 `TOTAL_HEAP_SIZE`，确保有足够的堆内存。
3. **钩子函数**: 根据需要启用或禁用钩子函数，特别是在调试和性能分析时。
4. **任务通知**: 如果需要简化任务间通信，可启用任务通知。
5. **节能模式**: 根据应用场景决定是否启用 `USE_TICKLESS_IDLE`。
6. **中断优先级**: 确保中断优先级合理设置，以避免高优先级中断对低优先级任务的干扰。

这些设置的具体配置应根据应用的实际需求、资源约束和性能要求进行调整。



## 钩子函数：

钩子函数（Hook Function）是一种特殊的函数，允许程序在特定事件发生时执行自定义代码。它们提供了一种灵活的方式来扩展或定制系统的行为，而无需修改底层代码。钩子函数在操作系统、库和框架中广泛使用，尤其是在实时操作系统（RTOS）和嵌入式系统中。

### 1. 钩子函数的类型

- **空闲任务钩子（Idle Hook）**:
  在系统处于空闲状态时执行，可以用来进行低优先级的后台处理，比如任务清理或低功耗管理。

- **滴答钩子（Tick Hook）**:
  在系统滴答中调用，适用于需要在每个滴答周期执行特定操作的场景，如统计系统性能或更新定时器。

- **内存分配失败钩子（Malloc Failed Hook）**:
  当内存分配失败时调用，适合进行错误处理或日志记录。

- **守护任务启动钩子（Daemon Task Startup Hook）**:
  在守护任务启动时调用，用于初始化资源或状态。

- **栈溢出检查钩子（Stack Overflow Hook）**:
  在检测到栈溢出时调用，用于处理错误或进行必要的调试。

### 2. 使用钩子函数的优点

- **灵活性**: 用户可以根据具体需求在特定事件发生时插入自定义代码。
- **可扩展性**: 提供了一种机制来扩展现有功能，而无需修改核心代码。
- **隔离性**: 钩子函数通常与主程序逻辑分开，有助于保持代码的整洁。

### 3. 示例

以下是一个简单的空闲任务钩子函数的示例：

```c
void vApplicationIdleHook(void) {
    // 在空闲任务中执行的代码
    // 例如，进入低功耗模式
}
```

### 4. 注意事项

- **性能**: 钩子函数的执行时间应尽量短，以避免影响系统的实时性。
- **错误处理**: 在钩子函数中处理错误时需谨慎，以免导致系统不稳定。
- **可配置性**: 在设计系统时，钩子函数的启用和调用应可配置，以便用户根据需求选择性使用。

### 总结

钩子函数是一种强大的编程工具，适用于需要在特定事件触发时执行特定逻辑的场景。它们为系统的灵活性和可扩展性提供了保障，是现代软件设计中的重要组成部分。







<img src="D:\Study\Note\stm32\MINE\RTOS\RTOS配置.assets\微信截图_20241002203939.png" alt="微信截图_20241002203939" style="zoom:67%;" />

这个图展示的是一些与任务管理相关的函数和功能的配置项，通常出现在实时操作系统（RTOS）或调度程序中。每个函数或功能都有“Enabled”或“Disabled”的状态，指示其在系统中的启用或禁用情况。

### 1. 任务管理相关函数

- **vTaskPrioritySet**: 设置任务的优先级，Enabled表示可以动态调整任务优先级。
  
- **uxTaskPriorityGet**: 获取任务的当前优先级，Enabled表示可以查询任务优先级。
  
- **vTaskDelete**: 删除任务，Enabled表示可以动态删除任务。
  
- **vTaskCleanUpResources**: 清理任务资源，Disabled表示不启用。
  
- **vTaskSuspend**: 暂停任务，Enabled表示可以暂停某个任务。
  
- **vTaskDelayUntil**: 实现任务延迟，支持精确延迟，Enabled表示可以使用此功能。
  
- **vTaskDelay**: 使任务延迟一段时间，Enabled表示可以使用此功能。

- **vTaskGetSchedulerState**: 获取调度器状态，Enabled表示可以查询调度器的当前状态。

- **xTaskResumeFromISR**: 从中断服务例程恢复任务，Enabled表示可以在ISR中恢复任务。

- **xQueueGetMutexHolder**: 获取当前持有互斥量的任务，Disabled表示不启用。

- **xSemaphoreGetMutexHolder**: 获取当前持有信号量的任务，Disabled表示不启用。

- **pcTaskGetTaskName**: 获取任务名称，Disabled表示不启用。

- **uxTaskGetStackHighWaterMark**: 获取任务栈的高水位标记，Disabled表示不启用。

- **xTaskGetCurrentTaskHandle**: 获取当前任务的句柄，Disabled表示不启用。

- **eTaskGetState**: 获取任务状态，Disabled表示不启用。

- **xEventGroupSetBitFromISR**: 从ISR中设置事件组的位，Disabled表示不启用。

- **xTimerPendFunctionCall**: 延迟调用定时器函数，Disabled表示不启用。

- **xTaskAbortDelay**: 允许中止任务的延迟，Disabled表示不启用。

- **xTaskGetHandle**: 获取任务的句柄，Disabled表示不启用。

### 2. 配置建议

- **启用必要的功能**: 根据应用需求，启用所需的任务管理功能。例如，如果需要动态管理任务优先级，则启用 `vTaskPrioritySet` 和 `uxTaskPriorityGet`。

- **优化性能**: 根据系统的实时性要求，合理配置任务暂停、延迟和恢复功能，以保持系统的响应性。

- **调试与监控**: 在开发和调试阶段，可以启用获取任务状态和栈高水位标记的功能，以帮助监控系统的运行情况。

### 总结

这些函数和功能的配置直接影响系统的任务管理能力及灵活性。在设置时，应根据项目的具体需求和资源限制进行合理选择。





<img src="D:\Study\Note\stm32\MINE\RTOS\RTOS配置.assets\微信截图_20241002203948-17278735617232.png" alt="微信截图_20241002203948" style="zoom:67%;" />

#### 1. Newlib 设置

- USE_NEWLIB_REENTRANT:
  - **状态**: Disabled（禁用）
  - **说明**: 此选项用于启用线程安全的 Newlib 库版本。禁用此选项意味着在多线程环境中，使用 Newlib 时可能会遇到数据竞争问题。

#### 2. 项目设置

- Use FW pack heap file:
  - **状态**: Enabled（启用）
  - **说明**: 此选项表明项目使用固件包提供的堆文件。这通常用于管理动态内存分配，以便在整个应用程序中有效利用堆空间。

