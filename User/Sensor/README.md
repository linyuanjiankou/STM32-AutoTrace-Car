# 5路红外巡线传感器驱动说明

## 概述

本传感器驱动采用**中断方式**，适用于 STM32F103 基础款巡线小车项目。5个红外反射传感器实时监测地面黑线位置，通过中断方式捕获状态变化，保证响应实时性。

## 硬件配置

### 传感器引脚连接

| 传感器名称 | 引脚 | 端口 | 描述 |
|-----------|------|------|------|
| LEFT2 (左侧最外端) | PC14 | GPIOC | 左侧最外侧传感器 |
| LEFT1 | PC13 | GPIOC | 左侧内侧传感器 |
| CENTER (居中) | PB1 | GPIOB | 中心传感器 |
| RIGHT1 | PA3 | GPIOA | 右侧内侧传感器 |
| RIGHT2 (右侧最外端) | PA2 | GPIOA | 右侧最外侧传感器 |

### 传感器特性

- **类型**：红外反射式传感器（数字量输出）
- **工作电压**：3.3V（与 STM32 兼容）
- **输出电平**：
  - **高电平 (1)**：检测到黑色线条
  - **低电平 (0)**：检测到白色表面
- **触发方式**：双边沿触发（上升沿+下降沿）
- **上拉电阻**：内部上拉

### 传感器安装示意

```
    ← 左侧                           右侧 →
  [LEFT2] [LEFT1] [CENTER] [RIGHT1] [RIGHT2]
     ●        ●         ●         ●        ●
     <--- 间距相等  ---> <--- 间距相等 --->
```

## 工作原理

### 中断机制

当传感器检测状态发生变化时（从白到黑或从黑到白），会触发外部中断：

1. 传感器状态变化 → 引脚电平变化
2. EXTI 检测到边沿 → 触发中断
3. 中断服务函数读取当前电平 → 更新全局变量
4. 设置"新数据标志" → 主程序可读取

### 数据更新流程

```
传感器状态变化
    ↓
中断服务函数调用
    ↓
读取当前引脚电平
    ↓
更新 SENSOR_Data 结构体
    ↓
设置 SENSOR_NewDataFlag = 1
    ↓
主程序检测到标志位 → 读取数据
    ↓
清除标志位，处理数据
```

### SENSOR_ReadRaw() 特点

- `SENSOR_ReadRaw()` 直接读取所有引脚电平，而不是使用缓存数据
- 这样可以确保读取到的是同一时刻的5路传感器数据
- 中断只更新单个传感器的缓存值，`SENSOR_ReadRaw()` 不使用缓存

## 使用指南

### 1. 包含头文件

```c
#include "sensor.h"
```

### 2. 初始化传感器

在 `main()` 函数的初始化部分调用：

```c
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    SENSOR_Init();  // 初始化传感器

    // ... 其他初始化
}
```

### 3. 读取传感器数据

在主循环中检查并读取传感器数据：

```c
while (1)
{
    // 检查是否有新的传感器数据
    if (SENSOR_CheckNewData())
    {
        SENSOR_Status_t sensors;

        // 读取传感器数据
        SENSOR_ReadRaw(&sensors);

        // 处理传感器数据
        if (sensors.CENTER == 1 && sensors.LEFT1 == 1 && sensors.RIGHT1 == 1)
        {
            // 五个传感器都检测到黑线 - 在白色区域
            Motor_Run(MOTOR_ID_A, MOTOR_FWD, 40);
            Motor_Run(MOTOR_ID_B, MOTOR_FWD, 50);
        }
        else if (sensors.CENTER == 0)
        {
            // 中心传感器检测到白线 - 调整位置
            // ... 你的巡线逻辑
        }
    }

    HAL_Delay(10);  // 主循环延迟
}
```

## API 参考

### SENSOR_Init(void)

**功能**：初始化传感器驱动

**参数**：无

**返回**：无

**说明**：
- 清空传感器状态缓存
- 清除新数据标志
- GPIO 引脚在 `MX_GPIO_Init()` 中已配置为中断模式

---

### SENSOR_ReadRaw(SENSOR_Status_t \*data)

**功能**：读取所有5路传感器的最新状态（直接读取引脚，非缓存）

**参数**：
- `data`：指向 `SENSOR_Status_t` 结构体的指针，用于存储读取的数据

**返回**：无

**说明**：
- 本函数直接读取所有引脚电平，确保数据一致性
- 不使用中断缓存的数据

**示例**：
```c
SENSOR_Status_t sensors;
SENSOR_ReadRaw(&sensors);

// 访问各个传感器数据
printf("LEFT2: %d, LEFT1: %d, CENTER: %d, RIGHT1: %d, RIGHT2: %d\r\n",
       sensors.LEFT2, sensors.LEFT1, sensors.CENTER,
       sensors.RIGHT1, sensors.RIGHT2);
```

---

### SENSOR_CheckNewData(void)

**功能**：检查是否有新的传感器数据

**参数**：无

**返回**：`uint8_t` - 1 表示有新数据，0 表示无新数据

**说明**：
- 读取后会自动清除标志位
- 建议在主循环中定期调用此函数，检查是否需要处理新数据

**示例**：
```c
if (SENSOR_CheckNewData())
{
    SENSOR_Status_t sensors;
    SENSOR_ReadRaw(&sensors);
    // 处理数据
}
```

## 传感器状态说明

### 数据含义

| 值 | 含义 | 说明 |
|---|------|------|
| 1 | 检测到白色 | 引脚为高电平（由于上拉电阻） |
| 0 | 检测到黑色 | 引脚被传感器拉低 |

### 典型场景

#### 场景1：小车在黑线左侧
```
  [□] [□] [■] [■] [■]
  LEFT2 LEFT1 CENTER RIGHT1 RIGHT2
   1      1      0      0      0
```
处理：右转调整

#### 场景2：小车在黑线右侧
```
  [■] [■] [■] [□] [□]
  LEFT2 LEFT1 CENTER RIGHT1 RIGHT2
   0      0      0      1      1
```
处理：左转调整

#### 场景3：小车居中行驶
```
  [□] [■] [■] [■] [□]
  LEFT2 LEFT1 CENTER RIGHT1 RIGHT2
   1      0      0      0      1
```
处理：保持直行

#### 场景4：全部检测到黑线（十字交叉）
```
  [■] [■] [■] [■] [■]
  LEFT2 LEFT1 CENTER RIGHT1 RIGHT2
   0      0      0      0      0
```
处理：根据实际需求决定（可能是十字路口）

## 完整示例代码

```c
#include "main.h"
#include "sensor.h"
#include "motor.h"

// 巡线速度参数
#define SPEED_LEFT   45
#define SPEED_RIGHT  45
#define SPEED_ADJUST 15

/**
  * @brief  巡线控制函数
  * @note   根据传感器状态调整小车行驶方向
  */
void LineFollow_Control(void)
{
    SENSOR_Status_t sensors;

    // 读取传感器数据
    SENSOR_ReadRaw(&sensors);

    // 判断传感器状态
    uint8_t left_sensors  = sensors.LEFT2 || sensors.LEFT1;
    uint8_t right_sensors = sensors.RIGHT1 || sensors.RIGHT2;
    uint8_t center_sensor = sensors.CENTER;

    if (left_sensors && !right_sensors)
    {
        // 黑线在右侧 - 右转
        Motor_Run(MOTOR_ID_A, MOTOR_FWD, SPEED_LEFT + SPEED_ADJUST);
        Motor_Run(MOTOR_ID_B, MOTOR_FWD, SPEED_RIGHT - SPEED_ADJUST);
    }
    else if (!left_sensors && right_sensors)
    {
        // 黑线在左侧 - 左转
        Motor_Run(MOTOR_ID_A, MOTOR_FWD, SPEED_LEFT - SPEED_ADJUST);
        Motor_Run(MOTOR_ID_B, MOTOR_FWD, SPEED_RIGHT + SPEED_ADJUST);
    }
    else
    {
        // 基本居中 - 直行
        Motor_Run(MOTOR_ID_A, MOTOR_FWD, SPEED_LEFT);
        Motor_Run(MOTOR_ID_B, MOTOR_FWD, SPEED_RIGHT);
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM4_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();
    MX_USART1_UART_Init();

    Motor_Init();
    SENSOR_Init();

    while (1)
    {
        // 检查是否有新的传感器数据
        if (SENSOR_CheckNewData())
        {
            LineFollow_Control();
        }

        HAL_Delay(5);  // 5ms 循环周期
    }
}
```

## 文件结构

```
User/Sensor/
├── sensor.h      # 传感器驱动头文件
├── sensor.c      # 传感器驱动实现文件
└── README.md     # 本说明文档
```

## 注意事项

1. **中断优先级**：所有传感器中断优先级设置为 0，可通过 `MX_GPIO_Init()` 中的 `HAL_NVIC_SetPriority()` 修改

2. **数据一致性**：`SENSOR_ReadRaw()` 直接读取所有引脚，确保数据一致性；中断缓存仅用于中断服务函数快速更新

3. **小车方向定义**：
   - `MOTOR_FWD`：电机正转
   - `MOTOR_BWD`：电机反转
   - 根据实际小车前进方向调整电机控制逻辑

4. **传感器调试**：
   - 可以通过串口打印传感器状态进行调试
   - 确保传感器安装高度合适（距离地面约 1-2cm）
   - 调节传感器上的电位器设置合适的检测阈值

5. **电源稳定性**：红外传感器对电源噪声较敏感，建议：
   - 使用稳定的 3.3V 电源
   - 在传感器附近添加去耦电容
   - 信号线尽量短并远离干扰源

## 许可证

Copyright (c) 2026 STMicroelectronics.
本软件根据 LICENSE 文件中规定的条款授权使用。
