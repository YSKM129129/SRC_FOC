# STM32G474 FOC电机控制板外设功能说明

## 芯片型号
**STM32G474CBTx** (LQFP48封装)
- 系统时钟：170MHz
- 基于ARM Cortex-M4内核

---

## 1. ADC（模数转换器）

### ADC1
**功能：电机相电流采样（U相和V相）**
- **PA0 (ADC1_IN1)**: 电机U相电流采样
- **PA1 (ADC1_IN2)**: 电机V相电流采样
- **配置**：
  - 12位分辨率
  - 注入式转换模式
  - 由TIM1_TRGO触发（与PWM同步采样）
  - 用于FOC算法的Clarke变换和Park变换
  
### ADC2
**功能：总线电压和温度监测**
- **PB11 (ADC2_IN14)**: 需根据硬件原理图确认（可能是母线电压或温度）
- **PA4 (ADC2_IN17)**: 需根据硬件原理图确认（可能是温度或母线电压）
- **配置**：
  - 12位分辨率
  - 独立模式运行

**说明**：
- FOC项目中使用了PC0(ADC2_IN6)和PC1(ADC2_IN7)
- 代码中定义了`vote_BUS`（母线电压）和`tempture`（温度）变量
- 具体哪个引脚对应哪个功能需要查看硬件原理图
- 常见配置：母线电压用于过压/欠压保护，温度用于过热保护

**用途**：电机驱动系统保护和监测

---

## 2. SPI（串行外设接口）

### SPI1 + DMA
**功能：AS5047P磁编码器通信**
- **PA5**: SPI1_SCK（时钟）
- **PA6**: SPI1_MISO（主机输入）
- **PA7**: SPI1_MOSI（主机输出）
- **PB2 (AS5047P_CS)**: 片选信号（GPIO控制）
- **配置**：
  - 主机模式
  - 16位数据
  - 波特率：5.3125 Mbits/s
  - 时钟相位：PHASE_2EDGE
  - DMA支持（DMA1_CH1接收，DMA1_CH2发送）

**连接器件**：AS5047P磁编码器（14位高精度位置传感器）
- 读取转子位置角度
- 用于FOC算法的电角度计算
- 支持多圈位置跟踪

### SPI3
**功能：DRV8353驱动器配置和状态读取**
- **PB3**: SPI3_SCK
- **PB4**: SPI3_MISO
- **PB5**: SPI3_MOSI
- **PB0 (DRV_CS)**: DRV8353片选
- **配置**：
  - 主机模式
  - 16位数据
  - 波特率：5.3125 Mbits/s

**连接器件**：DRV8353三相栅极驱动器
- 配置驱动器参数
- 读取故障状态
- 保护功能设置

---

## 3. TIM（定时器）

### TIM1
**功能：三相PWM波生成（SVPWM）**
- **PA8 (TIM1_CH1)**: U相上桥PWM输出
- **PA9 (TIM1_CH2)**: V相上桥PWM输出
- **PA10 (TIM1_CH3)**: W相上桥PWM输出
- **配置**：
  - 中心对齐模式（适合FOC）
  - PWM周期：2125计数值
  - 重复计数器：19（每20个PWM周期触发一次ADC采样）
  - 占空比初始值：50% (1062/2125)
  - 主输出触发：TIM_TRGO_UPDATE（触发ADC1同步采样）

**用途**：
- 生成SVPWM驱动三相电机
- 触发ADC在PWM周期中点采样电流（减少纹波干扰）
- PWM频率约40kHz

### TIM5
**功能：通用定时器（可能用于速度计算或任务调度）**
- **配置**：
  - 预分频器：1699
  - 周期：99
  - 可能用于1ms或10ms定时任务

---

## 4. USART（串口通信）

### USART1 + DMA
**功能：上位机通信/调试接口**
- **PB6**: USART1_TX（发送）
- **PB7**: USART1_RX（接收）
- **配置**：
  - 异步模式
  - DMA接收支持（DMA1_CH3）
  - 空闲中断检测
  - 接收缓冲区：128字节

**用途**：
- 与上位机通信
- 参数配置
- 数据监控
- 调试输出（printf重定向）

---

## 5. FDCAN（CAN FD总线）

### FDCAN1
**功能：CAN总线通信（机器人控制网络）**
- **PA11**: FDCAN1_RX
- **PA12**: FDCAN1_TX
- **配置**：
  - 波特率：1Mbps
  - 标称位时间：1000ns
  - 时间段1：13个时间量子
  - 时间段2：3个时间量子

**用途**：
- 与主控板或其他电机控制器通信
- 实时指令接收
- 状态反馈

---

## 6. GPIO（通用输入输出）

### 控制信号
- **PC14 (DRV_ENBLE)**: DRV8353使能控制
- **PC15 (DRV_cotr)**: DRV8353 PWM模式控制
- **PB0 (DRV_CS)**: DRV8353片选
- **PB2 (AS5047P_CS)**: AS5047P片选

### LED指示灯
- **PB13 (LED1)**: 状态指示灯1（可能是故障指示）
- **PB14 (LED2)**: 状态指示灯2（可能是运行指示）

### 调试接口
- **PA13**: SWD_IO（调试数据）
- **PA14**: SWD_CLK（调试时钟）

---

## 7. DMA（直接内存访问）

### DMA1配置
- **DMA1_CH1**: SPI1_RX（AS5047P编码器数据接收）
- **DMA1_CH2**: SPI1_TX（AS5047P编码器命令发送）
- **DMA1_CH3**: USART1_RX（串口数据接收）

**用途**：减少CPU负担，提高实时性

---

## 8. RCC（时钟配置）

### 时钟树配置
- **外部晶振**: 8MHz HSE
- **PLL配置**:
  - PLLM: 分频2
  - PLLN: 倍频85
  - 系统时钟：170MHz
- **外设时钟**:
  - ADC时钟：170MHz
  - APB1/APB2：170MHz
  - FDCAN时钟：170MHz

---

## 系统架构总结

### 核心功能：无刷电机FOC控制
1. **位置感知**: AS5047P磁编码器（SPI1） → 提供转子位置
2. **电流采样**: ADC1（PA0/PA1） → 采集相电流
3. **PWM输出**: TIM1三路PWM → 驱动DRV8353 → 控制电机
4. **驱动保护**: DRV8353（SPI3） → 栅极驱动和保护
5. **通信接口**: 
   - FDCAN1 → 实时控制总线
   - USART1 → 调试和参数配置

### 控制环路
```
位置环 → 速度环 → 电流环(Id/Iq) → SVPWM → 电机
   ↑        ↑          ↑
AS5047P  微分计算   ADC采样
```

### 中断优先级
1. ADC1注入转换中断（最高优先级） - 电流环
2. TIM1更新中断 - PWM更新
3. FDCAN接收中断 - 通信
4. SPI1 DMA中断 - 位置读取
5. USART1接收中断 - 串口通信

---

## 引脚分配表

| 引脚 | 功能 | 外设 | 说明 |
|------|------|------|------|
| PA0 | ADC1_IN1 | ADC1 | U相电流采样 |
| PA1 | ADC1_IN2 | ADC1 | V相电流采样 |
| PA2 | ADC1_IN3 | ADC1 | 保留 |
| PA3 | ADC1_IN4 | ADC1 | 保留 |
| PA4 | ADC2_IN17 | ADC2 | 温度/电压采样 |
| PA5 | SPI1_SCK | SPI1 | 编码器时钟 |
| PA6 | SPI1_MISO | SPI1 | 编码器数据输入 |
| PA7 | SPI1_MOSI | SPI1 | 编码器数据输出 |
| PA8 | TIM1_CH1 | TIM1 | U相PWM |
| PA9 | TIM1_CH2 | TIM1 | V相PWM |
| PA10 | TIM1_CH3 | TIM1 | W相PWM |
| PA11 | FDCAN1_RX | FDCAN | CAN接收 |
| PA12 | FDCAN1_TX | FDCAN | CAN发送 |
| PA13 | SWDIO | DEBUG | 调试接口 |
| PA14 | SWCLK | DEBUG | 调试时钟 |
| PB0 | GPIO_Output | GPIO | DRV8353片选 |
| PB2 | GPIO_Output | GPIO | AS5047P片选 |
| PB3 | SPI3_SCK | SPI3 | DRV时钟 |
| PB4 | SPI3_MISO | SPI3 | DRV数据输入 |
| PB5 | SPI3_MOSI | SPI3 | DRV数据输出 |
| PB6 | USART1_TX | USART | 串口发送 |
| PB7 | USART1_RX | USART | 串口接收 |
| PB11 | ADC2_IN14 | ADC2 | 总线电压采样 |
| PB13 | GPIO_Output | GPIO | LED1故障灯 |
| PB14 | GPIO_Output | GPIO | LED2运行灯 |
| PC14 | GPIO_Output | GPIO | DRV使能 |
| PC15 | GPIO_Output | GPIO | DRV控制 |
| PF0 | OSC_IN | RCC | 8MHz晶振输入 |
| PF1 | OSC_OUT | RCC | 8MHz晶振输出 |

---

## 关键参数

- **PWM频率**: ~40kHz（170MHz / 2125 / 2）
- **ADC采样率**: ~2kHz（40kHz / 20）
- **编码器精度**: 14位（16384步/圈）
- **电流采样**: 双相采样（U、V相），W相通过计算得出
- **控制算法**: FOC（磁场定向控制）
- **控制模式**: 位置/速度/力矩/混合控制

