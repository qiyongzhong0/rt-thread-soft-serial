# Soft serial

## 1.简介

**Soft serial** 是利用硬件定时器捕获/比较功能实现软件模拟串口的软件包。

### 1.1目录结构

`Soft serial` 软件包目录结构如下所示：

``` 
soft_serial
├───inc                             // 头文件目录
│   |   soft_serial.h               // API接口头文件
│   |   soft_serial_stm32f103rc.h   // 驱动配置头文件，芯片stm32f103rc
│   └───soft_serial_stm32f103ze.h   // 驱动配置头文件，芯片stm32f103ze
├───src                             // 源码目录
│   └───soft_serial.c               // 示例模块
│   license                         // 软件包许可证
│   readme.md                       // 软件包使用说明
└───SConscript                      // RT-Thread 默认的构建脚本
```

### 1.2许可证

Soft serial package 遵循 LGPLv2.1 许可，详见 `LICENSE` 文件。

### 1.3依赖

- RT_Thread 4.0

## 2.使用

### 2.1接口函数说明

#### int ss_scoms_init(void);
- 功能 ：软件串口初始化，默认情况下此函数由系统自动调用，用户可不用显式调用
- 参数 ：无
- 返回 ：0--成功，其它--错误

### 2.2特别说明
#### 软件串口初始化函数`int ss_scoms_init(void)`，默认情况下由系统自动调用，用户不必调用，只有在关闭系统自动初始化功能情况下，才须用户显式调用。

### 2.3获取组件

- **方式1：**
通过 *Env配置工具* 或 *RT-Thread studio* 开启软件包，根据需要配置各项参数；配置路径为 *RT-Thread online packages -> peripherals packages -> soft serial* 


### 2.4配置参数说明

| 参数宏 | 说明 |
| ---- | ---- |
| SS_TIM_CLK_FREQ 			| 硬件定时器工作时钟频率，注：此频率与系统时钟频率共同决定了软件串口波特率上限
| SS_TIM_PCLK1_MUL 			| APB1时钟外设的时钟倍频系数
| SS_TIM_PCLK2_MUL 			| APB2时钟外设的时钟倍频系数
| SS_CHK_RX_BEGIN_BIT 		| 使用接收开始位检测，当波特率较高时，不推荐开启
| SS_CHK_RX_PARITY 			| 使用接收校验位检测，不推荐开启

## 3. 联系方式

* 维护：qiyongzhong
* 主页：https://github.com/qiyongzhong0/rt-thread-soft-serial
* 主页：https://gitee.com/qiyongzhong0/rt-thread-soft-serial
* 邮箱：917768104@qq.com
