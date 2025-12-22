## STM32 学习记录

芯片:
* STM32F103C8T6
    
IDE:
* STM32CubeIDE version 1.19.0

OS:
* mac OS tahoe 26


```plainText
    |
    |-- register-base  // 最基本的寄存器学习环境示例(可复用)
    |-- register-led     // 练习一简单的寄存器亮灯示例
    |-- register-oled    // 练习二学习SPI串口协议点亮OLED屏示例(软硬件SPI实现)
    |-- register-uartlog // 练习三学习通连接接串口并将日志打印到电脑显示日志(需要USB转TTL模块)
    |-- register-u8g2.   // 练习四集成u8g2库以方便后续的学习中OLED进行显示(作为调试日志复用项目)。
    |
    |
    |
    |
    |
```




**引用三方库：**

[U8g2](https://github.com/olikraus/u8g2):各种屏显芯片库(u8g2 完全可以实现正弦波、温度变化折线图（chart）、柱状图、实时波形监测等图形显示),完美支持中文显示（内置大量 GB2312/Unicode 中文字体，如 12x12、16x16 等）。它比你之前的寄存器级驱动更高级：支持图形绘制（线、圆、矩形、位图）、多种字体（包括中文）、UTF-8 文本、滚动、翻转等，且有 全缓冲模式（Full Buffer）和 页模式（Page Mode）可选。

u8g2 只需要下载源码来下，把csrc导入项目即可使用。

[【u8g2使用手册】](https://github.com/olikraus/u8g2/wiki/u8g2reference)
[【u8x8使用手册】](https://github.com/olikraus/u8g2/wiki/u8x8reference)
[【u8log使用手册】](https://github.com/olikraus/u8g2/wiki/u8logreference)


