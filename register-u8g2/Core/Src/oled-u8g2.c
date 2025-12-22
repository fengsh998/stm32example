#include "oled-u8g2.h"
#include "clock/systick.h"
#include "stm32f103xb.h"
#include <string.h>

#define Use_Hardware_Spi	0

#if Use_Hardware_Spi
//GPIO 脚定义，使用硬件SPI， PA5是SCK PA7是MOSI
#define OLED_CS_GPIO GPIOA
#define OLED_CS_PIN 4			//PA4
#define OLED_DC_GPIO GPIOA
#define OLED_DC_PIN 3			//PA3
#define OLED_RST_GPIO GPIOA
#define OLED_RST_PIN 2			//PA2
#else
// 软件控制 SPI协议的 GPIO 引脚定义
#define OLED_CS_GPIO  GPIOA
#define OLED_CS_PIN   4
#define OLED_DC_GPIO  GPIOA
#define OLED_DC_PIN   3
#define OLED_RST_GPIO GPIOA
#define OLED_RST_PIN  2
#define OLED_SCK_GPIO GPIOA
#define OLED_SCK_PIN  5
#define OLED_MOSI_GPIO GPIOA
#define OLED_MOSI_PIN 7
#endif

//全局变量
u8g2_t u8g2;

//=========================== GPIO ========================
// GPIO 设置Pin脚高电平
void gpio_set(GPIO_TypeDef *gpio, uint16_t pin) {
	gpio->BSRR = (1 << pin);
}

// GPIO 设置Pin脚低电平
void gpio_reset(GPIO_TypeDef *gpio, uint16_t pin) {
	gpio->BSRR = (1 << (pin + 16));
}

#if Use_Hardware_Spi
void GPIO_hw_spi_init() {
	// 使能
	RCC->APB2ENR |=
	RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_SPI1EN;

	// PA5: SCK  -> AF Push-Pull 50MHz (0xB)
	// PA7: MOSI -> AF Push-Pull 50MHz (0xB)
	// PA4: CS   -> General Push-Pull 50MHz (0x3)
	// PA3: DC   -> General Push-Pull 50MHz (0x3)
	// PA2: RST  -> General Push-Pull 50MHz (0x3)
	GPIOA->CRL = (GPIOA->CRL & ~(0xF << (5 * 4))) | (0xB << (5 * 4)); // PA5 AF PP 50MHz
	GPIOA->CRL = (GPIOA->CRL & ~(0xF << (7 * 4))) | (0xB << (7 * 4)); // PA7 AF PP
	GPIOA->CRL = (GPIOA->CRL & ~(0xF << (4 * 4))) | (0x3 << (4 * 4)); // PA4 PP
	GPIOA->CRL = (GPIOA->CRL & ~(0xF << (3 * 4))) | (0x3 << (3 * 4)); // PA3 PP
	GPIOA->CRL = (GPIOA->CRL & ~(0xF << (2 * 4))) | (0x3 << (2 * 4)); // PA2 PP

	// 初始电平：CS 高（不选中），DC 高（无关），RST 高（非复位）
	gpio_set(OLED_CS_GPIO, OLED_CS_PIN);
	gpio_set(OLED_DC_GPIO, OLED_DC_PIN);
	gpio_set(OLED_RST_GPIO, OLED_RST_PIN);

	// SPI1 配置：Master, 8-bit, CPOL=0, CPHA=0, 软件NSS, 分频/8 (约9MHz @72MHz)
	SPI1->CR1 = SPI_CR1_MSTR          // 主模式
	| (3 << 3)              // BR[2:0] = 011 -> fPCLK/8
			| SPI_CR1_SSM           // 软件NSS管理
			| SPI_CR1_SSI;          // 内部从机选择=1（NSS内部高）
	SPI1->CR1 |= SPI_CR1_SPE;         // 使能SPI
}

/**
 * 硬件 SPI 通信回调（4线 SPI）
 *
 * @param msg 回调消息类型
 */
uint8_t u8x8_byte_4wire_hw_spi_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
		void *arg_ptr) {
	switch (msg) {
	case U8X8_MSG_BYTE_INIT: /*初始化函数*/
		GPIO_hw_spi_init(); // SPI 和 GPIO 初始化
		break;
	case U8X8_MSG_BYTE_SET_DC: /*设置DC引脚,表明发送的是数据还是命令*/
		u8x8_gpio_SetDC(u8x8, arg_int); // 直接调用(GPIO_DELAY)回调中的DC处理
		break;
	case U8X8_MSG_BYTE_SEND: { /*通过SPI发送arg_int个字节数据*/

		//写法一：
//		uint8_t *data = (uint8_t*) arg_ptr;
//		while (arg_int > 0) {
//			while (!(SPI1->SR & SPI_SR_TXE))
//				;   // 等待发送缓冲空
//			SPI1->DR = *data;					// 写入数据
//			while (!(SPI1->SR & SPI_SR_RXNE))
//				;  // 等待接收完成（SPI 需要读以清标志）
//			(void) SPI1->DR;                    // 丢弃接收数据
//			data++;
//			arg_int--;
//		}
		//写法二：
		uint8_t *data = (uint8_t*) arg_ptr;
		while (arg_int > 0) {
			while (!(SPI1->SR & SPI_SR_TXE))
				; // 等待发送缓冲区空
			SPI1->DR = *data;
			// 等待数据发送完成（不仅仅是进入移位寄存器）
			while (SPI1->SR & SPI_SR_BSY)
				;
			data++;
			arg_int--;
		}
	}
		break;

	case U8X8_MSG_BYTE_START_TRANSFER:   // 开始一帧传输，拉低CS
		// 触发GPIO_DELAY回调中设置
		u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_enable_level);
		break;

	case U8X8_MSG_BYTE_END_TRANSFER:     // 结束传输，拉高CS
		// 触发GPIO_DELAY回调中设置
		u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_disable_level);
		break;
	default:
		return 0;
	}
	return 1;
}

/**
 *  GPIO_DELAY延时回调
 */
uint8_t u8x8_gpio_and_delay_stm32(U8X8_UNUSED u8x8_t *u8x8,
U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
U8X8_UNUSED void *arg_ptr) {
	switch (msg) {
	case U8X8_MSG_GPIO_AND_DELAY_INIT: /*delay和GPIO的初始化*/
		break;
	case U8X8_MSG_DELAY_MILLI: /*延时函数*/
		delay_ms(arg_int);    // 调用谁mcu系统延时函数
		break;
	case U8X8_MSG_DELAY_NANO:           // 纳秒延时（SSD1306 需要极短等待）
		// 粗略实现：根据系统时钟大致延时，arg_int 为纳秒数
		// 对于72MHz，1次循环约几ns，可根据需要调整
		for (volatile uint32_t i = 0; i < (arg_int / 20); i++) {
			__NOP();
		}
		break;
	case U8X8_MSG_GPIO_CS: /*片选信号*/
		if (arg_int) {
			gpio_set(OLED_CS_GPIO, OLED_CS_PIN);
		} else {
			gpio_reset(OLED_CS_GPIO, OLED_CS_PIN);
		}
		break;
	case U8X8_MSG_GPIO_DC: /*设置DC引脚,表明发送的是数据还是命令*/
		if (arg_int) {
			gpio_set(OLED_DC_GPIO, OLED_DC_PIN);
		} else {
			gpio_reset(OLED_DC_GPIO, OLED_DC_PIN);
		}
		break;
	case U8X8_MSG_GPIO_RESET: /*复位引脚控制（重要！）*/
		if (arg_int) {
			gpio_set(OLED_RST_GPIO, OLED_RST_PIN); // 高电平（非复位）
		} else {
			gpio_reset(OLED_RST_GPIO, OLED_RST_PIN); // 低电平复位
		}
		break;
	}
	return 1;
}

void oled_u8g2_init(void) {
	/********************************************
	 U8G2_R0     //不旋转，不镜像
	 U8G2_R1     //旋转90度
	 U8G2_R2     //旋转180度
	 U8G2_R3     //旋转270度
	 U8G2_MIRROR   //没有旋转，横向显示左右镜像
	 U8G2_MIRROR_VERTICAL    //没有旋转，竖向显示镜像
	 ********************************************/
	u8g2_Setup_ssd1306_128x64_noname_f(&u8g2, U8G2_R0,  // R0: 不旋转
			u8x8_byte_4wire_hw_spi_stm32,  // SPI 回调
			u8x8_gpio_and_delay_stm32);    // GPIO/延时回调

	u8g2_InitDisplay(&u8g2);     // 初始化显示器
	u8g2_SetPowerSave(&u8g2, 0); // 关闭省电模式
	u8g2_ClearBuffer(&u8g2);     // 清缓冲
}
#else
//================================== 软件实现SPI 协议 ===============================
void GPIO_soft_spi_init() {
    // 1. 使能 GPIOA 时钟
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    // 2. 配置 PA2, PA3, PA4, PA5, PA7 为通用推挽输出 (50MHz)
    // 每个引脚占用 4 位，50MHz 推挽输出对应的配置值是 0x3
    GPIOA->CRL &= ~(0xF << (2 * 4)); GPIOA->CRL |= (0x3 << (2 * 4)); // PA2 RES
    GPIOA->CRL &= ~(0xF << (3 * 4)); GPIOA->CRL |= (0x3 << (3 * 4)); // PA3 DC
    GPIOA->CRL &= ~(0xF << (4 * 4)); GPIOA->CRL |= (0x3 << (4 * 4)); // PA4 CS
    GPIOA->CRL &= ~(0xF << (5 * 4)); GPIOA->CRL |= (0x3 << (5 * 4)); // PA5 SCK
    GPIOA->CRL &= ~(0xF << (7 * 4)); GPIOA->CRL |= (0x3 << (7 * 4)); // PA7 MOSI

    // 3. 初始电平设置
    gpio_set(OLED_CS_GPIO, OLED_CS_PIN);
    gpio_set(OLED_RST_GPIO, OLED_RST_PIN);
    gpio_reset(OLED_SCK_GPIO, OLED_SCK_PIN);
}

// 软件编程发送字节数据
void SW_SPI_SendByte(uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        // 数据准备：设置 MOSI 引脚
        if (byte & 0x80) {
            gpio_set(OLED_MOSI_GPIO, OLED_MOSI_PIN);
        } else {
            gpio_reset(OLED_MOSI_GPIO, OLED_MOSI_PIN);
        }

        // 时钟脉冲：拉高 SCK，然后再拉低
        gpio_set(OLED_SCK_GPIO, OLED_SCK_PIN);
        __NOP(); // 极短延时，确保 OLED 识别到高电平
        gpio_reset(OLED_SCK_GPIO, OLED_SCK_PIN);

        byte <<= 1; // 移位处理下一位
    }
}

uint8_t u8x8_gpio_and_delay_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_DELAY_MILLI:
            delay_ms(arg_int);
            break;
        case U8X8_MSG_GPIO_CS:
            if (arg_int) gpio_set(OLED_CS_GPIO, OLED_CS_PIN);
            else gpio_reset(OLED_CS_GPIO, OLED_CS_PIN);
            break;
        case U8X8_MSG_GPIO_DC:
            if (arg_int) gpio_set(OLED_DC_GPIO, OLED_DC_PIN);
            else gpio_reset(OLED_DC_GPIO, OLED_DC_PIN);
            break;
        case U8X8_MSG_GPIO_RESET:
            if (arg_int) gpio_set(OLED_RST_GPIO, OLED_RST_PIN);
            else gpio_reset(OLED_RST_GPIO, OLED_RST_PIN);
            break;
    }
    return 1;
}

uint8_t u8x8_byte_sw_spi_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_BYTE_INIT:
            GPIO_soft_spi_init();
            break;
        case U8X8_MSG_BYTE_SEND: {
            uint8_t *data = (uint8_t *)arg_ptr;
            while (arg_int--) {
                SW_SPI_SendByte(*data++);
            }
            break;
        }
        case U8X8_MSG_BYTE_START_TRANSFER:
            gpio_reset(OLED_CS_GPIO, OLED_CS_PIN); // 开始传输，CS 拉低
            break;
        case U8X8_MSG_BYTE_END_TRANSFER:
            gpio_set(OLED_CS_GPIO, OLED_CS_PIN);   // 结束传输，CS 拉高
            break;
        case U8X8_MSG_BYTE_SET_DC:
            if (arg_int) gpio_set(OLED_DC_GPIO, OLED_DC_PIN);
            else gpio_reset(OLED_DC_GPIO, OLED_DC_PIN);
            break;
        default:
            return 0;
    }
    return 1;
}

void oled_u8g2_init(void) {
    u8g2_Setup_ssd1306_128x64_noname_f(&u8g2, U8G2_R0,
            u8x8_byte_sw_spi_stm32,
            u8x8_gpio_and_delay_stm32);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
    u8g2_ClearBuffer(&u8g2);
}

#endif



void clear() {
	u8g2_ClearBuffer(&u8g2);
}

// 用可来输出日志
void oled_log(uint8_t line, const char* format, ...) {
    char buffer[32];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    // 设置字体并清除该行区域（可选）或者直接覆盖
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    // line 从 1 开始，每行 12 像素高
    u8g2_DrawStr(&u8g2, 0, line * 12, buffer);
    u8g2_SendBuffer(&u8g2);
}

//========================================== DEMO 部分 ========================================
// 示例一：hello world
void testHelloWorld() {
	u8g2_ClearBuffer(&u8g2);
	//设置字体
	u8g2_SetFont(&u8g2, u8g2_font_logisoso16_tf);
	//计算文字宽度，实现水平居中
	const char *text = "Hello World";
	uint16_t text_width = u8g2_GetStrWidth(&u8g2, text);
	uint8_t x = (128 - text_width) / 2;
	uint8_t y = 35;
	//绘制字符串
	u8g2_DrawStr(&u8g2, x, y, text);
	//发送缓冲区内容到 OLED 显示
	u8g2_SendBuffer(&u8g2);
}


// 示例二： 折线图
#define CHART_WIDTH 100
#define CHART_HEIGHT 40
#define CHART_X_OFFSET 20
#define CHART_Y_OFFSET 50

uint8_t data_buffer[CHART_WIDTH]; // 存储最近100个数据点

// 模拟数据更新
void update_data(uint8_t new_val) {
    // 将数据左移一位
    for (int i = 0; i < CHART_WIDTH - 1; i++) {
        data_buffer[i] = data_buffer[i + 1];
    }
    data_buffer[CHART_WIDTH - 1] = new_val % CHART_HEIGHT; // 限制在图表高度内
}

void draw_chart_demo() {
    u8g2_ClearBuffer(&u8g2);

    // 1. 画坐标轴
    u8g2_DrawVLine(&u8g2, CHART_X_OFFSET, CHART_Y_OFFSET - CHART_HEIGHT, CHART_HEIGHT + 2); // Y轴
    u8g2_DrawHLine(&u8g2, CHART_X_OFFSET, CHART_Y_OFFSET, CHART_WIDTH);                     // X轴

    // 2. 绘制折线
    for (int i = 0; i < CHART_WIDTH - 1; i++) {
        // U8G2 坐标系 Y轴向下为正，所以要用 偏移量 减去 数值
        u8g2_DrawLine(&u8g2,
            CHART_X_OFFSET + i, CHART_Y_OFFSET - data_buffer[i],
            CHART_X_OFFSET + i + 1, CHART_Y_OFFSET - data_buffer[i + 1]
        );
    }

    // 3. 画文字标签
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    u8g2_DrawStr(&u8g2, 0, 20, "Sensor:");

    u8g2_SendBuffer(&u8g2);
}

//示例3：柱状图（Bar Chart）
void draw_bar_chart(void) {
    u8g2_ClearBuffer(&u8g2);

    int values[] = {20, 35, 50, 30, 60, 45};
    int num_bars = 6;
    int bar_width = 15;
    int spacing = 6;

    for (int i = 0; i < num_bars; i++) {
        int x = 10 + i * (bar_width + spacing);
        int height = values[i] * 0.8;  // 缩放
        u8g2_DrawBox(&u8g2, x, 64 - height, bar_width, height);
    }

    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    u8g2_DrawStr(&u8g2, 2, 10, "Bar Chart");

    u8g2_SendBuffer(&u8g2);
}

//示例4：绘制正弦波（Sine Wave）
void draw_sine_wave(void) {
    u8g2_ClearBuffer(&u8g2);

    // 绘制坐标轴
    u8g2_DrawHLine(&u8g2, 0, 32, 128);  // 水平轴（Y=32 为零点）
    u8g2_DrawVLine(&u8g2, 0, 0, 64);    // 垂直轴

    // 绘制正弦波（幅度 30 像素，垂直居中偏移 32）
    for (int x = 0; x < 127; x++) {
        int y = 32 + (int)(30 * sin( (x * 3.14159 * 4) / 128 ));  // 4个周期
        u8g2_DrawPixel(&u8g2, x, y);
    }

    // 可选：加网格
    for (int i = 0; i < 128; i += 16) {
        u8g2_DrawVLine(&u8g2, i, 0, 64);
    }
    for (int i = 0; i < 64; i += 8) {
        u8g2_DrawHLine(&u8g2, 0, i, 128);
    }

    // 加文字说明
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    u8g2_DrawStr(&u8g2, 2, 10, "Sine Wave Demo");

    u8g2_SendBuffer(&u8g2);
}

//示例5：实时温度变化折线图（监测曲线）
float temp_data[DATA_POINTS];  // 存储温度历史（可从 ADC 获取）
int data_index = 0;
void update_temperature_chart(float new_temp) {
    // 更新数据（环形缓冲）
    temp_data[data_index] = new_temp;
    data_index = (data_index + 1) % DATA_POINTS;

    u8g2_ClearBuffer(&u8g2);

    // 绘制边框和网格
    u8g2_DrawFrame(&u8g2, 0, 10, 128, 54);  // 图表区域
    u8g2_DrawHLine(&u8g2, 0, 32, 128);      // 零线（可代表 25°C）
    u8g2_DrawStr(&u8g2, 2, 8, "Temp Monitor");

    // 绘制折线
    for (int i = 1; i < DATA_POINTS; i++) {
        int idx1 = (data_index + i - 1) % DATA_POINTS;
        int idx2 = (data_index + i) % DATA_POINTS;

        int x1 = i - 1;
        int x2 = i;
        int y1 = 62 - (int)((temp_data[idx1] - 10) * 1.5);  // 缩放：10~40°C → 像素
        int y2 = 62 - (int)((temp_data[idx2] - 10) * 1.5);

        // 限制 Y 范围
        if (y1 < 10) y1 = 10;
        if (y1 > 62) y1 = 62;
        if (y2 < 10) y2 = 10;
        if (y2 > 62) y2 = 62;

        u8g2_DrawLine(&u8g2, x1, y1, x2, y2);
    }

    // 显示当前温度
    char buf[20];
    //sprintf(buf, "%.1f C", new_temp);
    u8g2_SetFont(&u8g2, u8g2_font_10x20_tf);
    u8g2_DrawStr(&u8g2, 40, 58, buf);

    u8g2_SendBuffer(&u8g2);
}

