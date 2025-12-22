/*
 *
 *      示例一: 第一行跑马灯
 	 	oled_clear();

		// 第一行写超长文本（前后加空格更美观）
		oled_set_position(0, 0);
		my_printf("    Welcome to STM32 OLED! Long scrolling message demo...    ");

		// 其他行写固定内容
		oled_set_position(0, 2);
		my_printf("Temperature: 25.6 C");
		oled_set_position(0, 4);
		my_printf("Humidity:    60%");
		oled_set_position(0, 6);
		my_printf("Status:      OK");

		// 只滚动第一行（page 0）
		oled_set_vertical_scroll_area(8, 48);        // 顶部固定1行（8像素），剩余可滚动
		oled_scroll_left(0, 0, 0x07);                // 左滚，最快速度
 *
 *      示例二: 全屏左滚动
        oled_clear();
		my_printf("Line 1: Hello");
		my_printf("Line 2: STM32");
		my_printf("Line 3: SSD1306");
		my_printf("Line 4: Scroll Demo");
		my_printf("Line 5: 2025-12-19");
		my_printf("Line 6: Running...");
		my_printf("Line 7: End");

		oled_scroll_left(0, 7, 0x00);  // 全屏左滚，中等速度
 *
 *
 *		示例3：右滚动 + 停止

 		copy示例二的前面部分代码到这

 		oled_scroll_right(0, 7, 0x07);  // 全屏右滚，最快

		delay_ms(10000);

		oled_scroll_stop();            // 10秒后停止
 *
 *
 *		示例4：对角滚动（酷炫动画）
 *
 *		...
 *		oled_scroll_diag_left(0, 7, 0x07, 10);  // 左+向下滚动，垂直偏移10
 *
 */


#include "oled/spi-oled.h"
#include "oled/oled-ssd1306.h"


// va_list for my_printf
typedef __builtin_va_list va_list;
#define va_start(ap, param) __builtin_va_start(ap, param)
#define va_end(ap) __builtin_va_end(ap)
#define va_arg(ap, type) __builtin_va_arg(ap, type)

// 光标位置初始为左上(0,0)
static int cursor_x = 0;
static int cursor_y = 0;

//================================= SPI协议通讯 ================================
// GPIO 设置Pin脚高电平
void gpio_set(GPIO_TypeDef *gpio, uint16_t pin) {
    gpio->BSRR = (1 << pin);
}

// GPIO 设置Pin脚低电平
void gpio_reset(GPIO_TypeDef *gpio, uint16_t pin) {
    gpio->BSRR = (1 << (pin + 16));
}

#if USE_HARDWARE_SPI
// 硬件SPI协议发送byte
void spi_send_byte(uint8_t byte) {
    while (!(SPI1->SR & SPI_SR_TXE)); // 等待 TX empty
    SPI1->DR = byte;
    while (!(SPI1->SR & SPI_SR_RXNE)); // 等待 RX not empty
    (void)SPI1->DR; // Dummy read
}
#else
// 软件 SPI send byte (MSB first, CPOL=0, CPHA=0)
void spi_send_byte(uint8_t byte) {
    for (int i = 7; i >= 0; i--) {
        gpio_reset(OLED_SCK_GPIO, OLED_SCK_PIN); // SCK low
        if (byte & (1 << i)) {
            gpio_set(OLED_MOSI_GPIO, OLED_MOSI_PIN); // MOSI 1
        } else {
            gpio_reset(OLED_MOSI_GPIO, OLED_MOSI_PIN); // MOSI 0
        }
        gpio_set(OLED_SCK_GPIO, OLED_SCK_PIN); // SCK high (data sampled)
    }
    gpio_reset(OLED_SCK_GPIO, OLED_SCK_PIN); // Idle low
}
#endif

// Send command to OLED
void oled_command(uint8_t cmd) {
    gpio_reset(OLED_DC_GPIO, OLED_DC_PIN); // DC low for command
    gpio_reset(OLED_CS_GPIO, OLED_CS_PIN); // CS low
    spi_send_byte(cmd);
    gpio_set(OLED_CS_GPIO, OLED_CS_PIN); // CS high
}

// Send data to OLED
void oled_data(uint8_t data) {
    gpio_set(OLED_DC_GPIO, OLED_DC_PIN); // DC high for data
    gpio_reset(OLED_CS_GPIO, OLED_CS_PIN); // CS low
    spi_send_byte(data);
    gpio_set(OLED_CS_GPIO, OLED_CS_PIN); // CS high
}

// Set OLED page and column for writing
void oled_set_position(uint8_t column, uint8_t page) {
    oled_command(0xB0 + page); // Set page
    oled_command(column & 0x0F); // Lower column
    oled_command(0x10 + (column >> 4)); // Higher column
}

//=================================== 便捷操作函数 ===================================

// 清屏
void oled_clear() {
    for (uint8_t page = 0; page < 8; page++) {
        oled_set_position(0, page);
        for (uint8_t col = 0; col < OLED_WIDTH; col++) {
            oled_data(0x00);
        }
    }
}

/**
 * 			将整个 OLED 屏幕全部点亮（全白）
 * 			SSD1306 有 8 页（每页 8 行）
 */
void oled_fill_white() {
    for (uint8_t page = 0; page < 8; page++) {
    	// 设置当前页起始位置
        oled_set_position(0, page);
        for (uint8_t col = 0; col < OLED_WIDTH; col++) {  // 128 列
            oled_data(0xFF); // 每列写 0xFF（该页 8 行全部点亮）
        }
    }
}

// 显示一个字符
void oled_write_char(char ch) {
    if (ch < ' ' || ch > '~') ch = ' '; // Invalid char
    uint8_t idx = ch - ' ';
    if (cursor_x + 8 > OLED_WIDTH) {
        cursor_x = 0;
        cursor_y++;
    }
    if (cursor_y >= OLED_HEIGHT / 8) {
        cursor_y = 0; // Simple wrap
        oled_clear();
    }
    oled_set_position(cursor_x, cursor_y);
    for (uint8_t i = 0; i < 8; i++) {
        oled_data(font8x8[idx][i]);
    }
    cursor_x += 8;
}

// 显示字符串
void oled_write_string(const char *str) {
    while (*str) {
        oled_write_char(*str++);
    }
}

// 将整数转换为字符串(只支持正整数)
void itoa(int num, char *str) {
    int i = 0;
    do {
        str[i++] = num % 10 + '0';
        num /= 10;
    } while (num > 0);
    str[i] = '\0';
    // Reverse
    for (int j = 0; j < i / 2; j++) {
        char temp = str[j];
        str[j] = str[i - j - 1];
        str[i - j - 1] = temp;
    }
}

// 自定义 printf (basic %c %s %d)
void my_printf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    char buf[16]; // For numbers
    while (*fmt) {
        if (*fmt == '%') {
            fmt++;
            if (*fmt == 'c') {
                char c = va_arg(ap, int);
                oled_write_char(c);
            } else if (*fmt == 's') {
                const char *s = va_arg(ap, const char*);
                oled_write_string(s);
            } else if (*fmt == 'd') {
                int d = va_arg(ap, int);
                itoa(d, buf);
                oled_write_string(buf);
            }
        } else {
            oled_write_char(*fmt);
        }
        fmt++;
    }
    va_end(ap);
}

//=============================================== OLED滚动函数 ============================================
// 停止当前滚动
void oled_scroll_stop(void) {
    oled_command(0x2E);  // Deactivate scroll
}

/**
 *  				speed 取值
 *  	value     帧间隔        速度说明
 *		0x00       5帧 	        中等
 *		0x01       64帧 	        慢
 *		0x02       128帧 	    更慢
 *		0x03       256帧 	    最慢
 *		0x04       3帧 	        快
 *		0x05       4帧 	        中等偏快
 *		0x06       25帧 	        中等
 *		0x07       2帧 	        最快
 *
 */
// 水平右滚动（Right Horizontal Scroll）
void oled_scroll_right(uint8_t start_page, uint8_t end_page, uint8_t speed) {
    oled_scroll_stop();                 // 先停止当前滚动

    oled_command(0x26);                 // Right Horizontal Scroll
    oled_command(0x00);                 // Dummy byte
    oled_command(start_page);           // Start page address (0-7)
    oled_command(speed);                // Set frame interval (速度，见下表)
    oled_command(end_page);             // End page address (0-7)
    oled_command(0x00);                 // Dummy byte
    oled_command(0xFF);                 // Dummy byte

    oled_command(0x2F);                 // Activate scroll
}

// 水平左滚动（Left Horizontal Scroll）
void oled_scroll_left(uint8_t start_page, uint8_t end_page, uint8_t speed) {
    oled_scroll_stop();

    oled_command(0x27);                 // Left Horizontal Scroll
    oled_command(0x00);
    oled_command(start_page);
    oled_command(speed);
    oled_command(end_page);
    oled_command(0x00);
    oled_command(0xFF);

    oled_command(0x2F);
}

// 设置垂直滚动区域（用于部分区域滚动，顶部/底部固定）
void oled_set_vertical_scroll_area(uint8_t top_fixed_rows, uint8_t scroll_rows) {
    oled_command(0xA3);                 // Set Vertical Scroll Area
    oled_command(top_fixed_rows);       // 顶部固定行数（像素行数 = 行数 × 8）
    oled_command(scroll_rows);          // 可滚动行数（像素行数）
}

// 垂直 + 水平滚动（高级效果，常用于全屏平移）
void oled_scroll_diag_right(uint8_t start_page, uint8_t end_page, uint8_t speed, uint8_t vertical_offset) {
    oled_scroll_stop();

    oled_set_vertical_scroll_area(0, OLED_HEIGHT);  // 全屏垂直可滚动

    oled_command(0x29);                 // Vertical and Right Horizontal Scroll
    oled_command(0x00);
    oled_command(start_page);
    oled_command(speed);
    oled_command(end_page);
    oled_command(vertical_offset);      // 垂直偏移量（1~63）

    oled_command(0x2F);
}

void oled_scroll_diag_left(uint8_t start_page, uint8_t end_page, uint8_t speed, uint8_t vertical_offset) {
    oled_scroll_stop();

    oled_set_vertical_scroll_area(0, OLED_HEIGHT);

    oled_command(0x2A);                 // Vertical and Left Horizontal Scroll
    oled_command(0x00);
    oled_command(start_page);
    oled_command(speed);
    oled_command(end_page);
    oled_command(vertical_offset);

    oled_command(0x2F);
}

// OLED initialization sequence
void oled_init() {
	// 硬件复位（如果有 RST 引脚）
    gpio_reset(OLED_RST_GPIO, OLED_RST_PIN); // RST 拉低
    delay_ms(10);							 // 保持低电平至少 10ms
    gpio_set(OLED_RST_GPIO, OLED_RST_PIN);   // RST 拉高
    delay_ms(10);							 // 复位完成后稍作延迟

    oled_command(0xAE); // 显示关闭(Display off)
    // 设置内存寻址模式：水平寻址模式（Horizontal Addressing Mode）
    oled_command(0x20); //Set Memory Addressing Mode
    oled_command(0x00); // 0x00: Horizontal, 0x01: Vertical, 0x02: Page
    // 设置列地址复位（重要！确保从左上角开始）
    oled_command(0x00); //Set Lower Column Start Address
    oled_command(0x10); //Set Higher Column Start Address
    // 设置起始行地址
    oled_command(0x40); // Set Display Start Line (0x40 ~ 0x7F)
    // 设置段重映射（Segment Remap）—— 0xA0 正常，0xA1 翻转180度
    oled_command(0xA1); // A1: 列地址 127 映射到 SEG0（翻转显示）
    // 设置 COM 输出扫描方向 —— 0xC0 正常，0xC8 上下翻转
    oled_command(0xC8); // COM 输出扫描方向从 COM[N-1] 到 COM0（翻转）
    // 设置 COM 引脚硬件配置（128x64 必须用 0x12）
    oled_command(0xDA); // Set COM Pins Hardware Configuration
    oled_command(0x12); //0x02: Sequential (for 128x32), 0x12: Alternative (for 128x64)
    // 设置对比度（0x00 ~ 0xFF，越大越亮）
    oled_command(0x81); // Set Contrast Control
    oled_command(0x8F); // 推荐值：0x7F ~ 0xCF，我这里用 0x8F 中等偏亮（可自行调整）
    // 显示模式：0xA6 正常显示，0xA7 反显
    oled_command(0xA6);
    // 多路复用比率（Multiplex Ratio）
    oled_command(0xA8); // Set multiplex Ratio
    oled_command(0x3F); // 63 (1/64 duty, for 128x64)
    // 显示偏移（Display Offset）
    oled_command(0xD3); // Set Display Offset
    oled_command(0x00); // No offset
    // 显示时钟分频与振荡器频率
    oled_command(0xD5); // Set Display Clock Divide Ratio/Oscillator Frequency
    oled_command(0x80); // 默认值，频率适中
    // 预充电周期（Pre-charge Period）
    oled_command(0xD9); // Set Pre-charge Period
    oled_command(0xF1); // Phase 1: 15 clocks, Phase 2: 1 clock（常见推荐值）
    // VCOMH 去选择电平
    oled_command(0xDB); // Set VCOMH Deselect Level
    oled_command(0x40); // ~0.83 x VCC（推荐值）
    // 输出跟随 RAM 内容显示（Resume）
    oled_command(0xA4); // Entire Display ON: 0xA4 跟随 RAM，0xA5 强制全亮
    // 启用内置电荷泵（Charge Pump）
    oled_command(0x8D); // Charge Pump Setting
    oled_command(0x14); // 0x14: Enable, 0x10: Disable
    // 最后打开显示
    oled_command(0xAF); // Display on
    // 稍作延迟，确保初始化完成
    delay_ms(100);
    // 清屏（全黑），并复位光标位置
    oled_clear();
}

#if USE_HARDWARE_SPI
// MCU init
void mcu_init() {
    // Enable clocks
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_SPI1EN;

    // GPIO config: PA5 SCK AF PP, PA7 MOSI AF PP, PA4 CS PP, PA3 DC PP, PA2 RST PP
    GPIOA->CRL = (GPIOA->CRL & ~(0xF << (5*4))) | (0xB << (5*4)); // PA5 AF PP 50MHz
    GPIOA->CRL = (GPIOA->CRL & ~(0xF << (7*4))) | (0xB << (7*4)); // PA7 AF PP
    GPIOA->CRL = (GPIOA->CRL & ~(0xF << (4*4))) | (0x3 << (4*4)); // PA4 PP
    GPIOA->CRL = (GPIOA->CRL & ~(0xF << (3*4))) | (0x3 << (3*4)); // PA3 PP
    GPIOA->CRL = (GPIOA->CRL & ~(0xF << (2*4))) | (0x3 << (2*4)); // PA2 PP

    gpio_set(OLED_CS_GPIO, OLED_CS_PIN); // CS high
    gpio_set(OLED_DC_GPIO, OLED_DC_PIN); // DC high

    // SPI1 config: Master, 8bit, CPOL=0, CPHA=0, baud /8
    SPI1->CR1 = SPI_CR1_MSTR | (3 << 3) | SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1->CR1 |= SPI_CR1_SPE; // Enable SPI
}
#else
// MCU init (no SPI peripheral)
void mcu_init() {
    // Enable clocks
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    // GPIO config: All PP 50MHz
    GPIOA->CRL = (GPIOA->CRL & ~(0xF << (5*4))) | (0x3 << (5*4)); // PA5 SCK PP
    GPIOA->CRL = (GPIOA->CRL & ~(0xF << (7*4))) | (0x3 << (7*4)); // PA7 MOSI PP
    GPIOA->CRL = (GPIOA->CRL & ~(0xF << (4*4))) | (0x3 << (4*4)); // PA4 CS PP
    GPIOA->CRL = (GPIOA->CRL & ~(0xF << (3*4))) | (0x3 << (3*4)); // PA3 DC PP
    GPIOA->CRL = (GPIOA->CRL & ~(0xF << (2*4))) | (0x3 << (2*4)); // PA2 RST PP

    gpio_set(OLED_CS_GPIO, OLED_CS_PIN); // CS high
    gpio_set(OLED_DC_GPIO, OLED_DC_PIN); // DC high
    gpio_reset(OLED_SCK_GPIO, OLED_SCK_PIN); // SCK low idle
    gpio_reset(OLED_MOSI_GPIO, OLED_MOSI_PIN); // MOSI low
}
#endif
