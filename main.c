#include "stm32f4xx.h"
#include <stdio.h>

// Địa chỉ I2C của LCD
#define LCD_ADDR (0x27 << 1)

// Định nghĩa các chân của LCD
#define LCD_RS            0x01
#define LCD_EN            0x04
#define LCD_BL            0x08
#define LCD_COMMAND       0x00
#define LCD_DATA          0x01
#define LCD_CLEAR         0x01
#define LCD_HOME          0x02
#define LCD_FUNCTION_SET  0x28
#define LCD_DISPLAY_ON    0x0C
#define LCD_ENTRY_MODE    0x06

// Định nghĩa các chân của HC-SR04
#define TRIG_PIN 0  // Chân PA0
#define ECHO_PIN 1  // Chân PA1

uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;
uint8_t Distance = 0;

uint8_t system_active = 0; // Trạng thái hệ thống
uint32_t prev_status_pc11 = 1;
uint32_t prev_status_pc10 = 1;

uint8_t obstacle = 0;

void I2C1_Init(void);
void GPIO_Init(void);
void LCD_Init(void);
void LCD_Write(uint8_t data, uint8_t mode);
void LCD_SendCommand(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_Print(const char* str);
void delay_ms(uint32_t ms);
void Timer2_Init(void);
void Timer3_Init(void);
void delay_us(uint16_t time);
uint8_t HCSR04_GetDis(void);

int main(void) {
    GPIO_Init();
    I2C1_Init();
    Timer2_Init();
    Timer3_Init();
    delay_ms(100);
    LCD_Init();
    LCD_SendCommand(LCD_CLEAR);
    LCD_Print("Distance:");

    while (1) {
        // Đọc trạng thái các nút
        uint32_t current_status_pc11 = (GPIOC->IDR >> 11) & 0x01; // PC11
        uint32_t current_status_pc10 = (GPIOC->IDR >> 10) & 0x01; // PC10

        // Nút PC11: Toggle hệ thống (phải)
        if (prev_status_pc11 == 1 && current_status_pc11 == 0) {
            while ((GPIOC->IDR >> 11) & 0x01 == 0); // Chờ nút nhả
            system_active = !system_active; // Đổi trạng thái hệ thống
        }

        // Nút PC10: Tắt hệ thống (trái)
        if (prev_status_pc10 == 1 && current_status_pc10 == 0) {
            while ((GPIOC->IDR >> 10) & 0x01 == 0); // Chờ nút nhả
            system_active = 0; // Tắt hệ thống
            GPIOC->ODR &= ~(1 << 2); // Tắt LED PC2
            GPIOC->ODR &= ~(1 << 3); // Tắt LED PC3
        }

        // Lưu trạng thái trước
        prev_status_pc11 = current_status_pc11;
        prev_status_pc10 = current_status_pc10;

        if (system_active) {
            // Đo khoảng cách
            Distance = HCSR04_GetDis();

            // Hiển thị khoảng cách lên LCD
            LCD_SendCommand(0xC0); // Chuyển xuống dòng thứ hai
            char buffer[16];
            snprintf(buffer, sizeof(buffer), "%3d cm", Distance);
            LCD_Print(buffer);

            // Tính tần số nhấp nháy
            uint32_t freq = (Distance < 2) ? 10 : (Distance > 10 ? 2 : 12 - Distance);
            uint32_t delay = 500 / freq; // Chia đôi chu kỳ (ms)

            // Nhấp nháy đèn PC2
            GPIOC->BSRR = (1 << 2);  // Bật đèn
            delay_ms(delay);
            GPIOC->BSRR = (1 << (2 + 16)); // Tắt đèn
            delay_ms(delay);
        } else {
            // Tắt tất cả đèn nếu hệ thống không hoạt động
            GPIOC->ODR &= ~(1 << 2); // Tắt LED PC2
            GPIOC->ODR &= ~(1 << 3); // Tắt LED PC3
        }
    }
}

void GPIO_Init(void) {
    // Bật clock GPIOB và GPIOA
    RCC->AHB1ENR |= (1 << 1); // GPIOB
    RCC->AHB1ENR |= (1 << 0); // GPIOA

    // Cấu hình GPIOB cho I2C (PB8 và PB9)
    GPIOB->MODER &= ~(0xF << 16);        // Reset PB8 và PB9
    GPIOB->MODER |= (0xA << 16);         // Alternate Function Mode
    GPIOB->OTYPER |= (1 << 8) | (1 << 9); // Open-drain
    GPIOB->OSPEEDR |= (0xF << 16);       // High speed
    GPIOB->AFR[1] |= (4 << 0) | (4 << 4); // AF4 cho PB8, PB9

    // Cấu hình GPIOA cho HCSR04 (TRIG và ECHO)
    GPIOA->MODER &= ~(3 << (TRIG_PIN * 2));  // Reset mode PA0
    GPIOA->MODER |= (1 << (TRIG_PIN * 2));   // PA0: Output mode
    GPIOA->OTYPER &= ~(1 << TRIG_PIN);       // Push-pull
    GPIOA->OSPEEDR |= (3 << (TRIG_PIN * 2)); // High speed
    GPIOA->PUPDR &= ~(3 << (TRIG_PIN * 2));  // No pull-up/pull-down

    GPIOA->MODER &= ~(3 << (ECHO_PIN * 2));  // PA1: Input mode
    GPIOA->PUPDR &= ~(3 << (ECHO_PIN * 2));  // No pull-up/pull-down

    // Cấu hình GPIOC cho PC2 (Output mode)
    RCC->AHB1ENR |= (1 << 2);             // Bật clock GPIOC
    GPIOC->MODER &= ~(3 << (2 * 2));      // Reset mode PC2
    GPIOC->MODER |= (1 << (2 * 2));       // PC2: Output mode
    GPIOC->OTYPER &= ~(1 << 2);           // Push-pull
    GPIOC->OSPEEDR |= (3 << (2 * 2));     // High speed
    GPIOC->PUPDR &= ~(3 << (2 * 2));      // No pull-up/pull-down

    // Cấu hình PC3
    GPIOC->MODER &= ~(3 << (3 * 2));  // Reset mode PC3
    GPIOC->MODER |= (1 << (3 * 2));   // PC3: Output mode

    // Cấu hình GPIOC cho PC11 và PC10 (Input mode)
    GPIOC->MODER &= ~(3 << (11 * 2));  // PC11: Input mode
    GPIOC->PUPDR &= ~(3 << (11 * 2));  // Xóa Pull-Up/Pull-Down
    GPIOC->PUPDR |= (1 << (11 * 2));   // Pull-Up

    GPIOC->MODER &= ~(3 << (10 * 2));  // PC10: Input mode
    GPIOC->PUPDR &= ~(3 << (10 * 2));  // Xóa Pull-Up/Pull-Down
    GPIOC->PUPDR |= (1 << (10 * 2));   // Pull-Up
}

void I2C1_Init(void) {
    RCC->APB1ENR |= (1 << 21); // Bật clock cho I2C1
    I2C1->CR1 &= ~(1 << 0);    // Tắt I2C1 trước khi cấu hình
    I2C1->CR2 = 16;            // Tần số clock = 16 MHz
    I2C1->CCR = 80;            // 100kHz (Standard mode)
    I2C1->TRISE = 17;          // Tối đa tăng TRISE
    I2C1->CR1 |= (1 << 0);     // Bật I2C1
}

void Timer2_Init(void) {
    RCC->APB1ENR |= (1 << 0);  // Bật clock cho TIM2
    TIM2->PSC = 16 - 1;        // Prescaler 16 (1 MHz)
    TIM2->ARR = 0xFFFF;        // Auto-reload tối đa
    TIM2->CCMR1 |= (1 << 0);   // CC1S = 01 (Input capture trên IC1)
    TIM2->CCER |= (1 << 0);    // Kích hoạt kênh 1 (Rising edge)
    TIM2->CR1 |= (1 << 0);     // Bật TIM2
}

void Timer3_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;  // Bật clock cho TIM3

    TIM3->PSC = 84000 - 1;   // Chia tần số xuống 1kHz (84 MHz / 16000)
    TIM3->ARR = 500 - 1;    // Chu kỳ 1 giây (1000 ms)

    TIM3->DIER |= TIM_DIER_UIE;  // Bật ngắt cập nhật
    TIM3->CR1 |= TIM_CR1_CEN;    // Bật Timer3

    NVIC_EnableIRQ(TIM3_IRQn);   // Bật ngắt trong NVIC
    NVIC_SetPriority(TIM3_IRQn, 1);  // Đặt ưu tiên
}

// Hàm ngắt Timer3
void TIM3_IRQHandler(void) {
    if (TIM3->SR & TIM_SR_UIF) {  // Kiểm tra cờ ngắt
        TIM3->SR &= ~TIM_SR_UIF;  // Xóa cờ ngắt
        GPIOC->ODR ^= (1 << 3);   // Đảo trạng thái đèn PC3
    }
}

void delay_us(uint16_t time) {
    TIM2->CNT = 0;
    while (TIM2->CNT < time);
}

uint8_t HCSR04_GetDis(void) {
    GPIOA->BSRR = (1 << TRIG_PIN);  // TRIG = HIGH
    delay_us(10);                   // 10 µs
    GPIOA->BSRR = (1 << (TRIG_PIN + 16));  // TRIG = LOW

    while (!(GPIOA->IDR & (1 << ECHO_PIN))); // Chờ ECHO = HIGH
    TIM2->CNT = 0;  // Reset counter
    while (GPIOA->IDR & (1 << ECHO_PIN));   // Chờ ECHO = LOW

    Difference = TIM2->CNT;
    Distance = Difference * 0.034 / 2; // Tính khoảng cách
    return Distance;
}

void LCD_Write(uint8_t data, uint8_t mode) {
    uint8_t highnib = data & 0xF0;   // Phần cao 4 bit
    uint8_t lownib = (data << 4) & 0xF0; // Phần thấp 4 bit

    // 1. Gửi lệnh Start condition
    I2C1->CR1 |= (1 << 8); // Start condition
    while (!(I2C1->SR1 & (1 << 0))); // Chờ SB bit

    // 2. Gửi địa chỉ LCD
    I2C1->DR = LCD_ADDR; // Địa chỉ LCD (I2C)
    while (!(I2C1->SR1 & (1 << 1))); // Chờ ADDR
    (void)I2C1->SR2; // Đọc SR2 để reset ADDR

    // 3. Gửi high nibble
    I2C1->DR = highnib | mode | LCD_BL | LCD_EN; // Gửi EN = 1
    while (!(I2C1->SR1 & (1 << 7))); // Chờ BTF
    delay_us(40); // Thêm trễ nhỏ (40µs là tối ưu)

    I2C1->DR = highnib | mode | LCD_BL; // Gửi EN = 0
    while (!(I2C1->SR1 & (1 << 7))); // Chờ BTF

    // 4. Gửi low nibble
    I2C1->DR = lownib | mode | LCD_BL | LCD_EN; // Gửi EN = 1
    while (!(I2C1->SR1 & (1 << 7))); // Chờ BTF
    delay_us(40); // Thêm trễ nhỏ

    I2C1->DR = lownib | mode | LCD_BL; // Gửi EN = 0
    while (!(I2C1->SR1 & (1 << 7))); // Chờ BTF

    // 5. Stop condition
    I2C1->CR1 |= (1 << 9); // Stop condition
    delay_ms(2); // Thêm trễ để LCD xử lý
}

void LCD_SendCommand(uint8_t cmd) {
    LCD_Write(cmd, LCD_COMMAND);
}

void LCD_SendData(uint8_t data) {
    LCD_Write(data, LCD_DATA);
}

void LCD_Init(void) {
    delay_ms(50); // Đợi LCD ổn định

    // Gửi lệnh khởi tạo 4-bit mode 3 lần
    LCD_SendCommand(0x30);
    delay_ms(5);
    LCD_SendCommand(0x30);
    delay_ms(1);
    LCD_SendCommand(0x30);

    // Chuyển sang chế độ 4-bit
    LCD_SendCommand(0x20);

    // Các lệnh cài đặt
    LCD_SendCommand(LCD_FUNCTION_SET); // 4-bit mode, 2 lines, 5x8 dots
    LCD_SendCommand(LCD_DISPLAY_ON);  // Bật màn hình, tắt con trỏ
    LCD_SendCommand(LCD_CLEAR);       // Xóa màn hình
    LCD_SendCommand(LCD_ENTRY_MODE);  // Cài đặt chế độ nhập dữ liệu (tăng địa chỉ)
}

void LCD_Print(const char* str) {
    while (*str) {
        LCD_SendData(*str++);
    }
}

void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 1000; i++) {
        __NOP();
    }
}
