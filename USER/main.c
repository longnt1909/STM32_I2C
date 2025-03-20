#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include <stdio.h>

#define BH1750_ADDRESS 0x23  // Ð?a ch? I2C c?a BH1750

void GPIO_Config(void);
void USART1_Config(void);
void I2C1_Config(void);
void BH1750_Init(void);
uint16_t BH1750_ReadLight(void);
void I2C_Read(uint8_t address, uint8_t *data, uint8_t length);
void USART_SendString(char *str);
void Delay_ms(uint32_t ms);

int main(void) {
    SystemInit();
    GPIO_Config();
    USART1_Config();
    I2C1_Config();
    BH1750_Init();
    
    while (1) {
        uint16_t lux = BH1750_ReadLight();
        char buffer[50];
        sprintf(buffer, "Light: %d Lux\r\n", lux);
        USART_SendString(buffer);
        Delay_ms(1000);  // Ð?c d? li?u m?i 1 giây
    }
}

/* C?u hình GPIO cho UART */
void GPIO_Config(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
}

/* C?u hình UART1 (TX: PA9, RX: PA10) */
void USART1_Config(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    USART_InitStruct.USART_BaudRate = 9600;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStruct);
    USART_Cmd(USART1, ENABLE);
}

/* C?u hình I2C1 (SCL: PB6, SDA: PB7) */
void I2C1_Config(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    I2C_InitTypeDef I2C_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    I2C_InitStruct.I2C_ClockSpeed = 100000;  // 100kHz
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = 0;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitStruct);
    I2C_Cmd(I2C1, ENABLE);
}

void BH1750_Init(void) {
    uint8_t cmd = 0x10;  
    I2C_Read(BH1750_ADDRESS, &cmd, 1);
}

uint16_t BH1750_ReadLight(void) {
    uint8_t data[2] = {0};
    I2C_Read(BH1750_ADDRESS, data, 2);

    uint16_t lux = (data[0] << 8) | data[1];  // Ghép 2 byte thành 16-bit
    lux = lux / 1.2;  // Chuy?n d?i giá tr? raw thành Lux
    return lux;
}

void I2C_Read(uint8_t address, uint8_t *data, uint8_t length) {
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

    I2C_GenerateSTART(I2C1, ENABLE);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C1, address << 1, I2C_Direction_Receiver);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    for (uint8_t i = 0; i < length - 1; i++) {
        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
        data[i] = I2C_ReceiveData(I2C1);
    }

    I2C_AcknowledgeConfig(I2C1, DISABLE);
    I2C_GenerateSTOP(I2C1, ENABLE);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    data[length - 1] = I2C_ReceiveData(I2C1);
}

void USART_SendString(char *str) {
    while (*str) {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = *str++;
    }
}

void Delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 7200; i++);
}
