#include "include/stm32l051xx.h"
#include "util.h"
#include "dfplayer.h"

// USART2 init @ 9600 baud, TX = PA2, RX = PA3
void USART2_init(void) {
    // Enable GPIOA and USART2 clocks
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    RCC->CCIPR = (RCC->CCIPR & ~(RCC_CCIPR_USART2SEL)) | RCC_CCIPR_USART2SEL_0; // Set SYSCLK as USART2 clock source

    // Set PA2 and PA3 to AF mode
    TX_PORT->MODER = ((TX_PORT->MODER & ~(0x3 << TX_PIN * 2)) | (0x2 << TX_PIN * 2));
    RX_PORT->MODER = ((RX_PORT->MODER & ~(0x3 << RX_PIN * 2)) | (0x2 << RX_PIN * 2));
    TX_PORT->AFR[0] = ((TX_PORT->AFR[0] & ~(0xF << TX_PIN * 4)) | (0x4 << TX_PIN * 4)); // AF4 selected (0100)
    RX_PORT->AFR[0] = ((RX_PORT->AFR[0] & ~(0xF << RX_PIN * 4)) | (0x4 << RX_PIN * 4)); // AF4 selected (0100)

    USART2->CR1 &= ~(USART_CR1_UE); // Disable USART for configuration
    USART2->CR1 &= ~(USART_CR1_OVER8); // 8-bit data, no oversampling

    // Set USART2 baud rate, refer to p590 of datasheet
    // 9600 baud @ 32MHz: BRR = 32000000 / 9600 = 3333.33 → round to 3333 = 0x0D05
    USART2->BRR = 0xD05;

    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE; // Enable TX, RX
    USART2->CR1 |= USART_CR1_UE;                // Enable USART
}

// Simple delay (for startup delay or timing)
void delay(volatile uint32_t count) {
    while(count--) {
        __asm__("nop");
    }
}

// Send one byte over USART2
void USART2_send_byte(uint8_t byte) {
    while (!(USART2->ISR & USART_ISR_TXE)); // Wait until TX buffer empty
    USART2->TDR = byte;
    while (!(USART2->ISR & USART_ISR_TC));  // Wait for transmission complete
}

// Receive one byte over USART2
uint8_t USART2_receive_byte(void) {
    while (!(USART2->ISR & USART_ISR_RXNE)); // Wait for byte
    return USART2->RDR;
}

// DF Player Command Format: 
// Start Byte | Version | Len | Command | Feedback | Parameter1 | Parameter2 | Checksum | End Byte
// Start Byte: 0x7E
// Version Information: 0xFF
// Len: 0x06 (6 bytes of command data excluding start byte, checksum, and end byte)
// Command: 0x03
// Feedback: 0x00 (no feedback)
// Parameter1: Track number (1-255)
// Parameter2: 0x00 (not used for play command)
// Checksum: 16 bits - Calculated as 0xFFFF - (sum of bytes 1 to 6) + 1
// End Byte: 0xEF

void send_cmd(uint8_t cmd, uint8_t parameter1, uint8_t parameter2) 
{
    uint16_t checksum = 0 - (0xFF + 0x06 + cmd + 0x00 + parameter1 + parameter2);

    uint8_t command[10] = {
        0x7E,                       // Start byte
        0xFF,                       // Version
        0x06,                       // Length
        cmd,                        // Command
        0x00,                       // Feedback
        parameter1,                 // Parameter1 
        parameter2,                 // Parameter2 
        (checksum >> 8) & 0xFF,     // Checksum high byte
        checksum & 0xFF,            // Checksum low byte
        0xEF                        // End byte
    };

    for (int i = 0; i < 10; i++) {
            USART2_send_byte(command[i]);
    }
}

void DF_init(void) 
{
        // Initialize DFPlayer: CMD = 0x3F
        send_cmd(0x3F, 0x00, 0x02); // Initialize DFPlayer to play TF card
        delay(500); 
        // Set Volume: CMD = 0x06, Parameter 1/2: 0-30
        send_cmd(0x06, 0x00, 0x1E); // Set volume to 30 (max)
        delay(500);
}

// Folder 1-15 (XX), Track 1-255 (XXXX)
void DF_play(uint8_t folder, uint8_t track) 
{
        send_cmd(0x03, folder, track); // Play track 1
        delay(500);
}