#ifndef _XATOM_CONFIG_H_
#define _XATOM_CONFIG_H_

#include <driver/gpio.h>

// X-Atom board pin map (derived from stock-binary analysis
// diagnostics; GPIO39's electrical role is not yet identified — stock
// startup drives it HIGH and this board reproduces that).

// Shared I2C bus
#define XATOM_I2C_PORT I2C_NUM_1
#define XATOM_I2C_SDA_PIN GPIO_NUM_3
#define XATOM_I2C_SCL_PIN GPIO_NUM_2

// ES8311 codec (I2C device address 0x18; codec API takes 0x30)
#define XATOM_ES8311_ADDR 0x30
#define AUDIO_INPUT_SAMPLE_RATE 24000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

// I2S0
#define AUDIO_I2S_GPIO_MCLK GPIO_NUM_6
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_7
#define AUDIO_I2S_GPIO_DIN GPIO_NUM_8
#define AUDIO_I2S_GPIO_WS GPIO_NUM_9
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_10
#define AUDIO_I2S_PA_GPIO GPIO_NUM_41  // active high, preloaded low

// Power management (AXP2101, chip ID 0x4A)
#define XATOM_PMIC_ADDR 0x35

// QMI8658 IMU
#define XATOM_IMU_ADDR 0x6B
#define XATOM_IMU_IRQ_PIN GPIO_NUM_42  // input only

// RKB1161 radar
#define XATOM_RADAR_UART UART_NUM_2
#define XATOM_RADAR_TX_PIN GPIO_NUM_17
#define XATOM_RADAR_RX_PIN GPIO_NUM_18
#define XATOM_RADAR_IO_PIN GPIO_NUM_16  // digital input only

// ML307 modem
#define ML307_TX_PIN GPIO_NUM_43
#define ML307_RX_PIN GPIO_NUM_44
#define ML307_DTR_PIN GPIO_NUM_21

// Startup control — stock drives HIGH at init; role unverified
#define XATOM_CONTROL39_PIN GPIO_NUM_39

// LED strip: 4 WS2812 GRB pixels
#define XATOM_LED_PIN GPIO_NUM_40
#define XATOM_LED_COUNT 4

// Button, active low
#define XATOM_BUTTON_PIN GPIO_NUM_5

#endif  // _XATOM_CONFIG_H_
