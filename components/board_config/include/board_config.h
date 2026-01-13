#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <driver/gpio.h>

/* --- POWER & SYSTEM --- */
#define PIN_BATTERY_SENSE     GPIO_NUM_3   // Analog input for battery voltage divider

/* --- HIGH-SPEED PERIPHERALS (Direct to ESP32-S3) --- */
#define PIN_LCD_MOSI          GPIO_NUM_11
#define PIN_LCD_SCLK          GPIO_NUM_12
#define PIN_LCD_CS            GPIO_NUM_10
#define PIN_LCD_DC            GPIO_NUM_13
#define PIN_LCD_RST           GPIO_NUM_14
#define PIN_LCD_BL            GPIO_NUM_21  //Backlight PWM (Optional - Not currently connected)

#define PIN_E6B2_PHA          GPIO_NUM_1   // Z-Axis Industrial Encoder
#define PIN_E6B2_PHB          GPIO_NUM_2

// --- HIGH-SPEED PERIPHERALS (Direct to ESP32-S3) (SPI and Encoder defines) ...

#define PIN_JOYSTICK_X        GPIO_NUM_4
#define PIN_JOYSTICK_Y        GPIO_NUM_5
#define PIN_JOYSTICK_SW       GPIO_NUM_6   // Moved from MCP23017 to Direct GPIO

#define PIN_I2C_SDA           GPIO_NUM_8
#define PIN_I2C_SCL           GPIO_NUM_9
#define I2C_PORT              I2C_NUM_0

/* --- MCP23017 CONFIGURATION --- */
#define MCP23017_ADDR         0x20
#define MCP_IODIRA            0x00   // Direction Port A
#define MCP_IODIRB            0x01   // Direction Port B
#define MCP_GPPUA             0x0C   // Pull-up Port A
#define MCP_GPPUB             0x0D   // Pull-up Port B
#define MCP_GPIOA             0x12   // Data Port A
#define MCP_GPIOB             0x13   // Data Port B

/* --- I/O EXPANDER MAPPING --- */
// Port A (Bitwise 0-7)
#define MCP_PIN_EC11_PHA      0  
#define MCP_PIN_EC11_PHB      1  
#define MCP_PIN_EC11_SW       2  
#define MCP_PIN_PAIRING       3  
#define MCP_PIN_SHUTTER       4  
#define MCP_PIN_LOCK_PANEL_TOGGLE    5  
#define MCP_PIN_FINE_TOGGLE   6  
#define MCP_PIN_RESET_POS     7  

// Port B (Bitwise 0-7, though globally bit 8)
//#define MCP_PIN_SAVE_POS      0  // This is Pin 8 globally, but Bit 0 on Port B
#define MCP_PIN_TTP223_SAVE   0  // TTP223 Touch button for Saving Position
#define PIN_SLEEP_WAKE_BTN    GPIO_NUM_0   // Physical button for Deep Sleep toggle

/* --- ESP-NOW CONFIGURATION --- */
#define REMOTE_DEVICE_ROLE    "REMOTE_PANEL"
#define NVS_NAMESPACE         "storage"
#define NVS_KEY_PEER_MAC      "peer_mac"

#endif