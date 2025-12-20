#include "driver/spi_master.h"

#define PIN_NUM_CS 10
#define PIN_NUM_DC 46
#define PIN_NUM_RST 47
#define PIN_NUM_BUSY 48

#define IMAGE_W 400
#define IMAGE_H 300

void lcd_init(spi_device_handle_t spi, uint16_t width, uint16_t height);
void lcd_clear(spi_device_handle_t spi, uint16_t width, uint16_t height);