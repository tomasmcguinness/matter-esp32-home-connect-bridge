#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdint.h>
#include <vector>
#include "driver/spi_master.h"

#define PIN_NUM_CS 10
#define PIN_NUM_DC 46
#define PIN_NUM_RST 47
#define PIN_NUM_BUSY 48

#define IMAGE_W 24//400
#define IMAGE_H 24//300

void lcd_init(spi_device_handle_t spi);
void lcd_clear(spi_device_handle_t spi);
void lcd_draw(spi_device_handle_t spi, std::vector<uint8_t> buffer);