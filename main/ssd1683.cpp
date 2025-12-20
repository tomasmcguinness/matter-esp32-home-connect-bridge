#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "ssd1683.h"

void lcd_read_busy()
{
    while (1)
    {
        int level = gpio_get_level((gpio_num_t)PIN_NUM_BUSY);

        if (level == 0)
        {
            break;
        }
    }
}

void lcd_write_cmd(spi_device_handle_t spi, uint8_t cmd)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));                             // Zero out the transaction
    t.length = 8;                                         // Command is 8 bits
    t.tx_buffer = &cmd;                                   // The data is the cmd itself
    t.user = (void *)0;                                   // D/C needs to be set to 0
    esp_err_t ret = spi_device_polling_transmit(spi, &t); // Transmit!
    assert(ret == ESP_OK);
}

void lcd_write_data(spi_device_handle_t spi, uint8_t cmd)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));                             // Zero out the transaction
    t.length = 8;                                         // Command is 8 bits
    t.tx_buffer = &cmd;                                   // The data is the cmd itself
    t.user = (void *)1;                                   // D/C needs to be set to 1
    esp_err_t ret = spi_device_polling_transmit(spi, &t); // Transmit!
    assert(ret == ESP_OK);
}

void lcd_reset()
{
    gpio_set_level((gpio_num_t)PIN_NUM_RST, true);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level((gpio_num_t)PIN_NUM_RST, false);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level((gpio_num_t)PIN_NUM_RST, true);
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void lcd_init(spi_device_handle_t spi, uint16_t width, uint16_t height)
{
    lcd_reset();
    lcd_read_busy();
    lcd_write_cmd(spi, 0x12);
    lcd_read_busy();

    lcd_write_cmd(spi, 0x21); //  Display update control
    lcd_write_data(spi, 0x40);
    lcd_write_data(spi, 0x00);

    lcd_write_cmd(spi, 0x3C); // BorderWavefrom
    lcd_write_data(spi, 0x05);

    lcd_write_cmd(spi, 0x11);   // data  entry  mode
    lcd_write_data(spi, 0x03); // X-mode

    lcd_write_cmd(spi, 0x44);
    lcd_write_data(spi, (0 >> 3) & 0xFF);
    lcd_write_data(spi, (width >> 3) & 0xFF);

    lcd_write_cmd(spi, 0x45);
    lcd_write_data(spi, 0 & 0xFF);
    lcd_write_data(spi, (0 >> 8) & 0xFF);
    lcd_write_data(spi, height & 0xFF);
    lcd_write_data(spi, (height >> 8) & 0xFF);

    // Cursor
    lcd_write_cmd(spi, 0x4E); // SET_RAM_X_ADDRESS_COUNTER
    lcd_write_data(spi, 0 & 0xFF);

    lcd_write_cmd(spi, 0x4F); // SET_RAM_Y_ADDRESS_COUNTER
    lcd_write_data(spi, 0 & 0xFF);
    lcd_write_data(spi, (0 >> 8) & 0xFF);

    lcd_read_busy();
}

void lcd_update(spi_device_handle_t spi)
{
    lcd_write_cmd(spi, 0x22);
    lcd_write_data(spi, 0xF7);
    lcd_write_cmd(spi, 0x20);
    lcd_read_busy();
}

void lcd_clear(spi_device_handle_t spi, uint16_t width, uint16_t height)
{
    uint16_t i, j, w, h;
    w = (width % 8 == 0) ? (width / 8) : (width / 8 + 1);
    h = height;

    lcd_write_cmd(spi, 0x24);

    for (j = 0; j < h; j++)
    {
        for (i = 0; i < w; i++)
        {
            lcd_write_data(spi, 0x00);
        }
    }

    lcd_write_cmd(spi, 0x26);
    for (j = 0; j < h; j++)
    {
        for (i = 0; i < w; i++)
        {
            lcd_write_data(spi, 0x00);
        }
    }

    lcd_update(spi);
}
