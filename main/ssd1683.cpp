#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdint.h>
#include <esp_log.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "ssd1683.h"

static const char *TAG = "ssd1683";

static std::vector<uint8_t> buffer = {};

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

void lcd_init(spi_device_handle_t spi)
{
    uint16_t w = IMAGE_W / 8;
    uint16_t h = IMAGE_H;

    for (int j = 0; j < h; j++)
    {
        for (int i = 0; i < w; i++)
        {
            buffer.push_back(0xFF); // White
        }
    }

    lcd_reset();
    lcd_read_busy();
    lcd_write_cmd(spi, 0x12);
    lcd_read_busy();

    lcd_write_cmd(spi, 0x21); //  Display update control
    lcd_write_data(spi, 0x40);
    lcd_write_data(spi, 0x00);

    lcd_write_cmd(spi, 0x3C); // BorderWavefrom
    lcd_write_data(spi, 0x05);

    // 1.5s refresh
    // lcd_write_cmd(spi, 0x1A); // Write to temperature register
    // lcd_write_data(spi, 0x6E);

    // 1s refresh
    lcd_write_cmd(spi, 0x1A); // Write to temperature register
    lcd_write_data(spi, 0x5A);

    lcd_write_cmd(spi, 0x22); // Load temperature value
    lcd_write_data(spi, 0x91);
    lcd_write_cmd(spi, 0x20);
    lcd_read_busy();

    lcd_write_cmd(spi, 0x11);  // data  entry  mode
    lcd_write_data(spi, 0x03); // X-mode

    lcd_write_cmd(spi, 0x44);
    lcd_write_data(spi, (0 >> 3) & 0xFF);
    lcd_write_data(spi, ((IMAGE_W - 1) >> 3) & 0xFF);

    lcd_write_cmd(spi, 0x45);
    lcd_write_data(spi, 0 & 0xFF);
    lcd_write_data(spi, (0 >> 8) & 0xFF);
    lcd_write_data(spi, (IMAGE_H - 1) & 0xFF);
    lcd_write_data(spi, ((IMAGE_H - 1) >> 8) & 0xFF);

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
    // lcd_write_data(spi, 0xF7); // SLOW
    lcd_write_data(spi, 0xC7); // FAST
    lcd_write_cmd(spi, 0x20);
    lcd_read_busy();
}

void lcd_clear(spi_device_handle_t spi)
{
    uint16_t i, j, w, h;
    w = IMAGE_W / 8;
    h = IMAGE_H;

    // Write black (0xFF)
    //
    lcd_write_cmd(spi, 0x24);

    for (j = 0; j < h; j++)
    {
        for (i = 0; i < w; i++)
        {
            lcd_write_data(spi, 0xFF);
        }
    }

    lcd_update(spi);
}

void lcd_draw(spi_device_handle_t spi)
{
    ESP_LOGI(TAG, "lcd_draw");

    uint16_t i;

    lcd_write_cmd(spi, 0x24);

    for (i = 0; i < buffer.size(); i++)
    {
        lcd_write_data(spi, buffer[i]);
    }

    lcd_update(spi);
}

void lcd_set_pixel(uint16_t x, uint16_t y, bool black)
{
    uint16_t addr = x / 8 + y * (IMAGE_W / 8);
    uint8_t current = buffer[addr];

    if (black) // Set the bit to 1
    {
        buffer[addr] = current & ~(0x80 >> (x % 8));
    }
    else
    {
        buffer[addr] = current | (0x80 >> (x % 8));
    }
}

void lcd_show_char(spi_device_handle_t spi, uint16_t x, uint16_t y, uint16_t chr, uint16_t size)
{
    uint16_t i, m, temp, size2, chr1;
    uint16_t x0, y0;
    x += 1, y += 1, x0 = x, y0 = y;
    if (size == 8)
    {
        size2 = 6;
    }
    else
    {
        size2 = (size / 8 + ((size % 8) ? 1 : 0)) * (size / 2);
    }
    chr1 = chr - ' ';
    for (i = 0; i < size2; i++)
    {
        if (size == 8)
        {
            temp = ascii_0806[chr1][i];
        }
        // else if (size1 == 12)
        // {
        //   temp = ascii_1206[chr1][i]; //调用1206字体
        // }
        else if (size == 16)
        {
            temp = ascii_1608[chr1][i];
        }
        else if (size == 24)
        {
            temp = ascii_2412[chr1][i];
        }
        else if (size == 48)
        {
            temp = ascii_4824[chr1][i];
        }
        else
        {
            return;
        }

        for (m = 0; m < 8; m++)
        {
            if (temp & 0x01)
            {
                lcd_set_pixel(x, y, true);
            }
            else
            {
                lcd_set_pixel(x, y, false);
            }
            temp >>= 1;
            y++;
        }

        x++;
        if ((size != 8) && ((x - x0) == size / 2))
        {
            x = x0;
            y0 = y0 + 8;
        }
        y = y0;
    }
}

void lcd_show_string(spi_device_handle_t spi, uint16_t x, uint16_t y, uint16_t size, char *text)
{
    while (*text != '\0')
    {
        lcd_show_char(spi, x, y, *text, size);
        text++;
        x += size / 2;
    }
}

void lcd_draw_string(spi_device_handle_t spi, uint16_t x, uint16_t y, uint16_t size, char *text)
{
    ESP_LOGI(TAG, "Drawing \"%s\"", text);

    int max_characters = 50;

    int length = strlen(text);
    int i = 0;
    char line[max_characters + 1]; //+1 is to place the string terminator '\0'

    while (i < length)
    {
        int lineLength = 0;
        memset(line, 0, sizeof(line)); // Clear the line buffer

        // Fill the line until it reaches the screen width or the end of the string
        while (lineLength < max_characters && i < length)
        {
            line[lineLength++] = text[i++];
        }

        lcd_show_string(spi, x, y, size, line); // Display this line
        y += size;                              // Update the y-coordinate for displaying the next line

        // If there are still remaining strings and the next line exceeds the screen height, stop displaying
        // You need to decide when to stop displaying based on your screen height here
        // For example, if your screen height is 300 pixels and the height of each character is 24 pixels, then you can display 12 lines
        if (y >= 300)
        {
            break;
        }
    }
}