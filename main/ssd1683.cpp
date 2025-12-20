#include "driver/gpio.h"

void lcd_reset()
{
    gpio_set_level((gpio_num_t)PIN_NUM_RST, true);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level((gpio_num_t)PIN_NUM_RST, false);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level((gpio_num_t)PIN_NUM_RST, true);
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void lcd_init(&spi) 
{
    lcd_reset();
    lcd_read_busy();
}

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