#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <nvs.h>

#include "program_manager.h"

#include <set>

#define NVS_NAMESPACE "program_manager"
#define NVS_KEY "program_list"

static const char *TAG = "program_manager";

void program_manager_init(program_manager_t *manager)
{
    if (manager == NULL)
    {
        ESP_LOGE(TAG, "Manager pointer is NULL!");
        return;
    }

    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    memset(manager, 0, sizeof(program_manager_t));

    esp_err_t load_err = load_programs_from_nvs(manager);

    if (load_err == ESP_OK)
    {
        ESP_LOGI(TAG, "Loaded %d programs from NVS", manager->program_count);
    }
    else
    {
        ESP_LOGW(TAG, "No saved programs found in NVS (err: 0x%x)", load_err);
    }
}

program_t *find_program(program_manager_t *manager, uint8_t program_id)
{
    program_t *current = manager->program_list;

    while (current != NULL)
    {
        if (current->program_id == program_id)
        {
            return current;
        }

        current = current->next;
    }

    return NULL;
}

program_t *add_program(program_manager_t *manager, char *key, char *name)
{
    program_t *new_program = (program_t *)malloc(sizeof(program_t));

    if (!new_program)
    {
        return NULL;
    }

    memset(new_program, 0, sizeof(program_t));

    new_program->program_id = manager->program_count;
    new_program->key = key;
    new_program->name = name;
    new_program->next = manager->program_list;

    manager->program_list = new_program;
    manager->program_count++;

    return new_program;
}

esp_err_t load_programs_from_nvs(program_manager_t *manager)
{
    ESP_LOGI(TAG, "Loading programs from NVS...");

    if (!manager)
    {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK)
    {
        return err;
    }

    size_t required_size = 0;
    err = nvs_get_blob(nvs_handle, NVS_KEY, NULL, &required_size);
    if (err != ESP_OK || required_size == 0)
    {
        nvs_close(nvs_handle);
        return err;
    }

    ESP_LOGI(TAG,"Loading %d bytes for programs blob", required_size);

    uint8_t *buffer = (uint8_t *)malloc(required_size);
    if (!buffer)
    {
        nvs_close(nvs_handle);
        return ESP_ERR_NO_MEM;
    }

    err = nvs_get_blob(nvs_handle, NVS_KEY, buffer, &required_size);
    nvs_close(nvs_handle);

    if (err != ESP_OK)
    {
        free(buffer);
        return err;
    }

    uint8_t *ptr = buffer;
    uint8_t program_count = *((uint8_t *)ptr);
    ptr += sizeof(uint8_t);

    ESP_LOGI(TAG, "Program count is %u", program_count);

    for (uint8_t i = 0; i < program_count; i++)
    {
        program_t *program = (program_t *)calloc(1, sizeof(program_t));
        if (!program)
        {
            free(buffer);
            return ESP_ERR_NO_MEM;
        }

        program->program_id = *((uint8_t *)ptr);
        ptr += sizeof(uint8_t);

        program->key_length = *((uint8_t *)ptr);
        ptr += sizeof(uint8_t);

        ESP_LOGI(TAG, "Key length is %u", program->key_length);

        program->key = (char *)calloc(1, program->key_length + 1);

        memcpy(program->key, ptr, program->key_length);
        program->key[program->key_length] = 0x00;
        ptr += program->key_length;

        ESP_LOGI(TAG, "Key: %s", program->key);

        program->name_length = *((uint8_t *)ptr);
        ptr += sizeof(uint8_t);

        ESP_LOGI(TAG, "Name length is %u", program->name_length);

        program->name = (char *)calloc(1, program->name_length + 1);

        memcpy(program->name, ptr, program->name_length);
        program->name[program->name_length] = 0x00;
        ptr += program->name_length;

        ESP_LOGI(TAG, "Name: %s", program->name);

        program->next = manager->program_list;
        manager->program_list = program;
    }

    manager->program_count = program_count;
    free(buffer);

    return ESP_OK;
}

esp_err_t save_programs_to_nvs(program_manager_t *manager)
{
    if (!manager)
    {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
        return err;

    size_t required_size = sizeof(uint8_t); // program_count
    program_t *current = manager->program_list;

    while (current)
    {
        ESP_LOGI(TAG,"Saving %s:%u", current->key, strlen(current->key));
        required_size += sizeof(uint8_t); // program_id
        required_size += sizeof(uint8_t); // key length
        required_size += strlen(current->key);
        required_size += sizeof(uint8_t); // name length
        required_size += strlen(current->name);

        current = current->next;
    }

    ESP_LOGI(TAG,"Require %d", required_size);

    uint8_t *buffer = (uint8_t *)malloc(required_size);
    if (!buffer)
    {
        nvs_close(nvs_handle);
        return ESP_ERR_NO_MEM;
    }
    uint8_t *ptr = buffer;

    *((uint8_t *)ptr) = manager->program_count;
    ptr += sizeof(uint8_t);

    current = manager->program_list;
    while (current)
    {
        *((uint8_t *)ptr) = current->program_id;
        ptr += sizeof(uint8_t);

        *((uint8_t *)ptr) = strlen(current->key);
        ptr += sizeof(uint8_t);

        memcpy(ptr, current->key, strlen(current->key));
        ptr += strlen(current->key);

        *((uint8_t *)ptr) = strlen(current->name);
        ptr += sizeof(uint8_t);

        memcpy(ptr, current->name, strlen(current->name));
        ptr += strlen(current->name);

        current = current->next;
    }

    err = nvs_set_blob(nvs_handle, NVS_KEY, buffer, required_size);
    free(buffer);

    if (err == ESP_OK)
    {
        err = nvs_commit(nvs_handle);
    }

    nvs_close(nvs_handle);

    return err;
}