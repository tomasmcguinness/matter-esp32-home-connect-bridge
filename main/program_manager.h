#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef struct program
{
    uint8_t program_id;
    uint8_t key_length;
    char *key;
    uint8_t name_length;
    char *name;

    struct program *next;
} program;

typedef program program_t;

typedef struct
{
    program_t *program_list;
    uint8_t program_count;     
} program_manager_t;

void program_manager_init(program_manager_t *manager);

program_t *find_program(program_manager_t *manager, uint8_t program_id);
program_t *add_program(program_manager_t *manager, char *key, char *name);

esp_err_t save_programs_to_nvs(program_manager_t *manager);
esp_err_t load_programs_from_nvs(program_manager_t *manager);