/*
 * flash_store_data.c
 *
 *  Created on: Jul 25, 2025
 *      Author: Ti Manh
 */

#include "flash_store_data.h"
#include "stdint.h"
#include "stm32g4xx_it.h"
#include "stm32g4xx_hal.h"
#include "constants.h"
#include "string.h"

extern CRC_HandleTypeDef hcrc;

typedef uint64_t flash_datatype;
#define DATA_SIZE sizeof(flash_datatype)

uint32_t calculate_crc32(const uint8_t *data, uint32_t length) {
    return HAL_CRC_Calculate(&hcrc, (uint32_t*)data, length / 4);
}

void store_flash_memory(uint32_t memory_address, uint8_t *data, uint16_t data_length)
{
    typedef union {
        uint8_t bytes[8];
        uint64_t dword;
    } aligned_data_t;

    FLASH_EraseInitTypeDef flash_erase_struct = {0};
    uint32_t error_status = 0;

    HAL_FLASH_Unlock();

    flash_erase_struct.TypeErase = FLASH_TYPEERASE_PAGES;
    flash_erase_struct.Page = (memory_address - FLASH_BASE) / FLASH_PAGE_SIZE;
    flash_erase_struct.NbPages = 1;  // Only erase 1 page
    flash_erase_struct.Banks = FLASH_BANK_1;

    if (HAL_FLASHEx_Erase(&flash_erase_struct, &error_status) != HAL_OK) {
        printf("Flash erase failed! Error: 0x%08lX\r\n", error_status);
        HAL_FLASH_Lock();
        return;
    }

    aligned_data_t word_data;
    uint32_t i = 0;

    while (i + 8 <= data_length) {
        memcpy(word_data.bytes, &data[i], 8);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                              memory_address + i, word_data.dword) != HAL_OK) {
            printf("Flash program failed at address 0x%08lX\r\n", memory_address + i);
        }
        i += 8;
    }

    if (i < data_length) {
        // Zero-pad the remaining bytes
        memset(word_data.bytes, 0xFF, 8);
        memcpy(word_data.bytes, &data[i], data_length - i);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                              memory_address + i, word_data.dword) != HAL_OK) {
            printf("Flash program (last) failed at address 0x%08lX\r\n", memory_address + i);
        }
    }

    HAL_FLASH_Lock();
}

void read_flash_memory(uint32_t memory_address, uint8_t *data, uint16_t data_length)
{
    for(int i = 0; i < data_length; i++)
    {
	*(data + i) = (*(uint8_t *)(memory_address + i));
    }
}

void save_config_to_flash() {
    DeviceConfig temp = current_config;
    temp.crc = 0; // clear before calculation
    temp.crc = calculate_crc32((uint8_t*)&temp, sizeof(DeviceConfig));

    store_flash_memory(FLASH_CONFIG_ADDRESS, (uint8_t*)&temp, sizeof(DeviceConfig));
}

void load_config_from_flash() {
    DeviceConfig temp;
    read_flash_memory(FLASH_CONFIG_ADDRESS, (uint8_t*)&temp, sizeof(DeviceConfig));

    uint32_t stored_crc = temp.crc;
    temp.crc = 0;
    uint32_t calc_crc = calculate_crc32((uint8_t*)&temp, sizeof(DeviceConfig));

    if (stored_crc == calc_crc && temp.version == default_config.version) {
        memcpy(&current_config, &temp, sizeof(DeviceConfig));
        printf("Loaded Config: version=%lu, can_id=%d\r\n", current_config.version, current_config.can_id);
    } else {
        // corrupted or incompatible version
        memcpy(&current_config, &default_config, sizeof(DeviceConfig));
        printf("Failed to load Config, using default");
        printf("Loaded Config: version=%lu, can_id=%d\r\n", current_config.version, current_config.can_id);
        save_config_to_flash(); // optional recovery
    }
}

