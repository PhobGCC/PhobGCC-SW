#ifndef PERSISTENCE__FUNCTIONS_HPP
#define PERSISTENCE__FUNCTIONS_HPP

#include "pico/stdlib.h"
#include "hardware/flash.h"
#include <stdio.h>
#include "pico/sync.h"
#include <string.h>

namespace Persistence {

#define FLASH_OFFSET (1536 * 1024)

template<typename Page>
const Page* read() {
    static_assert(sizeof(Page)<=FLASH_SECTOR_SIZE);
    return (Page*)(XIP_BASE+FLASH_OFFSET+Page::index*FLASH_SECTOR_SIZE);
}

template<typename Page>
Page clone() {
    static_assert(sizeof(Page)<=FLASH_SECTOR_SIZE);
    Page copy = *(Page*)(XIP_BASE+FLASH_OFFSET+Page::index*FLASH_SECTOR_SIZE);
    return copy;
}

template<typename Page, typename FieldType>
void set(FieldType Page::* ptrToMember, FieldType newValue) {
    static_assert(sizeof(Page)<=FLASH_SECTOR_SIZE);
    uint32_t ints = save_and_disable_interrupts();
    uint32_t memoryAddress = FLASH_OFFSET + Page::index*FLASH_SECTOR_SIZE;
    uint8_t page[FLASH_SECTOR_SIZE];
    memcpy(page, (void*)(XIP_BASE+memoryAddress), FLASH_SECTOR_SIZE);
    (Page*)page->*ptrToMember = newValue;
    flash_range_erase(memoryAddress, FLASH_SECTOR_SIZE);
    flash_range_program(memoryAddress, page, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
}

template<typename Page>
void commitPtr(const Page* ptrToPage) {
    static_assert(sizeof(Page)<=FLASH_SECTOR_SIZE);
    uint32_t ints = save_and_disable_interrupts();
    uint8_t page[FLASH_SECTOR_SIZE];
    memcpy(page, (uint8_t*)ptrToPage, sizeof(Page));
    uint32_t memoryAddress = FLASH_OFFSET + Page::index*FLASH_SECTOR_SIZE;
    flash_range_erase(memoryAddress, FLASH_SECTOR_SIZE);
    flash_range_program(memoryAddress, page, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
}

template<typename Page>
void commit(const Page &ptrToPage) {
    commitPtr(&ptrToPage);
}

}

#endif
