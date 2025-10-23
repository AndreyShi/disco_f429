#include "sdram.h"


/**
 * @brief чтение запись bank1 sdram адрес 0xD0000000
 * 
 * @return true - чтение запись прошла успешно
 * @return false - чтение запись прошла с ошибками
 */
bool test_sdram_basic(void) {
    volatile uint32_t* sdram = (uint32_t*)SDRAM_BASE_ADDR;
    
    printf("Testing SDRAM at 0x%08lX...\n", (uint32_t)sdram);
    
    // Тест записи
    printf("Writing test pattern...\n");
    for(uint32_t i = 0; i < TEST_SIZE / 4; i++) {
        sdram[i] = 0x12345678 + i;  // Простой паттерн
    }
    
    // Тест чтения
    printf("Verifying data...\n");
    for(uint32_t i = 0; i < TEST_SIZE / 4; i++) {
        if(sdram[i] != (0x12345678 + i)) {
            printf("SDRAM ERROR at 0x%08lX: wrote 0x%08lX, read 0x%08lX\n",
                   (uint32_t)&sdram[i], (0x12345678 + i), sdram[i]);
            return false;
        }
    }
    
    printf("SDRAM basic test PASSED!\n");
    return true;
}