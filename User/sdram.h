#include "main.h"
#include <stdbool.h>
#include "user.h"

#define SDRAM_BASE_ADDR   0xD0000000
#define TEST_SIZE         0x10  // 4KB для теста

#ifdef __cplusplus
extern "C" {
#endif

bool test_sdram_basic(void);

#ifdef __cplusplus
}
#endif