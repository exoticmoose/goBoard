#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

uint8_t readI2C(uint32_t ui32base, uint8_t slave_addr, uint8_t reg_addr, uint8_t *array, uint8_t cnt);

uint8_t writeI2C(uint32_t ui32base, uint8_t slave_addr, uint8_t *array, uint8_t cnt);

uint8_t initI2C(uint32_t ui32base);
