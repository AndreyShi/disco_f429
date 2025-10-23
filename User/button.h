#ifndef BUTTON_H
#define BUTTON_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief обработчик нажатия кнопки, через прерывание
 * 
 * @param GPIO_Pin номер пина подсоединенного к обработчику прерываний
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#ifdef __cplusplus
}
#endif
#endif // !1