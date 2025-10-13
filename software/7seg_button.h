#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <system.h>
#include <gpio.h>
#include "pin_define.h"

// button
#define BUTTON_DOWN_SHORT 1
#define BUTTON_UP_SHORT 2
#define BUTTON_CVCC_SHORT 3
#define BUTTON_OP_SHORT 4
#define BUTTON_DOWN_LONG_HOLD (BUTTON_DOWN_SHORT * 10)
#define BUTTON_UP_LONG_HOLD (BUTTON_UP_SHORT * 10)
#define BUTTON_CVCC_LONG_HOLD (BUTTON_CVCC_SHORT * 10)
#define BUTTON_OP_LONG_HOLD (BUTTON_OP_SHORT * 10)
#define BUTTON_DOWN_LONG_RELEASE (BUTTON_DOWN_SHORT * 10 + BUTTON_DOWN_SHORT)
#define BUTTON_UP_LONG_RELEASE (BUTTON_UP_SHORT * 10 + BUTTON_UP_SHORT)
#define BUTTON_CVCC_LONG_RELEASE (BUTTON_CVCC_SHORT * 10 + BUTTON_CVCC_SHORT)
#define BUTTON_OP_LONG_RELEASE (BUTTON_OP_SHORT * 10 + BUTTON_OP_SHORT)
#define BUTTON_ANY 255 // while count < BUTTON_*_SHORT
#define BUTTON_NON 0 // any button isn't pushed
#define BUTTON_LONG_HOLD(button_short_num) (button_short_num * 10)
#define BUTTON_LONG_RELEASE(button_short_num) (button_short_num * 10 + button_short_num)
#define BUTTON_IS_LONG_RELEASE(button_num) (button_num == BUTTON_DOWN_LONG_RELEASE || button_num == BUTTON_UP_LONG_RELEASE || button_num == BUTTON_CVCC_LONG_RELEASE || button_num == BUTTON_OP_LONG_RELEASE)
#define BUTTON_IS_SHORT(button_num) (BUTTON_DOWN_SHORT <= button_num && button_num <= BUTTON_OP_SHORT)

#define BUTTON_LONGCOUNT 250 // time of recognized as a long press
#define BUTTON_SHORTCOUNT 2 // time of recognized as a short press

// 7seg pattern define
#define SEG_0 0
#define SEG_1 1
#define SEG_2 2
#define SEG_3 3
#define SEG_4 4
#define SEG_5 5
#define SEG_S SEG_5
#define SEG_6 6
#define SEG_7 7
#define SEG_8 8
#define SEG_9 9
#define SEG_NON 10
#define SEG_MINUS 11
#define SEG_F 12
#define SEG_I 13
#define SEG_X 14
#define SEG_P 15
#define SEG_C 16
#define SEG_A 17
#define SEG_L 18
#define SEG_E 19
#define SEG_T 20
#define SEG_R 21
#define SEG_G 22
#define SEG_D 23
#define SEG_V 24

void SEG_setEach(uint8_t seg_left, uint8_t seg_center, uint8_t seg_right, uint8_t _seg_digit); // Set display content for each segment
void SEG_number(uint32_t num, bool fix100m);
void SEG_driver();

uint8_t BUTTON_read();

#ifdef __cplusplus
}
#endif
