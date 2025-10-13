#include "7seg_button.h"

// ===================================================================================
// 7seg
// ===================================================================================
void segregisterset(uint8_t num) {
  switch (num) {
    case SEG_0:
      GPIOA->BSHR = SEG_0_REGA; // set SEG_E, SEG_D
      GPIOB->BSHR = SEG_0_REGB; // reset SEG_OP, SEG_G. set SEG_C, SEG_B
      GPIOC->BSHR = SEG_0_REGC; // set SEG_A, SEG_D
      break;
    case SEG_1:
      GPIOA->BSHR = SEG_1_REGA;
      GPIOB->BSHR = SEG_1_REGB;
      GPIOC->BSHR = SEG_1_REGC;
      break;
    case SEG_2:
      GPIOA->BSHR = SEG_2_REGA;
      GPIOB->BSHR = SEG_2_REGB;
      GPIOC->BSHR = SEG_2_REGC;
      break;
    case SEG_3:
      GPIOA->BSHR = SEG_3_REGA;
      GPIOB->BSHR = SEG_3_REGB;
      GPIOC->BSHR = SEG_3_REGC;
      break;
    case SEG_4:
      GPIOA->BSHR = SEG_4_REGA;
      GPIOB->BSHR = SEG_4_REGB;
      GPIOC->BSHR = SEG_4_REGC;
      break;
    case SEG_5:
      GPIOA->BSHR = SEG_5_REGA;
      GPIOB->BSHR = SEG_5_REGB;
      GPIOC->BSHR = SEG_5_REGC;
      break;
    case SEG_6:
      GPIOA->BSHR = SEG_6_REGA;
      GPIOB->BSHR = SEG_6_REGB;
      GPIOC->BSHR = SEG_6_REGC;
      break;
    case SEG_7:
      GPIOA->BSHR = SEG_7_REGA;
      GPIOB->BSHR = SEG_7_REGB;
      GPIOC->BSHR = SEG_7_REGC;
      break;
    case SEG_8:
      GPIOA->BSHR = SEG_8_REGA;
      GPIOB->BSHR = SEG_8_REGB;
      GPIOC->BSHR = SEG_8_REGC;
      break;
    case SEG_9:
      GPIOA->BSHR = SEG_9_REGA;
      GPIOB->BSHR = SEG_9_REGB;
      GPIOC->BSHR = SEG_9_REGC;
      break;
    case SEG_NON: // no display
      GPIOA->BSHR = SEG_NON_REGA;
      GPIOB->BSHR = SEG_NON_REGB;
      GPIOC->BSHR = SEG_NON_REGC;
      break;
    case SEG_MINUS: // display "-"
      GPIOA->BSHR = SEG_MINUS_REGA;
      GPIOB->BSHR = SEG_MINUS_REGB;
      GPIOC->BSHR = SEG_MINUS_REGC;
      break;
    case SEG_F: // F
      GPIOA->BSHR = SEG_F_REGA;
      GPIOB->BSHR = SEG_F_REGB;
      GPIOC->BSHR = SEG_F_REGC;
      break;
    case SEG_I: // I
      GPIOA->BSHR = SEG_I_REGA;
      GPIOB->BSHR = SEG_I_REGB;
      GPIOC->BSHR = SEG_I_REGC;
      break;
    case SEG_X: // X
      GPIOA->BSHR = SEG_X_REGA;
      GPIOB->BSHR = SEG_X_REGB;
      GPIOC->BSHR = SEG_X_REGC;
      break;
    case SEG_P: // P
      GPIOA->BSHR = SEG_P_REGA;
      GPIOB->BSHR = SEG_P_REGB;
      GPIOC->BSHR = SEG_P_REGC;
      break;
    case SEG_C: // C
      GPIOA->BSHR = SEG_C_REGA;
      GPIOB->BSHR = SEG_C_REGB;
      GPIOC->BSHR = SEG_C_REGC;
      break;
    case SEG_A: // A
      GPIOA->BSHR = SEG_A_REGA;
      GPIOB->BSHR = SEG_A_REGB;
      GPIOC->BSHR = SEG_A_REGC;
      break;
    case SEG_L: // L
      GPIOA->BSHR = SEG_L_REGA;
      GPIOB->BSHR = SEG_L_REGB;
      GPIOC->BSHR = SEG_L_REGC;
      break;
    case SEG_E: // E
      GPIOA->BSHR = SEG_E_REGA;
      GPIOB->BSHR = SEG_E_REGB;
      GPIOC->BSHR = SEG_E_REGC;
      break;
    case SEG_T: // T
      GPIOA->BSHR = SEG_T_REGA;
      GPIOB->BSHR = SEG_T_REGB;
      GPIOC->BSHR = SEG_T_REGC;
      break;
    case SEG_R: // R
      GPIOA->BSHR = SEG_R_REGA;
      GPIOB->BSHR = SEG_R_REGB;
      GPIOC->BSHR = SEG_R_REGC;
      break;
    case SEG_G: // G
      GPIOA->BSHR = SEG_G_REGA;
      GPIOB->BSHR = SEG_G_REGB;
      GPIOC->BSHR = SEG_G_REGC;
      break;
    case SEG_D: // D
      GPIOA->BSHR = SEG_D_REGA;
      GPIOB->BSHR = SEG_D_REGB;
      GPIOC->BSHR = SEG_D_REGC;
      break;
    case SEG_V: // V
      GPIOA->BSHR = SEG_V_REGA;
      GPIOB->BSHR = SEG_V_REGB;
      GPIOC->BSHR = SEG_V_REGC;
      break;
    default:
      break;
  }
}

uint8_t seg_num[3]; // Left,Center,Right.
uint8_t seg_digit = 0; // 0:none,1:Left,2:Center,3:Right

// Set display content for each segment
void SEG_setEach(uint8_t seg_left, uint8_t seg_center, uint8_t seg_right, uint8_t _seg_digit) {
  seg_num[0] = seg_left;
  seg_num[1] = seg_center;
  seg_num[2] = seg_right;
  seg_digit = _seg_digit;
}

// set numbers(0~999000m) to display on 7seg
void SEG_number(uint32_t num, bool fix100m) {
  uint16_t temp = 0;
  if (num < 10000) {
    seg_digit = 1;
    temp = num / 10;
  } else if (num < 100000) {
    seg_digit = 2;
    temp = num / 100;
  } else {
    seg_digit = 0;
    temp = num / 1000;
  }
  if (!fix100m || num >= 10000) { // use all segs. Ex 10.2, 5.02
    seg_num[0] = temp / 100;
    seg_num[1] = (temp / 10) % 10;
    seg_num[2] = temp % 10;
  } else { // only use 2 segs. Ex 3.3, 9.9
    seg_digit = 2;
    seg_num[0] = 10; // empty
    seg_num[1] = temp/100;
    seg_num[2] = (temp / 10) % 10;
  }
}

uint8_t seg_driving = PIN_SEG_A1;
// 7seg driver
void SEG_driver() {
  switch (seg_driving) {
    case PIN_SEG_A3: // now Right,next is Left
      segregisterset(seg_num[0]);
      PIN_low(PIN_SEG_A1);
      if (seg_digit == 1) {
        PIN_high(PIN_SEG_OP);
      }
      seg_driving = PIN_SEG_A1;
      break;
    case PIN_SEG_A1: // now Left,next is Center
      segregisterset(seg_num[1]);
      PIN_low(PIN_SEG_A2);
      if (seg_digit == 2) {
        PIN_high(PIN_SEG_OP);
      }
      seg_driving = PIN_SEG_A2;
      break;
    case PIN_SEG_A2: // now Center,next is Right
      segregisterset(seg_num[2]);
      PIN_low(PIN_SEG_A3);
      if (seg_digit == 3) {
        PIN_high(PIN_SEG_OP);
      }
      seg_driving = PIN_SEG_A3;
      break;
    default:
      seg_driving = PIN_SEG_A1;
      break;
  }
}


// ===================================================================================
// button
// ===================================================================================
uint8_t button_pushed = 0; // last pushed button. 0 = none
uint8_t button_pushing = 0;
uint16_t button_count = 0; // how long button is pressed
// short push return Normal,long pushing return *10,release after long press retun *10+
uint8_t BUTTON_read() {
  // read button
  if (!PIN_read(PIN_BUTTON)) {
    switch (seg_driving) {
      case PIN_SEG_A1:
        button_pushing = BUTTON_OP_SHORT;
        break;
      case PIN_SEG_A2:
        button_pushing = BUTTON_CVCC_SHORT;
        break;
      case PIN_SEG_A3:
        button_pushing = BUTTON_UP_SHORT;
    }
    button_count++;
  } else if (PIN_read(PIN_BOOT_BUTTON)) {
    button_pushing = BUTTON_DOWN_SHORT;
    button_count++;
  } else if (button_pushing) {
    if (button_pushing == BUTTON_OP_SHORT && seg_driving == PIN_SEG_A1) {
      button_pushing = 0;
      button_pushed = BUTTON_OP_SHORT;
    } else if (button_pushing == BUTTON_CVCC_SHORT && seg_driving == PIN_SEG_A2) {
      button_pushing = 0;
      button_pushed = BUTTON_CVCC_SHORT;
    } else if (button_pushing == BUTTON_UP_SHORT && seg_driving == PIN_SEG_A3) {
      button_pushing = 0;
      button_pushed = BUTTON_UP_SHORT;
    } else if (button_pushing == BUTTON_DOWN_SHORT && !PIN_read(PIN_BOOT_BUTTON)) {
      button_pushing = 0;
      button_pushed = BUTTON_DOWN_SHORT;
    } else {
      button_count++;
    }
  }

  // reset and return
  if (button_pushing && button_count > BUTTON_LONGCOUNT) {
    return BUTTON_LONG_HOLD(button_pushing); // long pressing
  } else if (!button_pushing && button_pushed) { // release
    uint8_t temp;
    if (button_count > BUTTON_LONGCOUNT) {
      button_count = 0;
      temp = button_pushed;
      button_pushed = 0;
      return BUTTON_LONG_RELEASE(temp); // release after long press
    } else if (button_count > BUTTON_SHORTCOUNT) {
      button_count = 0;
      temp = button_pushed;
      button_pushed = 0;
      return temp; // short pressed
    } else { // chattering
    }
  } else if (button_pushing) {
    return BUTTON_ANY; // pushing some button
  }
  button_count = 0;
  button_pushed = 0;
  return 0; // not pushed
}