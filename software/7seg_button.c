#include "7seg_button.h"

// ===================================================================================
// 7seg
// ===================================================================================
static void SEG_setRegister(uint8_t num) {
  switch (num) {
    case SEG_0:
      GPIOA->BSHR = SEG_0_REGAH; // set SEG_E, SEG_D
      GPIOB->BSHR = SEG_0_REGBH; // reset SEG_OP, SEG_G. set SEG_C, SEG_B
      GPIOC->BSHR = SEG_0_REGCH; // set SEG_A, SEG_D
      break;
    case SEG_1:
      GPIOA->BSHR = SEG_1_REGAH;
      GPIOB->BSHR = SEG_1_REGBH;
      GPIOC->BSHR = SEG_1_REGCH;
      break;
    case SEG_2:
      GPIOA->BSHR = SEG_2_REGAH;
      GPIOB->BSHR = SEG_2_REGBH;
      GPIOC->BSHR = SEG_2_REGCH;
      break;
    case SEG_3:
      GPIOA->BSHR = SEG_3_REGAH;
      GPIOB->BSHR = SEG_3_REGBH;
      GPIOC->BSHR = SEG_3_REGCH;
      break;
    case SEG_4:
      GPIOA->BSHR = SEG_4_REGAH;
      GPIOB->BSHR = SEG_4_REGBH;
      GPIOC->BSHR = SEG_4_REGCH;
      break;
    case SEG_5:
      GPIOA->BSHR = SEG_5_REGAH;
      GPIOB->BSHR = SEG_5_REGBH;
      GPIOC->BSHR = SEG_5_REGCH;
      break;
    case SEG_6:
      GPIOA->BSHR = SEG_6_REGAH;
      GPIOB->BSHR = SEG_6_REGBH;
      GPIOC->BSHR = SEG_6_REGCH;
      break;
    case SEG_7:
      GPIOA->BSHR = SEG_7_REGAH;
      GPIOB->BSHR = SEG_7_REGBH;
      GPIOC->BSHR = SEG_7_REGCH;
      break;
    case SEG_8:
      GPIOA->BSHR = SEG_8_REGAH;
      GPIOB->BSHR = SEG_8_REGBH;
      GPIOC->BSHR = SEG_8_REGCH;
      break;
    case SEG_9:
      GPIOA->BSHR = SEG_9_REGAH;
      GPIOB->BSHR = SEG_9_REGBH;
      GPIOC->BSHR = SEG_9_REGCH;
      break;
    case SEG_NON: // no display
      GPIOA->BSHR = SEG_NON_REGAH;
      GPIOB->BSHR = SEG_NON_REGBH;
      GPIOC->BSHR = SEG_NON_REGCH;
      break;
    case SEG_MINUS: // display "-"
      GPIOA->BSHR = SEG_MINUS_REGAH;
      GPIOB->BSHR = SEG_MINUS_REGBH;
      GPIOC->BSHR = SEG_MINUS_REGCH;
      break;
    case SEG_F: // F
      GPIOA->BSHR = SEG_F_REGAH;
      GPIOB->BSHR = SEG_F_REGBH;
      GPIOC->BSHR = SEG_F_REGCH;
      break;
    case SEG_I: // I
      GPIOA->BSHR = SEG_I_REGAH;
      GPIOB->BSHR = SEG_I_REGBH;
      GPIOC->BSHR = SEG_I_REGCH;
      break;
    case SEG_X: // X
      GPIOA->BSHR = SEG_X_REGAH;
      GPIOB->BSHR = SEG_X_REGBH;
      GPIOC->BSHR = SEG_X_REGCH;
      break;
    case SEG_P: // P
      GPIOA->BSHR = SEG_P_REGAH;
      GPIOB->BSHR = SEG_P_REGBH;
      GPIOC->BSHR = SEG_P_REGCH;
      break;
    case SEG_C: // C
      GPIOA->BSHR = SEG_C_REGAH;
      GPIOB->BSHR = SEG_C_REGBH;
      GPIOC->BSHR = SEG_C_REGCH;
      break;
    case SEG_A: // A
      GPIOA->BSHR = SEG_A_REGAH;
      GPIOB->BSHR = SEG_A_REGBH;
      GPIOC->BSHR = SEG_A_REGCH;
      break;
    case SEG_L: // L
      GPIOA->BSHR = SEG_L_REGAH;
      GPIOB->BSHR = SEG_L_REGBH;
      GPIOC->BSHR = SEG_L_REGCH;
      break;
    case SEG_E: // E
      GPIOA->BSHR = SEG_E_REGAH;
      GPIOB->BSHR = SEG_E_REGBH;
      GPIOC->BSHR = SEG_E_REGCH;
      break;
    case SEG_T: // T
      GPIOA->BSHR = SEG_T_REGAH;
      GPIOB->BSHR = SEG_T_REGBH;
      GPIOC->BSHR = SEG_T_REGCH;
      break;
    case SEG_R: // R
      GPIOA->BSHR = SEG_R_REGAH;
      GPIOB->BSHR = SEG_R_REGBH;
      GPIOC->BSHR = SEG_R_REGCH;
      break;
    case SEG_G: // G
      GPIOA->BSHR = SEG_G_REGAH;
      GPIOB->BSHR = SEG_G_REGBH;
      GPIOC->BSHR = SEG_G_REGCH;
      break;
    case SEG_D: // D
      GPIOA->BSHR = SEG_D_REGAH;
      GPIOB->BSHR = SEG_D_REGBH;
      GPIOC->BSHR = SEG_D_REGCH;
      break;
    case SEG_V: // V
      GPIOA->BSHR = SEG_V_REGAH;
      GPIOB->BSHR = SEG_V_REGBH;
      GPIOC->BSHR = SEG_V_REGCH;
      break;
    default:
      break;
  }
}

static uint8_t SEG_num[3]; // Left,Center,Right.
static uint8_t SEG_digit = 0; // 0:none,1:Left,2:Center,3:Right

// Set display content for each segment
void SEG_setEach(uint8_t seg_left, uint8_t seg_center, uint8_t seg_right, uint8_t _SEG_digit) {
  SEG_num[0] = seg_left;
  SEG_num[1] = seg_center;
  SEG_num[2] = seg_right;
  SEG_digit = _SEG_digit;
}

// set numbers(0~999000m) to display on 7seg
void SEG_setNumber(uint32_t num, bool fix100m) {
  uint16_t temp = 0;
  if (num < 10000) {
    SEG_digit = 1;
    temp = num / 10;
  } else if (num < 100000) {
    SEG_digit = 2;
    temp = num / 100;
  } else {
    SEG_digit = 0;
    temp = num / 1000;
  }
  if (!fix100m || num >= 10000) { // use all segs. Ex 10.2, 5.02
    SEG_num[0] = temp / 100;
    SEG_num[1] = (temp / 10) % 10;
    SEG_num[2] = temp % 10;
  } else { // only use 2 segs. Ex 3.3, 9.9
    SEG_digit = 2;
    SEG_num[0] = 10; // empty
    SEG_num[1] = temp/100;
    SEG_num[2] = (temp / 10) % 10;
  }
}

static uint8_t SEG_driving = PIN_SEG_A1;
// 7seg driver
void SEG_driver() {
  switch (SEG_driving) {
    case PIN_SEG_A3: // now Right,next is Left
      SEG_setRegister(SEG_num[0]);
      PIN_low(PIN_SEG_A1);
      if (SEG_digit == 1) {
        PIN_high(PIN_SEG_OP);
      }
      SEG_driving = PIN_SEG_A1;
      break;
    case PIN_SEG_A1: // now Left,next is Center
      SEG_setRegister(SEG_num[1]);
      PIN_low(PIN_SEG_A2);
      if (SEG_digit == 2) {
        PIN_high(PIN_SEG_OP);
      }
      SEG_driving = PIN_SEG_A2;
      break;
    case PIN_SEG_A2: // now Center,next is Right
      SEG_setRegister(SEG_num[2]);
      PIN_low(PIN_SEG_A3);
      if (SEG_digit == 3) {
        PIN_high(PIN_SEG_OP);
      }
      SEG_driving = PIN_SEG_A3;
      break;
    default:
      SEG_driving = PIN_SEG_A1;
      break;
  }
}


// ===================================================================================
// button
// ===================================================================================
static uint8_t BUTTON_pushed = 0; // last pushed button. 0 = none
static uint8_t BUTTON_pushing = 0;
static uint16_t BUTTON_count = 0; // how long button is pressed
// short push return Normal,long pushing return *10,release after long press retun *10+
uint8_t BUTTON_read() {
  // read button
  if (!PIN_read(PIN_BUTTON)) {
    switch (SEG_driving) {
      case PIN_SEG_A1:
        BUTTON_pushing = BUTTON_OP_SHORT;
        break;
      case PIN_SEG_A2:
        BUTTON_pushing = BUTTON_CVCC_SHORT;
        break;
      case PIN_SEG_A3:
        BUTTON_pushing = BUTTON_UP_SHORT;
    }
    BUTTON_count++;
  } else if (PIN_read(PIN_BOOT_BUTTON)) {
    BUTTON_pushing = BUTTON_DOWN_SHORT;
    BUTTON_count++;
  } else if (BUTTON_pushing) {
    if (BUTTON_pushing == BUTTON_OP_SHORT && SEG_driving == PIN_SEG_A1) {
      BUTTON_pushing = 0;
      BUTTON_pushed = BUTTON_OP_SHORT;
    } else if (BUTTON_pushing == BUTTON_CVCC_SHORT && SEG_driving == PIN_SEG_A2) {
      BUTTON_pushing = 0;
      BUTTON_pushed = BUTTON_CVCC_SHORT;
    } else if (BUTTON_pushing == BUTTON_UP_SHORT && SEG_driving == PIN_SEG_A3) {
      BUTTON_pushing = 0;
      BUTTON_pushed = BUTTON_UP_SHORT;
    } else if (BUTTON_pushing == BUTTON_DOWN_SHORT && !PIN_read(PIN_BOOT_BUTTON)) {
      BUTTON_pushing = 0;
      BUTTON_pushed = BUTTON_DOWN_SHORT;
    } else {
      BUTTON_count++;
    }
  }

  // reset and return
  if (BUTTON_pushing && BUTTON_count > BUTTON_LONGCOUNT) {
    return BUTTON_LONG_HOLD(BUTTON_pushing); // long pressing
  } else if (!BUTTON_pushing && BUTTON_pushed) { // release
    uint8_t temp;
    if (BUTTON_count > BUTTON_LONGCOUNT) {
      BUTTON_count = 0;
      temp = BUTTON_pushed;
      BUTTON_pushed = 0;
      return BUTTON_LONG_RELEASE(temp); // release after long press
    } else if (BUTTON_count > BUTTON_SHORTCOUNT) {
      BUTTON_count = 0;
      temp = BUTTON_pushed;
      BUTTON_pushed = 0;
      return temp; // short pressed
    } else { // chattering
    }
  } else if (BUTTON_pushing) {
    return BUTTON_ANY; // pushing some button
  }
  BUTTON_count = 0;
  BUTTON_pushed = 0;
  return 0; // not pushed
}
