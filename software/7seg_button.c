#include "7seg_button.h"

// ===================================================================================
// 7seg
// ===================================================================================
static void SEG_setRegister(uint8_t num) {
  switch (num) {
    case SEG_0:
      GPIOA->BSHR = SEG_0_REGAH; // set SEG_D,SEG_E,SEG_A2,SEG_A3
      GPIOB->BSHR = SEG_0_REGBH; // reset SEG_G set SEG_A1
      GPIOC->BSHR = SEG_0_REGCH; // set SEG_A0,SEG_F
      GPIOC->BSXR = SEG_0_REGCX; // reset SEG_OP set SEG_B,SEG_C
      break;
    case SEG_1:
      GPIOA->BSHR = SEG_1_REGAH;
      GPIOB->BSHR = SEG_1_REGBH;
      GPIOC->BSHR = SEG_1_REGCH;
      GPIOC->BSXR = SEG_1_REGCX;
      break;
    case SEG_2:
      GPIOA->BSHR = SEG_2_REGAH;
      GPIOB->BSHR = SEG_2_REGBH;
      GPIOC->BSHR = SEG_2_REGCH;
      GPIOC->BSXR = SEG_2_REGCX;
      break;
    case SEG_3:
      GPIOA->BSHR = SEG_3_REGAH;
      GPIOB->BSHR = SEG_3_REGBH;
      GPIOC->BSHR = SEG_3_REGCH;
      GPIOC->BSXR = SEG_3_REGCX;
      break;
    case SEG_4:
      GPIOA->BSHR = SEG_4_REGAH;
      GPIOB->BSHR = SEG_4_REGBH;
      GPIOC->BSHR = SEG_4_REGCH;
      GPIOC->BSXR = SEG_4_REGCX;
      break;
    case SEG_5:
      GPIOA->BSHR = SEG_5_REGAH;
      GPIOB->BSHR = SEG_5_REGBH;
      GPIOC->BSHR = SEG_5_REGCH;
      GPIOC->BSXR = SEG_5_REGCX;
      break;
    case SEG_6:
      GPIOA->BSHR = SEG_6_REGAH;
      GPIOB->BSHR = SEG_6_REGBH;
      GPIOC->BSHR = SEG_6_REGCH;
      GPIOC->BSXR = SEG_6_REGCX;
      break;
    case SEG_7:
      GPIOA->BSHR = SEG_7_REGAH;
      GPIOB->BSHR = SEG_7_REGBH;
      GPIOC->BSHR = SEG_7_REGCH;
      GPIOC->BSXR = SEG_7_REGCX;
      break;
    case SEG_8:
      GPIOA->BSHR = SEG_8_REGAH;
      GPIOB->BSHR = SEG_8_REGBH;
      GPIOC->BSHR = SEG_8_REGCH;
      GPIOC->BSXR = SEG_8_REGCX;
      break;
    case SEG_9:
      GPIOA->BSHR = SEG_9_REGAH;
      GPIOB->BSHR = SEG_9_REGBH;
      GPIOC->BSHR = SEG_9_REGCH;
      GPIOC->BSXR = SEG_9_REGCX;
      break;
    case SEG_NON: // no display
      GPIOA->BSHR = SEG_NON_REGAH;
      GPIOB->BSHR = SEG_NON_REGBH;
      GPIOC->BSHR = SEG_NON_REGCH;
      GPIOC->BSXR = SEG_NON_REGCX;
      break;
    case SEG_MINUS: // display "-"
      GPIOA->BSHR = SEG_MINUS_REGAH;
      GPIOB->BSHR = SEG_MINUS_REGBH;
      GPIOC->BSHR = SEG_MINUS_REGCH;
      GPIOC->BSXR = SEG_MINUS_REGCX;
      break;
    case SEG_F: // F
      GPIOA->BSHR = SEG_F_REGAH;
      GPIOB->BSHR = SEG_F_REGBH;
      GPIOC->BSHR = SEG_F_REGCH;
      GPIOC->BSXR = SEG_F_REGCX;
      break;
    case SEG_I: // I
      GPIOA->BSHR = SEG_I_REGAH;
      GPIOB->BSHR = SEG_I_REGBH;
      GPIOC->BSHR = SEG_I_REGCH;
      GPIOC->BSXR = SEG_I_REGCX;
      break;
    case SEG_X: // X
      GPIOA->BSHR = SEG_X_REGAH;
      GPIOB->BSHR = SEG_X_REGBH;
      GPIOC->BSHR = SEG_X_REGCH;
      GPIOC->BSXR = SEG_X_REGCX;
      break;
    case SEG_P: // P
      GPIOA->BSHR = SEG_P_REGAH;
      GPIOB->BSHR = SEG_P_REGBH;
      GPIOC->BSHR = SEG_P_REGCH;
      GPIOC->BSXR = SEG_P_REGCX;
      break;
    case SEG_C: // C
      GPIOA->BSHR = SEG_C_REGAH;
      GPIOB->BSHR = SEG_C_REGBH;
      GPIOC->BSHR = SEG_C_REGCH;
      GPIOC->BSXR = SEG_C_REGCX;
      break;
    case SEG_A: // A
      GPIOA->BSHR = SEG_A_REGAH;
      GPIOB->BSHR = SEG_A_REGBH;
      GPIOC->BSHR = SEG_A_REGCH;
      GPIOC->BSXR = SEG_A_REGCX;
      break;
    case SEG_L: // L
      GPIOA->BSHR = SEG_L_REGAH;
      GPIOB->BSHR = SEG_L_REGBH;
      GPIOC->BSHR = SEG_L_REGCH;
      GPIOC->BSXR = SEG_L_REGCX;
      break;
    case SEG_E: // E
      GPIOA->BSHR = SEG_E_REGAH;
      GPIOB->BSHR = SEG_E_REGBH;
      GPIOC->BSHR = SEG_E_REGCH;
      GPIOC->BSXR = SEG_E_REGCX;
      break;
    case SEG_T: // T
      GPIOA->BSHR = SEG_T_REGAH;
      GPIOB->BSHR = SEG_T_REGBH;
      GPIOC->BSHR = SEG_T_REGCH;
      GPIOC->BSXR = SEG_T_REGCX;
      break;
    case SEG_R: // R
      GPIOA->BSHR = SEG_R_REGAH;
      GPIOB->BSHR = SEG_R_REGBH;
      GPIOC->BSHR = SEG_R_REGCH;
      GPIOC->BSXR = SEG_R_REGCX;
      break;
    case SEG_G: // G
      GPIOA->BSHR = SEG_G_REGAH;
      GPIOB->BSHR = SEG_G_REGBH;
      GPIOC->BSHR = SEG_G_REGCH;
      GPIOC->BSXR = SEG_G_REGCX;
      break;
    case SEG_D: // D
      GPIOA->BSHR = SEG_D_REGAH;
      GPIOB->BSHR = SEG_D_REGBH;
      GPIOC->BSHR = SEG_D_REGCH;
      GPIOC->BSXR = SEG_D_REGCX;
      break;
    case SEG_V: // V
      GPIOA->BSHR = SEG_V_REGAH;
      GPIOB->BSHR = SEG_V_REGBH;
      GPIOC->BSHR = SEG_V_REGCH;
      GPIOC->BSXR = SEG_V_REGCX;
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
bool button_sampled = false; // if any button is pushed ,be true
// short push return Normal,long pushing return *10,release after long press retun *10+
uint8_t BUTTON_read() {
  // read button
  if(PIN_read(PIN_BUTTON)){ // HIGH
    if(!button_sampled && SEG_driving == PIN_SEG_A2){ // when SEG_A1 was low and SEG_A2 is high
      BUTTON_pushing = BUTTON_OP_SHORT;
    }else if(SEG_driving == PIN_SEG_A3 && BUTTON_pushing == 0 && button_sampled){ // always high
      BUTTON_pushing = BUTTON_DOWN_SHORT;
    }
    button_sampled = true;
  }else if(button_sampled){
    switch(SEG_driving){
      case PIN_SEG_A1:
        // none
        break;
      case PIN_SEG_A2:
        BUTTON_pushing = BUTTON_CVCC_SHORT;
        break;
      case PIN_SEG_A3:
        if(BUTTON_pushing == 0){
          BUTTON_pushing = BUTTON_UP_SHORT;
        }else{ // double press?
          button_sampled = false;
          BUTTON_pushing = 0;
        }
        break;
      default:
        break;
    }
  }else{ // released
    button_sampled = false;
    BUTTON_pushing = 0;
  }

  // count and reset
  if(SEG_driving == PIN_SEG_A3){
    if(button_sampled){ // any button was pushed
      if(BUTTON_pushing == BUTTON_pushed){ // button is pushed
        BUTTON_count++;
        if(BUTTON_count > BUTTON_LONGCOUNT){
          BUTTON_pushing = 0;
          button_sampled = false;
          return BUTTON_LONG_HOLD(BUTTON_pushed); // long pressing
        }
      }else{ // pushed button is changed or pressed
        BUTTON_count = 1;
        BUTTON_pushed = BUTTON_pushing;
      }
    }else if(BUTTON_pushed > 0){ // button is relased
      if(BUTTON_count > BUTTON_LONGCOUNT){
        BUTTON_count = 0;
        BUTTON_pushing = BUTTON_pushed; // temp
        BUTTON_pushed = 0;
        button_sampled = false;
        return BUTTON_LONG_RELEASE(BUTTON_pushing); // relase after long press
      }else if(BUTTON_count > BUTTON_SHORTCOUNT){
        BUTTON_count = 0;
        BUTTON_pushing = BUTTON_pushed;
        BUTTON_pushed = 0;
        button_sampled = false;
        return BUTTON_pushing; // short pushed
      }else{ // chattering
        BUTTON_pushed = 0;
        BUTTON_count = 0;
      }
    }else{ // no button is pushed
      BUTTON_pushed = 0;
      BUTTON_count = 0;
      return 0;
    }
    // reset
    BUTTON_pushing = 0;
    button_sampled = false;
    return 0;
  }
  return 255;
}
