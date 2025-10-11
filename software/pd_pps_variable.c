// ===================================================================================
// mini PD-PPS VariablePowerSupply firmware
#define VERSION 910 // 0.91
// Author: Unagi Dojyou
// based on https://github.com/wagiminator
// License: http://creativecommons.org/licenses/by-sa/3.0/
// ===================================================================================
#include <config.h>             // user configurations
#include <system.h>             // system functions
#include <debug_serial.h>       // serial debug functions
#include <gpio.h>               // GPIO functions
#include <usbpd_sink.h>         // USB PD sink functions

// ===================================================================================
// configuration
// ===================================================================================

// if DEBUG is defined, calibration is disaled.
// #define DEBUG

// PIN assign
#define V_ADC PB0
#define I_ADC PA4
#define I_IN PA7
#define I_OPAN PB1
#define ONOFF PA5
#define CV PA2 // pull Down
#define CC PA3 // pull Down
#define B_BUTTON PC17
#define BUTTON PB11

#define SEG_A1 PB4
#define SEG_A2 PB6
#define SEG_A3 PB3

#define SEG_A0 PC3 // SEG_A
#define SEG_B PB7
#define SEG_C PB9
#define SEG_D PC0
#define SEG_E PA1
#define SEG_F PA0
#define SEG_G PB8
#define SEG_OP PB10

// button count, setting
#define LONGCOUNT 250 // time of recognized as a long press
#define SHORTCOUNT 2 // time of recognized as a short press
#define MAXCOUNT 200 // Proportional to the interval of voltage,ampar,watt update time
#define LONGPUSH_SPEED 100 // supeed of long push down/up

// PD setting
#define KEEPTIME 40 // Prevents the reset after about 10 seconds

// flash addr
#define USE_FLASH_ADD 0x800BF00 // page192
#define V_A_ADD USE_FLASH_ADD
#define V_B_ADD (V_A_ADD + 0x4)
#define I_A_ADD (V_B_ADD + 0x4)
#define I_B_ADD (I_A_ADD + 0x4)
#define TRIGGER_V_ADD (I_B_ADD + 0x4)
#define TRIGGER_A_ADD (TRIGGER_V_ADD + 0x4)
#define USE_FLASH_END TRIGGER_A_ADD
#define FLASH_END 0x0800F7FF // end of main flash memory

// calibration setting
#define CALV1 5000 // calibration at 5V
#define CALV2 18000 // calibration at 18V
#define CALA1 0 // calibration at 0A
#define CALA2 3000 // calibration at 3A

// define PPS
#define DEFAULT_PPS_VOLTAGE 5000 // 5V
#define DEFAULT_FIX_VOLTAGE 5000 // 5V
#define MAX_TRIGGER_V 21000 // 21V
#define MIN_TRIGGER_V 3300 // 3.3V
#define MAX_TRIGGER_A 3000 // 3A
#define MIN_TRIGGER_A MIN_PPS_CURRENT
#define MIN_PPS_CURRENT 500 // 500mA

// over current protection
#define LIMIT_CURRENT 500 // set_current + 500mA
#define LIMIT_FIVE_CURRENT 2400 // USB BC1.2 max Current is 1500mA

// over voltage protection
#define LIMIT_VOLTAGE 1000 // set_voltage + 1V

// Amount of change at one time
#define STEP_VOLTAGE_SHORT 100 // 100mV/(one button push)
#define STEP_VOLTAGE_LONG 300
#define STEP_CURRENT_SHORT 100 // 100mA/(one button push)
#define STEP_CURRENT_LONG 100
#define STEP_VOLTAGE_LONG_NEG 1000 // Send pps when this value changes while holding down the button
#define STEP_CURRENT_LONG_NEG 500 // Send pps when this value changes while holding down the button

// time out of triger menu
#define TRIGGER_TIMEOUT 2000 // 2second

// ===================================================================================
// define
// ===================================================================================

#define bool _Bool
#define false ((bool)+0)
#define true ((bool)+1)

// mode
#define MODE_5V 0
#define MODE_FIX 1
#define MODE_PPS 2
#define MODE_CAL 3
#define MODE_TRG 4
#define MODE_SETTRG 5
#define MODE_DELTRG 6
#define MODE_VER 7

// display mode
#define DISPVOLTAGE 0
#define DISPCURRENT 1
#define DISPWATT 2

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

// OPA addr
#define OPA_CTLR1_PSEL2_MASK 0x00180000

// ===================================================================================
// functions
// ===================================================================================

void segregisterset(uint8_t num) {
  switch (num) {
    case 0:
      GPIOA->BSHR = 0b00000000000000000000000000000011; // set SEG_E, SEG_D
      GPIOB->BSHR = 0b00000101000000000000001011011000; // reset SEG_OP, SEG_G. set SEG_C, SEG_B
      GPIOC->BSHR = 0b00000000000000000000000000001001; // set SEG_A, SEG_D
      break;
    case 1:
      GPIOA->BSHR = 0b00000000000000110000000000000000;
      GPIOB->BSHR = 0b00000101000000000000001011011000;
      GPIOC->BSHR = 0b00000000000010010000000000000000;
      break;
    case 2:
      GPIOA->BSHR = 0b00000000000000010000000000000010;
      GPIOB->BSHR = 0b00000110000000000000000111011000;
      GPIOC->BSHR = 0b00000000000000000000000000001001;
      break;
    case 3:
      GPIOA->BSHR = 0b00000000000000110000000000000000;
      GPIOB->BSHR = 0b00000100000000000000001111011000;
      GPIOC->BSHR = 0b00000000000000000000000000001001;
      break;
    case 4:
      GPIOA->BSHR = 0b00000000000000100000000000000001;
      GPIOB->BSHR = 0b00000100000000000000001111011000;
      GPIOC->BSHR = 0b00000000000010010000000000000000;
      break;
    case 5:
      GPIOA->BSHR = 0b00000000000000100000000000000001;
      GPIOB->BSHR = 0b00000100100000000000001101011000;
      GPIOC->BSHR = 0b00000000000000000000000000001001;
      break;
    case 6:
      GPIOA->BSHR = 0b00000000000000000000000000000011;
      GPIOB->BSHR = 0b00000100100000000000001101011000;
      GPIOC->BSHR = 0b00000000000000000000000000001001;
      break;
    case 7:
      GPIOA->BSHR = 0b00000000000000100000000000000001;
      GPIOB->BSHR = 0b00000101000000000000001011011000;
      GPIOC->BSHR = 0b00000000000000010000000000001000;
      break;
    case 8:
      GPIOA->BSHR = 0b00000000000000000000000000000011;
      GPIOB->BSHR = 0b00000100000000000000001111011000;
      GPIOC->BSHR = 0b00000000000000000000000000001001;
      break;
    case 9:
      GPIOA->BSHR = 0b00000000000000100000000000000001;
      GPIOB->BSHR = 0b00000100000000000000001111011000;
      GPIOC->BSHR = 0b00000000000000000000000000001001;
      break;
    case 10: // no display
      GPIOA->BSHR = 0b00000000000000110000000000000000;
      GPIOB->BSHR = 0b00000111100000000000000001011000;
      GPIOC->BSHR = 0b00000000000010010000000000000000;
      break;
    case 11: // display "-"
      GPIOA->BSHR = 0b00000000000000110000000000000000;
      GPIOB->BSHR = 0b00000110100000000000000101011000;
      GPIOC->BSHR = 0b00000000000010010000000000000000;
      break;
    case 12: // F
      GPIOA->BSHR = 0b00000000000000000000000000000011;
      GPIOB->BSHR = 0b00000110100000000000000101011000;
      GPIOC->BSHR = 0b00000000000000010000000000001000;
      break;
    case 13: // I
      GPIOA->BSHR = 0b00000000000000000000000000000011;
      GPIOB->BSHR = 0b00000111100000000000000001011000;
      GPIOC->BSHR = 0b00000000000010010000000000000000;
      break;
    case 14: // X
      GPIOA->BSHR = 0b00000000000000000000000000000011;
      GPIOB->BSHR = 0b00000100000000000000001111011000;
      GPIOC->BSHR = 0b00000000000010010000000000000000;
      break;
    case 15: // P
      GPIOA->BSHR = 0b00000000000000000000000000000011;
      GPIOB->BSHR = 0b00000110000000000000000111011000;
      GPIOC->BSHR = 0b00000000000000010000000000001000;
      break;
    case 16: // C
      GPIOA->BSHR = 0b00000000000000000000000000000011;
      GPIOB->BSHR = 0b00000111100000000000000001011000;
      GPIOC->BSHR = 0b00000000000000000000000000001001;
      break;
    case 17: // A
      GPIOA->BSHR = 0b00000000000000000000000000000011;
      GPIOB->BSHR = 0b00000100000000000000001111011000;
      GPIOC->BSHR = 0b00000000000000010000000000001000;
      break;
    case 18: // L
      GPIOA->BSHR = 0b00000000000000000000000000000011;
      GPIOB->BSHR = 0b00000111100000000000000001011000;
      GPIOC->BSHR = 0b00000000000010000000000000000001;
      break;
    case 19: // E
      GPIOA->BSHR = 0b00000000000000000000000000000011;
      GPIOB->BSHR = 0b00000110100000000000000101011000;
      GPIOC->BSHR = 0b00000000000000000000000000001001;
      break;
    case 20: // T
      GPIOA->BSHR = 0b00000000000000000000000000000011;
      GPIOB->BSHR = 0b00000110100000000000000101011000;
      GPIOC->BSHR = 0b00000000000010000000000000000001;
      break;
    case 22: // R
      GPIOA->BSHR = 0b00000000000000010000000000000010;
      GPIOB->BSHR = 0b00000110100000000000000101011000;
      GPIOC->BSHR = 0b00000000000010010000000000000000;
      break;
    case 23: // G
      GPIOA->BSHR = 0b00000000000000000000000000000011;
      GPIOB->BSHR = 0b00000101100000000000001001011000;
      GPIOC->BSHR = 0b00000000000000000000000000001001;
      break;
    case 24: // D
      GPIOA->BSHR = 0b00000000000000010000000000000010;
      GPIOB->BSHR = 0b00000100000000000000001111011000;
      GPIOC->BSHR = 0b00000000000010000000000000000001;
      break;
    case 25: // V
      GPIOA->BSHR = 0b00000000000000000000000000000011;
      GPIOB->BSHR = 0b00000110000000000000000111011000;
      GPIOC->BSHR = 0b00000000000010010000000000000000;
      break;
    default:
      break;
  }
}

uint8_t seg_num[3]; // Left,Center,Right.
uint8_t seg_digit = 0; // 0:none,1:Left,2:Center,3:Right
// set numbers(0~999000m) to display on 7seg
void setseg(uint32_t num,bool fix100m) {
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

uint8_t seg_driving = SEG_A1;
// 7seg driver
void dispseg() {
  switch (seg_driving) {
    case SEG_A3: // now Right,next is Left
      segregisterset(seg_num[0]);
      PIN_low(SEG_A1);
      if (seg_digit == 1) {
        PIN_high(SEG_OP);
      }
      seg_driving = SEG_A1;
      break;
    case SEG_A1: // now Left,next is Center
      segregisterset(seg_num[1]);
      PIN_low(SEG_A2);
      if (seg_digit == 2) {
        PIN_high(SEG_OP);
      }
      seg_driving = SEG_A2;
      break;
    case SEG_A2: // now Center,next is Right
      segregisterset(seg_num[2]);
      PIN_low(SEG_A3);
      if (seg_digit == 3) {
        PIN_high(SEG_OP);
      }
      seg_driving = SEG_A3;
      break;
    default:
      seg_driving = SEG_A1;
      break;
  }
}

uint8_t button_pushed = 0; // last pushed button. 0 = none
uint8_t button_pushing = 0;
uint16_t button_count = 0; // how long button is pressed
// short push return Normal,long pushing return *10,release after long press retun *10+
uint8_t readbutton() {
  // read button
  if (!PIN_read(BUTTON)) {
    switch (seg_driving) {
      case SEG_A1:
        button_pushing = BUTTON_OP_SHORT;
        break;
      case SEG_A2:
        button_pushing = BUTTON_CVCC_SHORT;
        break;
      case SEG_A3:
        button_pushing = BUTTON_UP_SHORT;
    }
    button_count++;
  } else if (PIN_read(B_BUTTON)) {
    button_pushing = BUTTON_DOWN_SHORT;
    button_count++;
  } else if (button_pushing) {
    if (button_pushing == BUTTON_OP_SHORT && seg_driving == SEG_A1) {
      button_pushing = 0;
      button_pushed = BUTTON_OP_SHORT;
    } else if (button_pushing == BUTTON_CVCC_SHORT && seg_driving == SEG_A2) {
      button_pushing = 0;
      button_pushed = BUTTON_CVCC_SHORT;
    } else if (button_pushing == BUTTON_UP_SHORT && seg_driving == SEG_A3) {
      button_pushing = 0;
      button_pushed = BUTTON_UP_SHORT;
    } else if (button_pushing == BUTTON_DOWN_SHORT && !PIN_read(B_BUTTON)) {
      button_pushing = 0;
      button_pushed = BUTTON_DOWN_SHORT;
    } else {
      button_count++;
    }
  }

  // reset and return
  if (button_pushing && button_count > LONGCOUNT) {
    return BUTTON_LONG_HOLD(button_pushing); // long pressing
  } else if (!button_pushing && button_pushed) { // release
    uint8_t temp;
    if (button_count > LONGCOUNT) {
      button_count = 0;
      temp = button_pushed;
      button_pushed = 0;
      return BUTTON_LONG_RELEASE(temp); // release after long press
    } else if (button_count > SHORTCOUNT) {
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

uint8_t mode = 0; // 0:5V, 1:Fix, 2:PPS, 3:Cal, 4:trg 5:settrg
bool ppsable = false;
bool fixable = false;

void setdefaultmode() {
  if (ppsable && fixable) {
    mode = MODE_PPS;
  } else if (fixable) {
    mode = MODE_FIX;
  } else {
    mode = MODE_5V;
  }
}

void setmode() {
  PIN_input(SEG_A1);
  PIN_input(SEG_A2);
  PIN_input(SEG_A3);
  if (PIN_read(B_BUTTON)) {
    // Down button
    // it must be download mode
    setdefaultmode();
    return;
  } else {
    PIN_output(SEG_A3);
    PIN_low(SEG_A3);
    DLY_ms(1);
    if (!PIN_read(BUTTON)) {
      // up
      setdefaultmode();
      return;
    }
    PIN_input(SEG_A3);
    PIN_output(SEG_A2);
    PIN_low(SEG_A2);
    DLY_ms(1);
    if (!PIN_read(BUTTON)) {
      PIN_input(SEG_A2);
      PIN_output(SEG_A1);
      PIN_low(SEG_A1);
      DLY_ms(1);
      if (!PIN_read(BUTTON)) {
        // CVCC and OP
        mode = MODE_CAL;
        return;
      } else {
        // cvcc
        PIN_low(SEG_A1);
        mode = MODE_SETTRG;
        return;
      }
    }
    PIN_input(SEG_A2);
    PIN_output(SEG_A1);
    PIN_low(SEG_A1);
    DLY_ms(1);
    if (!PIN_read(BUTTON) && fixable) { // fix mode
      // op
      mode = MODE_FIX;
      return;
    }
  }
  setdefaultmode();
}

bool UnlockFlash() {
  FLASH->KEYR = FLASH_KEY1;
  FLASH->KEYR = FLASH_KEY2;
  uint32_t temp = FLASH->CTLR; // read FLASH_CTLR
  return !(temp & FLASH_CTLR_LOCK);
}

bool FastUnlockFlash() {
  FLASH->MODEKEYR = FLASH_KEY1;
  FLASH->MODEKEYR = FLASH_KEY2;
  uint32_t temp = FLASH->CTLR; // read FLASH_CTLR
  return !(temp & FLASH_CTLR_FLOCK);
}

// when EOP is 1 return true,else return false
bool WaitFlashBusy() {
  uint32_t temp;
  do {
    temp = FLASH->STATR; // read FLASH_STATR
  } while (temp & FLASH_STATR_BSY); // while(BSY==1)
  if (!(temp & FLASH_STATR_EOP)) { // check EOP
    FLASH->CTLR = 0; // reset CTLR
    return false;
  }
  FLASH->STATR = temp; // reset EOP
  return true;
}

// Erase Flash Page(256Byte). "address" is Page start address.
bool FastEraseFlash(uint32_t address) {
  // 1) check locks
  uint32_t temp = FLASH->CTLR; // read FLASH_CTLR
  if (temp & FLASH_CTLR_LOCK) { // check lock
    if (!UnlockFlash()) {
      return false;
    }
  }
  // 2)
  temp = FLASH->CTLR;
  if (temp & FLASH_CTLR_FLOCK) { // check flock
    if (!FastUnlockFlash()) {
      return false;
    }
  }
  // 3) check other operation
  WaitFlashBusy();
  // 4) set erase mode
  FLASH->CTLR = FLASH_CTLR_FTER; // set FTER bit
  // 5) set erase address
  FLASH->ADDR = address;
  // 6) start erasec
  temp = FLASH->CTLR;
  temp |= FLASH_CTLR_STRT; // set FTER STRT
  FLASH->CTLR = temp; // write reg
  // 7,8) wait erase
  if (!WaitFlashBusy()) return false;
  // 9) reset CTRL
  FLASH->CTLR = 0;
  return true;
}

// Write Flash only 32Byte. "message" must be uint32_t[8]. "address" is Page start address.
bool FastWriteFlash32(uint32_t address,uint32_t *message) {
  // check locks
  uint32_t temp = FLASH->CTLR; // read FLASH_CTLR
  if (temp & FLASH_CTLR_LOCK) {  // check lock
    if (!UnlockFlash()) {
      return false;
    }
  }
  temp = FLASH->CTLR;
  if (temp & FLASH_CTLR_FLOCK) { // check flock
    if (!FastUnlockFlash()) {
      return false;
    }
  }
  // 3) check other operation
  WaitFlashBusy();
  // 4) set programming mode
  FLASH->CTLR = FLASH_CTLR_FTPG; // set FTPG
  // 15)
  for (int i = 0;i < 16;i++) {
    //5)
    temp = FLASH->CTLR;
    temp |= FLASH_CTLR_BUFRST; // set BUFRST
    FLASH->CTLR = temp; // write reg
    // 6) wait buffa reset
    if (!WaitFlashBusy()) return false;
    // 10)
    for (int j = 0;j < 4;j++) {
      // 7) writedata
      if (i == 0) {
        *(volatile uint32_t*)(address+(i * 0x10)+(j * 4)) = message[j];
      } else if (i == 1) {
        *(volatile uint32_t*)(address+(i * 0x10)+(j * 4)) = message[j + 4];
      } else {
        *(volatile uint32_t*)(address+(i * 0x10)+(j * 4)) = 0xFFFFFFFF;
      }
      // 8) BUFLOAD
      temp = FLASH->CTLR;
      temp |= FLASH_CTLR_BUFLOAD; // set BUFLOAD
      FLASH->CTLR = temp; // write reg
      // 9) wait BUFFLOAD
      WaitFlashBusy();
    }
    // 11)
    FLASH->ADDR = address;
    // 12)
    temp = FLASH->CTLR;
    temp |= FLASH_CTLR_STRT;
    FLASH->CTLR = temp;
    // 13)
    if (!WaitFlashBusy()) return false;
  }
  FLASH->CTLR = 0; // reset CTLR
  return true;
}


// lock flash write
void relockFlash() {
  WaitFlashBusy();
  uint32_t temp = 0x00008080;
  FLASH->CTLR = temp;
}

// read flash
uint32_t ReadFlash32(uint32_t address) {
  if (FLASH_BASE <= address && address < FLASH_END) {
    WaitFlashBusy();
    return (*(volatile const uint32_t*)(address));
  }
  return 0;
}

uint32_t V_a = 0;
int32_t V_b = 0;
uint32_t I_a = 0;
int32_t I_b = 0;
bool readcoeff() {
  V_a = ReadFlash32(V_A_ADD);
  V_b = (int32_t)ReadFlash32(V_B_ADD);
  I_a = ReadFlash32(I_A_ADD);
  I_b = (int32_t)ReadFlash32(I_B_ADD);
  return !(V_a == 0xFFFFFFFF || V_b == 0xFFFFFFFF || I_a == 0xFFFFFFFF || I_b == 0xFFFFFFFF || V_a == 0 || I_a == 0);
}

uint32_t triggervoltage = 0;
uint32_t triggercurrent = 0;
bool readtrigger() {
  triggervoltage = ReadFlash32(TRIGGER_V_ADD);
  triggercurrent = ReadFlash32(TRIGGER_A_ADD);
  return (MIN_TRIGGER_V <= triggervoltage && triggervoltage <= MAX_TRIGGER_V && 
  ((MIN_TRIGGER_A <= triggercurrent && triggercurrent <= MAX_TRIGGER_A) || triggercurrent == UINT16_MAX));
}

bool writeceff() {
  if (V_a == 0 && V_b == 0 && I_a == 0) { // can't read flash
    return false;
  }
  if (!(ReadFlash32(USE_FLASH_END + 0x4) == 0xFFFFFFFF)) { // check using
    return false;
  }
  if (!(ReadFlash32(USE_FLASH_ADD - 0x4) == 0xFFFFFFFF)) { // check using
    return false;
  }
  if (!FastEraseFlash(USE_FLASH_ADD)) {
    return false;
  }
  uint32_t message[8] = {V_a, V_b, I_a, I_b, triggervoltage, triggercurrent, UINT32_MAX, UINT32_MAX};
  if (!FastWriteFlash32(USE_FLASH_ADD, message)) {
    return false;
  }
  relockFlash();
  return true;
}

void enable_OPA() {
  OPA->OPAKEY = OPA_KEY1; // OPA Unlock
  OPA->OPAKEY = OPA_KEY2; // OPA Unlock
  OPA->CTLR1 &= ~OPA_CTLR1_NSEL2_OFF; // unmask CTLR1_NSEL2
  // OPA->CTLR1 |= OPA_CTLR1_NSEL2_PB1; // configure PB1 as opa-
  OPA->CTLR1 |= OPA_CTLR1_NSEL2_PGA_32X; // coufigure opa- 32x
  OPA->CTLR1 |= OPA_CTLR1_FB_EN2; // enable FB
  OPA->CTLR1 &= ~OPA_CTLR1_PSEL2_MASK; // unmask CTLR1_NSEL2
  OPA->CTLR1 |= OPA_CTLR1_PSEL2_PA7; // configure PA7 as opa+
  OPA->CTLR1 |= OPA_CTLR1_MODE2_PA4; // configure PA4 as opaOUT
  OPA->CTLR1 |= OPA_CTLR1_EN2; // enable OPA2
  OPA->CTLR1 |= OPA_CTLR1_OPA_LOCK; // OPA Lock
}

int32_t mes_Voltage = 0;
int32_t mes_Current = 0;
uint32_t sum_Voltage = 0;
uint32_t sum_Current = 0;
uint16_t set_Voltage = DEFAULT_PPS_VOLTAGE; // ~= PD_getVoltage
uint16_t set_Current = 0; // ~= PD_getCurrent
uint16_t min_Voltage = 0;
uint16_t max_Voltage = 0;
uint16_t min_Current = 0;
uint16_t max_Current = 0;
uint32_t Voltage = 0;
uint32_t Current = 0;
uint8_t pdonum = 0;
uint8_t dispmode = DISPVOLTAGE;
bool invalid_voltage = true;
bool output = false; // OFF
uint16_t count = 0; // while counter

// read ADC and check
void mesureVA() {
  // measure Voltage and Current
  ADC_input(V_ADC);
  mes_Voltage = (ADC_read() * V_a) / 1000 + V_b / 1000;
  #ifndef DEBUG
  if (mes_Voltage >= set_Voltage + LIMIT_VOLTAGE) { // stop over voltage
    if (output) {
      output = false;
      PIN_low(ONOFF);
      invalid_voltage = true;
    }
  }
  #endif
  if (mes_Voltage < 0) mes_Voltage = 0;
  sum_Voltage += mes_Voltage;
  DLY_ms(1);
  ADC_input(I_ADC);
  mes_Current = (ADC_read() * I_a) / 1000 + I_b / 1000;
  #ifndef DEBUG
  if (mes_Current > set_Current + LIMIT_CURRENT) { // stop over current
    if (output) {
      output = false;
      PIN_low(ONOFF);
      invalid_voltage = true;
    }
  }
  #endif
  if (mes_Current < 0) mes_Current = 0;
  sum_Current += mes_Current;
}

void manage_onoff() {
  if (mode == MODE_PPS) {
    if (!PIN_read(ONOFF) && output && !invalid_voltage) { // OFF -> ON
      PIN_high(ONOFF);
      invalid_voltage = !PD_setPPS(set_Voltage, set_Current);
    } else if (PIN_read(ONOFF) && !output) { // ON -> OFF
      PIN_low(ONOFF);
      invalid_voltage = !PD_setPPS(min_Voltage, set_Current);
    }
    if ((set_Voltage != PD_getVoltage() || set_Current != PD_getCurrent()) && output && !invalid_voltage) {
      invalid_voltage = !PD_setPPS(set_Voltage, set_Current);
    }
  } else if (output && !invalid_voltage) {
    PIN_high(ONOFF);
  } else {
    PIN_low(ONOFF);
  }
}

void manage_disp(uint16_t disp_set_Voltage, uint16_t disp_set_Current) {
  if (output) { // ON
    switch (dispmode) {
      case DISPVOLTAGE:
        setseg(Voltage, false);
        PIN_high(CV);
        PIN_low(CC);
        break;
      case DISPCURRENT:
        setseg(Current, false);
        PIN_low(CV);
        PIN_high(CC);
        break;
      case DISPWATT:
        setseg(Voltage * Current / 1000, false);
        PIN_high(CV);
        PIN_high(CC);
        break;
    }
  } else { // OFF
    switch (dispmode) {
      case DISPCURRENT:
        PIN_low(CV);
        PIN_high(CC);
        if (disp_set_Current == UINT16_MAX) {
          seg_num[0] = 10; // " "
          seg_num[1] = 11; // -
          seg_num[2] = 10; // " "
          seg_digit = 0;
        } else {
          setseg(disp_set_Current, true);
        }
        break;
      case DISPWATT:
        dispmode = DISPVOLTAGE;
        // not need break
      case DISPVOLTAGE:
        setseg(disp_set_Voltage, true);
        PIN_high(CV);
        PIN_low(CC);
        break;
    }
  }
  dispseg();
}

void ppsmode_setup();
void ppsmode_loop();
void fiveVmode();
void fixmode_setup();
void fixmode_loop();
void calmode();
void triggermode_setup();
void triggermode_loop();
void triggersetmode_setup();
void triggersetmode_loop();
void triggerdelmode();
void vermode();
void mode_menu();

// ===================================================================================
// Main Function
// ===================================================================================
int main(void) {
  // Setup
  PIN_output(ONOFF);
  PIN_output(CV);
  PIN_output(CC);
  PIN_input_PD(B_BUTTON); // pull down
  PIN_input_PU(BUTTON); // pull up
 
  PIN_output(SEG_A0);
  PIN_output(SEG_B);
  PIN_output(SEG_C);
  PIN_output(SEG_D);
  PIN_output(SEG_E);
  PIN_output(SEG_F);
  PIN_output(SEG_G);
  PIN_output(SEG_OP);
  
  PIN_low(ONOFF);

  // Setup
  enable_OPA();
  ADC_init(); // init ADC
  ADC_slow();

  if (PD_connect()) { // init USB PD
    if (PD_getPPSNum() > 0) {
      ppsable = true;
    }
    if (PD_getFixedNum() > 0) {
      fixable = true;
    }
  }

  setmode();

  if (readtrigger() && mode != MODE_SETTRG && mode != MODE_CAL) {
    mode = MODE_TRG;
  }

  if (!readcoeff()) {
    #ifndef DEBUG
    mode = MODE_CAL;
    #endif
    #ifdef DEBUG
    V_a = (VDD * (HIGH_R + LOW_R) / LOW_R) * 1000 / 4095;
    V_b = 0;
    I_a = (1000 * VDD) / ((SHUNT_R * 4095) / 1000);
    I_b = -AMPOFFSET * 1000;
    #endif
  }
  
  PIN_output(SEG_A1);
  PIN_output(SEG_A2);
  PIN_output(SEG_A3);

  mode_menu();
}

// ===================================================================================
// mode select
// ===================================================================================
#define MODE_LIST_MAX 2

uint8_t mode_list[6][MODE_LIST_MAX + 1] = {{MODE_5V, MODE_SETTRG, UINT8_MAX}, // fiveVmode
                                      {MODE_FIX, MODE_SETTRG, UINT8_MAX}, // fixmode
                                      {MODE_PPS, MODE_FIX, MODE_SETTRG}, // ppsmode
                                      {MODE_VER, MODE_CAL, UINT8_MAX}, // calmode
                                      {MODE_TRG, MODE_DELTRG, UINT8_MAX}, // triggermode
                                      {MODE_SETTRG, UINT8_MAX, UINT8_MAX}}; // settriger

void mode_menu() {
  uint8_t menu_num = 0;
  bool notpushed = true; // Press any button to be false
  count = 0;

  while(1) {
    dispseg();
    DLY_ms(1);

    // set disp
    seg_digit = 0;
    switch (mode_list[mode][menu_num]) {
      case MODE_5V:
        seg_num[0] = 10; // " "
        seg_num[1] = 5; // 5
        seg_num[2] = 10; // " "
        break;
      case MODE_FIX:
        seg_num[0] = 12; // F
        seg_num[1] = 13; // I
        seg_num[2] = 14; // X
        break;
      case MODE_PPS:
        seg_num[0] = 15; // P
        seg_num[1] = 15; // P
        seg_num[2] = 5; // S
        break;
      case MODE_CAL:
        seg_num[0] = 16; // C
        seg_num[1] = 17; // A
        seg_num[2] = 18; // L
        break;
      case MODE_TRG:
        seg_num[0] = 20; // T
        seg_num[1] = 22; // R
        seg_num[2] = 23; // G
        break;
      case MODE_SETTRG:
        seg_num[0] = 20; // T
        seg_num[1] = 22; // R
        seg_num[2] = 23; // G
        break;
      case MODE_DELTRG:
        seg_num[0] = 24; // D
        seg_num[1] = 19; // E
        seg_num[2] = 18; // L
        break;
      case MODE_VER:
        seg_num[0] = 25; // V
        seg_num[1] = 19; // E
        seg_num[2] = 22; // R
        break;
    }

    switch (readbutton()) {
      case BUTTON_NON:
        break;
      case BUTTON_ANY:
        break;
      case BUTTON_DOWN_SHORT:
        if (menu_num < MODE_LIST_MAX) {
          if (mode_list[mode][menu_num + 1] < UINT8_MAX) menu_num++; // next index
        }
        notpushed = false;
        break;
      case BUTTON_UP_SHORT:
        if (menu_num > 0) menu_num--; // previous index
        break;
      case BUTTON_CVCC_SHORT:
        if (mode == MODE_CAL) notpushed = false;
        break;
      case BUTTON_OP_SHORT:
        switch (mode_list[mode][menu_num]) {
          case MODE_5V:
            fiveVmode();
            break;
          case MODE_FIX:
            mode = MODE_FIX;
            fixmode_setup();
            fixmode_loop();
            break;
          case MODE_PPS:
            ppsmode_setup();
            ppsmode_loop();
            break;
          case MODE_CAL:
            calmode();
            break;
          case MODE_TRG:
            triggermode_setup();
            triggermode_loop();
            mode = MODE_TRG;
            break;
          case MODE_SETTRG:
            mode = MODE_SETTRG;
            triggersetmode_setup();
            triggersetmode_loop();
            break;
          case MODE_DELTRG:
            mode = MODE_DELTRG;
            triggerdelmode();
            break;
          case MODE_VER:
            if (notpushed) { // Prevents detection when releasing out&cvcc button
              notpushed = false;
            } else {
              vermode();
            }
            break;
          default:
            break;
        }
        break;
      default:
        break;
    }

    // trigger mode time out
    if (mode == MODE_TRG && notpushed) {
      count++;
      if (count > TRIGGER_TIMEOUT) {
        count = 0;
        triggermode_setup();
        triggermode_loop();
      }
    }
  }
}

// ===================================================================================
// pps mode
// ===================================================================================
void ppsmode_setup() {
  for (uint8_t i = 1; i <= PD_getPDONum(); i++) {
    if (i <= PD_getFixedNum());
    else if (max_Voltage < PD_getPDOMaxVoltage(i)) { // select more high voltage
      set_Voltage = DEFAULT_PPS_VOLTAGE;
      min_Voltage = PD_getPDOMinVoltage(i);
      max_Voltage = PD_getPDOMaxVoltage(i);
      max_Current = PD_getPDOMaxCurrent(i);
      set_Current = max_Current;
      min_Current = MIN_PPS_CURRENT;
      pdonum = i;
    }
  }

  // disp maxVoltage
  PIN_high(CV);
  PIN_low(CC);
  setseg(max_Voltage,true);
  do {
    count = readbutton();
    DLY_ms(1);
    dispseg();
  } while (!BUTTON_IS_SHORT(count));

  // disp minVoltage
  setseg(min_Voltage,true);
  do {
    count = readbutton();
    DLY_ms(1);
    dispseg();
  } while (!BUTTON_IS_SHORT(count));

  // disp maxCurrent
  PIN_low(CV);
  PIN_high(CC);
  setseg(max_Current,true);
  do {
    count = readbutton();
    DLY_ms(1);
    dispseg();
  } while (!BUTTON_IS_SHORT(count));

  // init
  count = 0;
  dispmode = DISPVOLTAGE;
  output = false;
  invalid_voltage = !PD_setPPS(min_Voltage, set_Current);
}


void ppsmode_loop() {
  uint16_t countkeep = 0;
  uint16_t temp_set_Voltage = set_Voltage; // use in long pressing. avoid detect invalid voltage in mesureVA
  uint16_t temp_set_Current = set_Current;
  bool countflag = false;

  while (1) {
    manage_onoff();
    manage_disp(temp_set_Voltage, temp_set_Current);
    mesureVA();
    count++;

    // Refresh disp
    if (count > MAXCOUNT) {
      count = 0;
      countkeep++;
      countflag = true;
      
      Voltage = (uint32_t)(sum_Voltage / MAXCOUNT);
      Current = (uint32_t)(sum_Current / MAXCOUNT);
      sum_Voltage = 0;
      sum_Current = 0;
      
      if (countkeep > KEEPTIME) {
        PD_PDO_request();
        countkeep = 0;
      }
    }

    // button
    switch (readbutton()) {
      case BUTTON_NON: // not pushed
        countflag = false;
        break;
      case BUTTON_ANY: // under processing
        break;
      case BUTTON_DOWN_SHORT:
        if (dispmode == DISPVOLTAGE) { // decrease voltage
          set_Voltage -= STEP_VOLTAGE_SHORT;
          if (set_Voltage < min_Voltage) set_Voltage = min_Voltage;
          temp_set_Voltage = set_Voltage;
        } else if (dispmode == DISPCURRENT) {  // decrease current
          set_Current -= STEP_CURRENT_SHORT;
          if (set_Current < min_Current) set_Current = min_Current;
          temp_set_Current = set_Current;
        }
        break;
      case BUTTON_DOWN_LONG_HOLD:
        if (dispmode == DISPVOLTAGE && countflag) {
          if ((set_Voltage - temp_set_Voltage) >= STEP_VOLTAGE_LONG_NEG) {
            set_Voltage = temp_set_Voltage;
          }
          temp_set_Voltage -= STEP_VOLTAGE_LONG;
          if (temp_set_Voltage < min_Voltage) {
            temp_set_Voltage = min_Voltage;
            set_Voltage = temp_set_Voltage;
          }
          countflag = false;
        } else if (dispmode == DISPCURRENT && countflag) {
          if ((set_Current - temp_set_Current) >= STEP_CURRENT_LONG_NEG) {
            set_Current = temp_set_Current;
          }
          temp_set_Current -= STEP_CURRENT_LONG;
          if (temp_set_Current < min_Current) {
            temp_set_Current = min_Current;
            set_Current = temp_set_Current;
          }
          countflag = false;
        }
        break;
      case BUTTON_DOWN_LONG_RELEASE:
        if (dispmode == DISPVOLTAGE || dispmode == DISPCURRENT) {
          set_Voltage = temp_set_Voltage;
          set_Current = temp_set_Current;
        }
        break;
      case BUTTON_UP_SHORT:
        if (dispmode == DISPVOLTAGE) {
          set_Voltage += STEP_VOLTAGE_SHORT;
          if (set_Voltage > max_Voltage) set_Voltage = max_Voltage;
          temp_set_Voltage = set_Voltage;
        } else if (dispmode == DISPCURRENT) {
          set_Current += STEP_CURRENT_SHORT;
          if (set_Current > max_Current) set_Current = max_Current;
          temp_set_Current = set_Current;
        }
        break;
      case BUTTON_UP_LONG_HOLD:
        if (dispmode == DISPVOLTAGE && countflag) {
          if ((temp_set_Voltage - set_Voltage) >= STEP_VOLTAGE_LONG_NEG) {
            set_Voltage = temp_set_Voltage;
          }
          temp_set_Voltage += STEP_VOLTAGE_LONG;
          if (temp_set_Voltage > max_Voltage) {
            temp_set_Voltage = max_Voltage;
            set_Voltage = temp_set_Voltage;
          }
          countflag = false;
        } else if (dispmode == DISPCURRENT && countflag) {
          if ((temp_set_Current - set_Current) >= STEP_CURRENT_LONG_NEG) {
            set_Current = temp_set_Current;
          }
          temp_set_Current += STEP_CURRENT_LONG;
          if (temp_set_Current > max_Current) {
            temp_set_Current = max_Current;
            set_Current = temp_set_Current;
          }
          countflag = false;
        }
        break;
      case BUTTON_UP_LONG_RELEASE:
        if (dispmode == DISPVOLTAGE || dispmode == DISPCURRENT) {
          set_Voltage = temp_set_Voltage;
          set_Current = temp_set_Current;
        }
        break;
      case BUTTON_CVCC_SHORT:
        switch (dispmode) {
          case DISPWATT:
            dispmode = DISPVOLTAGE;
            break;
          case DISPVOLTAGE:
            dispmode = DISPCURRENT;
            break;
          case DISPCURRENT:
            dispmode = DISPVOLTAGE;
            break;
        }
        break;
      case BUTTON_CVCC_LONG_HOLD:
        if (output && dispmode < DISPWATT) dispmode = DISPWATT;
        break;
      case BUTTON_OP_SHORT:
        if (!invalid_voltage) {
          output =! output;
        }
        break;
      default:
        break;
    }
  }
}

// ===================================================================================
// fix mode
// ===================================================================================
void fixmode_setup() {
  // find pdo with DEFAULT_FIX_VOLTAGE
  pdonum = 1;
  for (uint8_t i = 1; i <= PD_getFixedNum(); i++) {
    if (PD_getPDOVoltage(i) == DEFAULT_FIX_VOLTAGE) {
      pdonum = i;
      break;
    }
  }
  set_Voltage = PD_getPDOVoltage(pdonum);
  set_Current = PD_getPDOMaxCurrent(pdonum);

  // init
  count = 0;
  dispmode = DISPVOLTAGE;
  output = false;
  invalid_voltage = !PD_setVoltage(set_Voltage);
}


void fixmode_loop() {
  while (1) {
    manage_onoff();
    manage_disp(set_Voltage, set_Current);
    mesureVA();
    count++;
    if (count > MAXCOUNT) {
      count = 0;
      
      Voltage = (uint32_t)(sum_Voltage / MAXCOUNT);
      Current = (uint32_t)(sum_Current / MAXCOUNT);
      sum_Voltage = 0;
      sum_Current = 0;
    }

    switch (readbutton()) {
      case BUTTON_NON:
        break;
      case BUTTON_ANY:
        break;
      case BUTTON_DOWN_SHORT:
        if (output) {
          output = false;
          PIN_low(ONOFF);
        } else if (pdonum > 1) {
          pdonum--;
          set_Voltage = PD_getPDOVoltage(pdonum);
          set_Current = PD_getPDOMaxCurrent(pdonum);
          invalid_voltage = !PD_setVoltage(set_Voltage);
        }
        break;
      case BUTTON_UP_SHORT:
        if (output) {
          output = false;
          PIN_low(ONOFF);
        } else if (pdonum < PD_getFixedNum()) {
          pdonum++;
          set_Voltage = PD_getPDOVoltage(pdonum);
          set_Current = PD_getPDOMaxCurrent(pdonum);
          invalid_voltage = !PD_setVoltage(set_Voltage);
        }
        break;
      case BUTTON_CVCC_SHORT:
        switch (dispmode) {
          case DISPWATT:
            dispmode = DISPVOLTAGE;
            break;
          case DISPVOLTAGE:
            dispmode = DISPCURRENT;
            break;
          case DISPCURRENT:
            dispmode = DISPVOLTAGE;
            break;
        }
        break;
      case BUTTON_CVCC_LONG_HOLD:
        if (output && dispmode < DISPWATT) dispmode = DISPWATT;
        break;
      case BUTTON_OP_SHORT:
        if (!invalid_voltage) {
          output =! output;
        }
        break;
      default:
        break;
    }
  }
}

// ===================================================================================
// 5V mode
// ===================================================================================
void fiveVmode() {
  dispmode = DISPVOLTAGE;
  output = false;

  count = 0;
  set_Voltage = 5000;
  set_Current = LIMIT_FIVE_CURRENT;
  
  // check voltage and output
  invalid_voltage = false;
  sum_Voltage = 0;
  sum_Current = 0;
  mesureVA();
  output = true;
  if (!invalid_voltage) {
    PIN_high(ONOFF);
    output = true;
  } else {
    return;
  }
  Voltage = sum_Voltage;
  Current = sum_Current;
  sum_Voltage = 0;
  sum_Current = 0;

  while (1) {
    manage_onoff();
    manage_disp(set_Voltage, set_Current);
    mesureVA();
    if (invalid_voltage) return;
    count++;
    if (count > MAXCOUNT) {
      count = 0;
      
      Voltage = (uint32_t)(sum_Voltage / MAXCOUNT);
      Current = (uint32_t)(sum_Current / MAXCOUNT);
      sum_Voltage = 0;
      sum_Current = 0;
    }

    switch (readbutton()) {
      case BUTTON_CVCC_SHORT:
        if (output) {
          switch (dispmode) {
            case DISPWATT:
              dispmode = DISPVOLTAGE;
              break;
            case DISPVOLTAGE:
              dispmode = DISPCURRENT;
              break;
            case DISPCURRENT:
              dispmode = DISPVOLTAGE;
              break;
          }
        }
        break;
      case BUTTON_CVCC_LONG_HOLD:
        if (output && dispmode < DISPWATT) dispmode = DISPWATT;
        break;
      case BUTTON_OP_SHORT:
        PIN_low(ONOFF);
        output = false;
        PIN_low(CC);
        PIN_low(CV);
        return;
      default:
        break;
    }
  }
}

// ===================================================================================
// calibration mode
// ===================================================================================
void calmode() {
  uint16_t count = 0;
  uint32_t sum = 0;
  uint32_t aveV1 = 0;
  uint32_t aveV2 = 0;
  uint32_t aveA1 = 0;
  uint32_t aveA2 = 0;

  // calivration 5.00V
  setseg(CALV1,false);
  PIN_high(CV);
  PIN_low(CC);
  PIN_high(ONOFF);
  do {
    count = readbutton();
    DLY_ms(1);
    dispseg();
  } while (!BUTTON_IS_SHORT(count));
  sum = 0;
  for (int i = 0; i < MAXCOUNT; i++) {
    ADC_input(V_ADC);
    DLY_ms(1);
    sum += ADC_read();
  }
  aveV1 = (100 * sum) / MAXCOUNT;

  // calivration 18.00V
  setseg(CALV2,false);
  PIN_high(CV);
  PIN_low(CC);
  PIN_high(ONOFF);
  do {
    count = readbutton();
    DLY_ms(1);
    dispseg();
  } while (!BUTTON_IS_SHORT(count));
  sum = 0;
  for (int i = 0; i < MAXCOUNT; i++) {
    ADC_input(V_ADC);
    DLY_ms(1);
    sum += ADC_read();
  }
  aveV2 = (sum / MAXCOUNT) * 100;

  // calivration 0.00A
  PIN_low(ONOFF);
  setseg(CALA1,false);
  PIN_low(CV);
  PIN_high(CC);
  sum = 0;
  for (int i = 0; i < MAXCOUNT; i++) { // automatically done
    ADC_input(I_ADC);
    DLY_ms(1);
    sum += ADC_read();
  }
  aveA1 = (100 * sum) / MAXCOUNT;

  // calivration 3.00A
  setseg(CALA2,false);
  PIN_low(CV);
  PIN_high(CC);
  PIN_high(ONOFF);
  do {
    count = readbutton();
    DLY_ms(1);
    dispseg();
  } while (!BUTTON_IS_SHORT(count));
  sum = 0;
  for (int i = 0; i < MAXCOUNT; i++) {
    ADC_input(I_ADC);
    DLY_ms(1);
    sum += ADC_read();
  }
  aveA2 = (sum / MAXCOUNT) * 100;
  PIN_low(ONOFF);

  V_a = (1000 * 100 * (CALV2 - CALV1)) / (aveV2 - aveV1);
  V_b = 1000 * CALV2 - ((V_a * aveV2) / 100);
  I_a = (1000 * 100 * (CALA2 - CALA1)) / (aveA2 - aveA1);
  I_b = 1000 * CALA1 - ((I_a * aveA1) / 100);

  // disp result
  setseg(V_a,false);
  PIN_high(CV);
  PIN_low(CC);
  do {
    count = readbutton();
    DLY_ms(1);
    dispseg();
  } while (!BUTTON_IS_SHORT(count));
  
  setseg(I_a,false);
  PIN_high(CC);
  PIN_low(CV);
  do {
    count = readbutton();
    DLY_ms(1);
    dispseg();
  } while (!BUTTON_IS_SHORT(count));

  if (writeceff()) {
    DLY_ms(10);
    if (readcoeff()) {
      // sucess
      RST_now();
    }
  }
  seg_num[0] = 10; // " "
  seg_num[1] = 19; // E
  seg_num[2] = 10; // " "
  seg_digit = 0;
  while (1) {
    dispseg();
    DLY_ms(1);
  }
}

// ===================================================================================
// trigger mode
// ===================================================================================
void triggermode_setup() {
  pdonum = 0; // No pdonum selected
  
  seg_num[0] = 11; // -
  seg_num[1] = 11; // -
  seg_num[2] = 11; // -

  // select pdo
  for (uint8_t i = 1; i <= PD_getPDONum(); i++) {
    if (i <= PD_getFixedNum()) { // fix
      if (PD_getPDOVoltage(i) == triggervoltage && triggercurrent == UINT16_MAX) {
        set_Voltage = triggervoltage;
        set_Current = PD_getPDOMaxCurrent(i);
        pdonum = i;
        mode = MODE_FIX;
      }
    } else if (PD_getPDOMinVoltage(i) <= triggervoltage && triggervoltage <= PD_getPDOMaxVoltage(i) &&
    (triggercurrent <= PD_getPDOMaxCurrent(i) || triggercurrent == UINT16_MAX)) { // pps
      if ((pdonum && PD_getPDOMaxCurrent(i) > PD_getPDOMaxCurrent(pdonum)) || !pdonum) { // select higher current pdo
        set_Voltage = triggervoltage;
        min_Voltage = PD_getPDOMinVoltage(i);
        if (triggercurrent == UINT16_MAX) {
          set_Current = PD_getPDOMaxCurrent(i);
        } else {
          set_Current = triggercurrent;
        }
        pdonum = i;
        mode = MODE_PPS;
      }
    }
  }

  if (triggervoltage == 5000 && triggercurrent == UINT16_MAX && !pdonum) {
    set_Voltage = triggervoltage;
    set_Current = LIMIT_FIVE_CURRENT;
    mode = MODE_5V;
  }

  // set PD
  switch (mode) {
    case MODE_5V:
      break;
    case MODE_FIX:
      if (!PD_setPDO(pdonum, set_Voltage)) {
        while (1) {
          dispseg();
          DLY_ms(1);
        }
      }
      break;
    case MODE_PPS:
      if (!PD_setPPS(min_Voltage, set_Current)) {
        while (1) {
          dispseg();
          DLY_ms(1);
        }
      }
      break;
    case MODE_CAL:
      break;
    default: // Don't supply triggervoltage
      while (1) {
        dispseg();
        DLY_ms(1);
      }
      break;
  }

  // init
  count = 0;
  dispmode = DISPVOLTAGE;
  output = false;

  // check voltage and output
  invalid_voltage = false;
  sum_Voltage = 0;
  sum_Current = 0;
  mesureVA();
  output = true;
  if (!invalid_voltage) {
    output = true;
  } else {
    while (1) {
      dispseg();
      DLY_ms(1);
    }
  }
  Voltage = sum_Voltage;
  Current = sum_Current;
  sum_Voltage = 0;
  sum_Current = 0;
}

void triggermode_loop() {
  uint16_t countkeep = 0;

  while (1) {
    manage_onoff();
    manage_disp(set_Voltage, set_Current);
    mesureVA();
    count++;

    // Refresh disp
    if (count > MAXCOUNT) {
      count = 0;
      
      Voltage = (uint32_t)(sum_Voltage / MAXCOUNT);
      Current = (uint32_t)(sum_Current / MAXCOUNT);
      sum_Voltage = 0;
      sum_Current = 0;

      if (mode == MODE_PPS) {
        countkeep++;
        if (countkeep > KEEPTIME) {
          PD_PDO_request();
          countkeep = 0;
        }
      }
    }

    // button
    switch (readbutton()) {
      case BUTTON_NON:
        break;
      case BUTTON_ANY:
        break;
      case BUTTON_CVCC_SHORT:
        switch (dispmode) {
          case DISPWATT:
            dispmode = DISPVOLTAGE;
            break;
          case DISPVOLTAGE:
            dispmode = DISPCURRENT;
            break;
          case DISPCURRENT:
            dispmode = DISPVOLTAGE;
            break;
        }
        break;
      case BUTTON_CVCC_LONG_HOLD:
        if (dispmode < DISPWATT) dispmode = DISPWATT;
        break;
      default:
        break;
    }
  } 
}

// ===================================================================================
// trigger setup mode
// ===================================================================================
void triggersetmode_setup() {
  seg_num[0] = 5; // S
  seg_num[1] = 19; // E
  seg_num[2] = 20; // T
  do {
    count = readbutton();
    DLY_ms(1);
    dispseg();
  } while (!BUTTON_IS_SHORT(count));

  // init
  count = 0;
  dispmode = DISPVOLTAGE;
  output = false;
  triggervoltage = DEFAULT_PPS_VOLTAGE;
  triggercurrent = UINT16_MAX; // not set current limit
}

void triggersetmode_loop() {
  bool countflag = false;

  while (1) {
    DLY_ms(1);
    manage_disp(triggervoltage, triggercurrent);
    count++;
    if (count > MAXCOUNT) {
      count = 0;
      countflag = true;
    }

    // button
    switch (readbutton()) {
      case BUTTON_NON:
        countflag = false;
        break;
      case BUTTON_ANY:
        break;
      case BUTTON_DOWN_SHORT:
        if (dispmode == DISPVOLTAGE) {
          triggervoltage -= STEP_VOLTAGE_SHORT;
          if (triggervoltage < MIN_TRIGGER_V) triggervoltage = MIN_TRIGGER_V;
        } else if (dispmode == DISPCURRENT) {
          if (triggercurrent == UINT16_MAX) {
            triggercurrent = MAX_TRIGGER_A;
          } else {
            triggercurrent -= STEP_CURRENT_SHORT;
          }
          if (triggercurrent < MIN_TRIGGER_A) triggercurrent = MIN_TRIGGER_A;
        }
        break;
      case BUTTON_DOWN_LONG_HOLD:
        if (dispmode == DISPVOLTAGE && countflag) {
          triggervoltage -= STEP_VOLTAGE_LONG;
          if (triggervoltage < MIN_TRIGGER_V) triggervoltage = MIN_TRIGGER_V;
        } else if (dispmode == DISPCURRENT && countflag) {
          if (triggercurrent == UINT16_MAX) {
            triggercurrent = MAX_TRIGGER_A;
          } else {
            triggercurrent -= STEP_CURRENT_LONG;
          }
          if (triggercurrent < MIN_TRIGGER_A) triggercurrent = MIN_TRIGGER_A;
        }
        countflag = false;
        break;
      case BUTTON_UP_SHORT:
        if (dispmode == DISPVOLTAGE) {
          triggervoltage += STEP_VOLTAGE_SHORT;
          if (triggervoltage > MAX_TRIGGER_V) triggervoltage = MAX_TRIGGER_V;
        } else if (dispmode == DISPCURRENT) {
          triggercurrent += STEP_CURRENT_SHORT;
          if (triggercurrent > MAX_TRIGGER_A) triggercurrent = UINT16_MAX;
        }
        break;
      case BUTTON_UP_LONG_HOLD:
        if (dispmode == DISPVOLTAGE && countflag) {
          triggervoltage += STEP_VOLTAGE_LONG;
          if (triggervoltage > MAX_TRIGGER_V) triggervoltage = MAX_TRIGGER_V;
        } else if (dispmode == DISPCURRENT && countflag) {
          triggercurrent += STEP_CURRENT_LONG;
          if (triggercurrent > MAX_TRIGGER_A) triggercurrent = UINT16_MAX;
        }
        countflag = false;
        break;
      case BUTTON_CVCC_SHORT:
        switch (dispmode) {
          case DISPVOLTAGE:
            dispmode = DISPCURRENT;
            break;
          case DISPCURRENT:
            dispmode = DISPVOLTAGE;
            break;
        }
        break;
      case BUTTON_OP_SHORT:
        if (MIN_TRIGGER_V <= triggervoltage && triggervoltage <= MAX_TRIGGER_V && 
        ((MIN_TRIGGER_A <= triggercurrent && triggercurrent <= MAX_TRIGGER_A) || triggercurrent == UINT16_MAX)) {
          if (writeceff()) {
            DLY_ms(10);
            if (readtrigger()) {
              // sucess
              RST_now();
            }
          }
          seg_num[0] = 10; // " "
          seg_num[1] = 19; // E
          seg_num[2] = 10; // " "
          seg_digit = 0;
          while (1) {
            dispseg();
            DLY_ms(1);
          }
        }
        break;
      default:
        break;
    }
  }
}

// ===================================================================================
// trigger delete mode
// ===================================================================================
void triggerdelmode() {
  if (readtrigger()) {
    triggervoltage = 0;
    triggercurrent = 0;
    if (writeceff()) {
      DLY_ms(10);
      if (!readtrigger()) {
        // sucess
        RST_now();
      }
    }
    seg_num[0] = 10; // " "
    seg_num[1] = 19; // E
    seg_num[2] = 10; // " "
    seg_digit = 0;
    while (1) {
      dispseg();
      DLY_ms(1);
    }
  }
}

// ===================================================================================
// trigger delete mode
// ===================================================================================
void vermode() {
  setseg(VERSION, false);
  do {
    count = readbutton();
    DLY_ms(1);
    dispseg();
  } while (!BUTTON_IS_SHORT(count));
}
