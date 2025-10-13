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
#include <flash_rom.h>          // flash rom functions

#include "7seg_button.h"
#include "pin_define.h"

// ===================================================================================
// configuration
// ===================================================================================

// if DEBUG is defined, calibration is disaled.
// #define DEBUG
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

// OPA addr
#define OPA_CTLR1_PSEL2_MASK 0x00180000

// ===================================================================================
// functions
// ===================================================================================

void OPA_enable() {
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

uint8_t mode = 0; // 0:5V, 1:Fix, 2:PPS, 3:Cal, 4:trg 5:settrg
bool ppsable = false;
bool fixable = false;
uint32_t V_a = 0;
int32_t V_b = 0;
uint32_t I_a = 0;
int32_t I_b = 0;
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
uint32_t triggervoltage = 0;
uint32_t triggercurrent = 0;

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
  PIN_input(PIN_SEG_A1);
  PIN_input(PIN_SEG_A2);
  PIN_input(PIN_SEG_A3);
  if (PIN_read(PIN_BOOT_BUTTON)) {
    // Down button
    // it must be download mode
    setdefaultmode();
    return;
  } else {
    PIN_output(PIN_SEG_A3);
    PIN_low(PIN_SEG_A3);
    DLY_ms(1);
    if (!PIN_read(PIN_BUTTON)) {
      // up
      setdefaultmode();
      return;
    }
    PIN_input(PIN_SEG_A3);
    PIN_output(PIN_SEG_A2);
    PIN_low(PIN_SEG_A2);
    DLY_ms(1);
    if (!PIN_read(PIN_BUTTON)) {
      PIN_input(PIN_SEG_A2);
      PIN_output(PIN_SEG_A1);
      PIN_low(PIN_SEG_A1);
      DLY_ms(1);
      if (!PIN_read(PIN_BUTTON)) {
        // CVCC and OP
        mode = MODE_CAL;
        return;
      } else {
        // CVCC
        PIN_low(PIN_SEG_A1);
        mode = MODE_SETTRG;
        return;
      }
    }
    PIN_input(PIN_SEG_A2);
    PIN_output(PIN_SEG_A1);
    PIN_low(PIN_SEG_A1);
    DLY_ms(1);
    if (!PIN_read(PIN_BUTTON) && fixable) { // fix mode
      // op
      mode = MODE_FIX;
      return;
    }
  }
  setdefaultmode();
}

bool readcoeff() {
  V_a = FLASH_read4Byte(V_A_ADD);
  V_b = (int32_t)FLASH_read4Byte(V_B_ADD);
  I_a = FLASH_read4Byte(I_A_ADD);
  I_b = (int32_t)FLASH_read4Byte(I_B_ADD);
  return !(V_a == 0xFFFFFFFF || V_b == 0xFFFFFFFF || I_a == 0xFFFFFFFF || I_b == 0xFFFFFFFF || V_a == 0 || I_a == 0);
}

bool readtrigger() {
  triggervoltage = FLASH_read4Byte(TRIGGER_V_ADD);
  triggercurrent = FLASH_read4Byte(TRIGGER_A_ADD);
  return (MIN_TRIGGER_V <= triggervoltage && triggervoltage <= MAX_TRIGGER_V && 
  ((MIN_TRIGGER_A <= triggercurrent && triggercurrent <= MAX_TRIGGER_A) || triggercurrent == UINT16_MAX));
}

bool writeceff() {
  if (V_a == 0 && V_b == 0 && I_a == 0) { // can't read flash
    return false;
  }
  if (!(FLASH_read4Byte(USE_FLASH_END + 0x4) == 0xFFFFFFFF)) { // check using
    return false;
  }
  if (!(FLASH_read4Byte(USE_FLASH_ADD - 0x4) == 0xFFFFFFFF)) { // check using
    return false;
  }
  if (!FLASH_erasePage(USE_FLASH_ADD)) {
    return false;
  }
  uint32_t message[8] = {V_a, V_b, I_a, I_b, triggervoltage, triggercurrent, UINT32_MAX, UINT32_MAX};
  if (!FLASH_write32Byte(USE_FLASH_ADD, message)) {
    return false;
  }
  FLASH_relock();
  return true;
}

// read ADC and check
void mesureVA() {
  // measure Voltage and Current
  ADC_input(PIN_V_ADC);
  mes_Voltage = (ADC_read() * V_a) / 1000 + V_b / 1000;
  #ifndef DEBUG
  if (mes_Voltage >= set_Voltage + LIMIT_VOLTAGE) { // stop over voltage
    if (output) {
      output = false;
      PIN_low(PIN_ONOFF);
      invalid_voltage = true;
    }
  }
  #endif
  if (mes_Voltage < 0) mes_Voltage = 0;
  sum_Voltage += mes_Voltage;
  DLY_ms(1);
  ADC_input(PIN_I_ADC);
  mes_Current = (ADC_read() * I_a) / 1000 + I_b / 1000;
  #ifndef DEBUG
  if (mes_Current > set_Current + LIMIT_CURRENT) { // stop over current
    if (output) {
      output = false;
      PIN_low(PIN_ONOFF);
      invalid_voltage = true;
    }
  }
  #endif
  if (mes_Current < 0) mes_Current = 0;
  sum_Current += mes_Current;
}

void manage_onoff() {
  if (mode == MODE_PPS) {
    if (!PIN_read(PIN_ONOFF) && output && !invalid_voltage) { // OFF -> ON
      PIN_high(PIN_ONOFF);
      invalid_voltage = !PD_setPPS(set_Voltage, set_Current);
    } else if (PIN_read(PIN_ONOFF) && !output) { // ON -> OFF
      PIN_low(PIN_ONOFF);
      invalid_voltage = !PD_setPPS(min_Voltage, set_Current);
    }
    if ((set_Voltage != PD_getVoltage() || set_Current != PD_getCurrent()) && output && !invalid_voltage) {
      invalid_voltage = !PD_setPPS(set_Voltage, set_Current);
    }
  } else if (output && !invalid_voltage) {
    PIN_high(PIN_ONOFF);
  } else {
    PIN_low(PIN_ONOFF);
  }
}

void manage_disp(uint16_t disp_set_Voltage, uint16_t disp_set_Current) {
  if (output) { // ON
    switch (dispmode) {
      case DISPVOLTAGE:
        SEG_number(Voltage, false);
        PIN_high(PIN_CV);
        PIN_low(PIN_CC);
        break;
      case DISPCURRENT:
        SEG_number(Current, false);
        PIN_low(PIN_CV);
        PIN_high(PIN_CC);
        break;
      case DISPWATT:
        SEG_number(Voltage * Current / 1000, false);
        PIN_high(PIN_CV);
        PIN_high(PIN_CC);
        break;
    }
  } else { // OFF
    switch (dispmode) {
      case DISPCURRENT:
        PIN_low(PIN_CV);
        PIN_high(PIN_CC);
        if (disp_set_Current == UINT16_MAX) {
          SEG_setEach(SEG_NON, SEG_MINUS, SEG_NON, 0); // " - "
        } else {
          SEG_number(disp_set_Current, true);
        }
        break;
      case DISPWATT:
        dispmode = DISPVOLTAGE;
        // not need break
      case DISPVOLTAGE:
        SEG_number(disp_set_Voltage, true);
        PIN_high(PIN_CV);
        PIN_low(PIN_CC);
        break;
    }
  }
  SEG_driver();
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
  PIN_output(PIN_ONOFF);
  PIN_output(PIN_CV);
  PIN_output(PIN_CC);
  PIN_input_PD(PIN_BOOT_BUTTON); // pull down
  PIN_input_PU(PIN_BUTTON); // pull up
 
  PIN_output(PIN_SEG_A0);
  PIN_output(PIN_SEG_B);
  PIN_output(PIN_SEG_C);
  PIN_output(PIN_SEG_D);
  PIN_output(PIN_SEG_E);
  PIN_output(PIN_SEG_F);
  PIN_output(PIN_SEG_G);
  PIN_output(PIN_SEG_OP);
  
  PIN_low(PIN_ONOFF);

  // Setup
  OPA_enable();
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
  
  PIN_output(PIN_SEG_A1);
  PIN_output(PIN_SEG_A2);
  PIN_output(PIN_SEG_A3);

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
    SEG_driver();
    DLY_ms(1);

    // set disp
    switch (mode_list[mode][menu_num]) {
      case MODE_5V:
        SEG_setEach(SEG_NON, SEG_5, SEG_NON, 0); // " 5 "
        break;
      case MODE_FIX:
        SEG_setEach(SEG_F, SEG_I, SEG_X, 0); // "FIX"
        break;
      case MODE_PPS:
        SEG_setEach(SEG_P, SEG_P, SEG_S, 0); // "PPS"
        break;
      case MODE_CAL:
        SEG_setEach(SEG_C, SEG_A, SEG_L, 0); // "CAL"
        break;
      case MODE_TRG:
        SEG_setEach(SEG_T, SEG_R, SEG_G, 0); // "TRG"
        break;
      case MODE_SETTRG:
        SEG_setEach(SEG_T, SEG_R, SEG_G, 0); // "TRG"
        break;
      case MODE_DELTRG:
        SEG_setEach(SEG_D, SEG_E, SEG_L, 0); // "DEL"
        break;
      case MODE_VER:
        SEG_setEach(SEG_V, SEG_E, SEG_R, 0); // "VER"
        break;
    }

    switch (BUTTON_read()) {
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
  PIN_high(PIN_CV);
  PIN_low(PIN_CC);
  SEG_number(max_Voltage,true);
  do {
    count = BUTTON_read();
    DLY_ms(1);
    SEG_driver();
  } while (!BUTTON_IS_SHORT(count));

  // disp minVoltage
  SEG_number(min_Voltage,true);
  do {
    count = BUTTON_read();
    DLY_ms(1);
    SEG_driver();
  } while (!BUTTON_IS_SHORT(count));

  // disp maxCurrent
  PIN_low(PIN_CV);
  PIN_high(PIN_CC);
  SEG_number(max_Current,true);
  do {
    count = BUTTON_read();
    DLY_ms(1);
    SEG_driver();
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
    switch (BUTTON_read()) {
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

    switch (BUTTON_read()) {
      case BUTTON_NON:
        break;
      case BUTTON_ANY:
        break;
      case BUTTON_DOWN_SHORT:
        if (output) {
          output = false;
          PIN_low(PIN_ONOFF);
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
          PIN_low(PIN_ONOFF);
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
    PIN_high(PIN_ONOFF);
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

    switch (BUTTON_read()) {
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
        PIN_low(PIN_ONOFF);
        output = false;
        PIN_low(PIN_CC);
        PIN_low(PIN_CV);
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
  SEG_number(CALV1,false);
  PIN_high(PIN_CV);
  PIN_low(PIN_CC);
  PIN_high(PIN_ONOFF);
  do {
    count = BUTTON_read();
    DLY_ms(1);
    SEG_driver();
  } while (!BUTTON_IS_SHORT(count));
  sum = 0;
  for (int i = 0; i < MAXCOUNT; i++) {
    ADC_input(PIN_V_ADC);
    DLY_ms(1);
    sum += ADC_read();
  }
  aveV1 = (100 * sum) / MAXCOUNT;

  // calivration 18.00V
  SEG_number(CALV2,false);
  PIN_high(PIN_CV);
  PIN_low(PIN_CC);
  PIN_high(PIN_ONOFF);
  do {
    count = BUTTON_read();
    DLY_ms(1);
    SEG_driver();
  } while (!BUTTON_IS_SHORT(count));
  sum = 0;
  for (int i = 0; i < MAXCOUNT; i++) {
    ADC_input(PIN_V_ADC);
    DLY_ms(1);
    sum += ADC_read();
  }
  aveV2 = (sum / MAXCOUNT) * 100;

  // calivration 0.00A
  PIN_low(PIN_ONOFF);
  SEG_number(CALA1,false);
  PIN_low(PIN_CV);
  PIN_high(PIN_CC);
  sum = 0;
  for (int i = 0; i < MAXCOUNT; i++) { // automatically done
    ADC_input(PIN_I_ADC);
    DLY_ms(1);
    sum += ADC_read();
  }
  aveA1 = (100 * sum) / MAXCOUNT;

  // calivration 3.00A
  SEG_number(CALA2,false);
  PIN_low(PIN_CV);
  PIN_high(PIN_CC);
  PIN_high(PIN_ONOFF);
  do {
    count = BUTTON_read();
    DLY_ms(1);
    SEG_driver();
  } while (!BUTTON_IS_SHORT(count));
  sum = 0;
  for (int i = 0; i < MAXCOUNT; i++) {
    ADC_input(PIN_I_ADC);
    DLY_ms(1);
    sum += ADC_read();
  }
  aveA2 = (sum / MAXCOUNT) * 100;
  PIN_low(PIN_ONOFF);

  V_a = (1000 * 100 * (CALV2 - CALV1)) / (aveV2 - aveV1);
  V_b = 1000 * CALV2 - ((V_a * aveV2) / 100);
  I_a = (1000 * 100 * (CALA2 - CALA1)) / (aveA2 - aveA1);
  I_b = 1000 * CALA1 - ((I_a * aveA1) / 100);

  // disp result
  SEG_number(V_a,false);
  PIN_high(PIN_CV);
  PIN_low(PIN_CC);
  do {
    count = BUTTON_read();
    DLY_ms(1);
    SEG_driver();
  } while (!BUTTON_IS_SHORT(count));
  
  SEG_number(I_a,false);
  PIN_high(PIN_CC);
  PIN_low(PIN_CV);
  do {
    count = BUTTON_read();
    DLY_ms(1);
    SEG_driver();
  } while (!BUTTON_IS_SHORT(count));

  if (writeceff()) {
    DLY_ms(10);
    if (readcoeff()) {
      // sucess
      RST_now();
    }
  }
  SEG_setEach(SEG_NON, SEG_E, SEG_NON, 0); // " E "
  while (1) {
    SEG_driver();
    DLY_ms(1);
  }
}

// ===================================================================================
// trigger mode
// ===================================================================================
void triggermode_setup() {
  pdonum = 0; // No pdonum selected
  
  SEG_setEach(SEG_MINUS, SEG_MINUS, SEG_MINUS, 0); // "---"

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
          SEG_driver();
          DLY_ms(1);
        }
      }
      break;
    case MODE_PPS:
      if (!PD_setPPS(min_Voltage, set_Current)) {
        while (1) {
          SEG_driver();
          DLY_ms(1);
        }
      }
      break;
    case MODE_CAL:
      break;
    default: // Don't supply triggervoltage
      while (1) {
        SEG_driver();
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
      SEG_driver();
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
    switch (BUTTON_read()) {
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
  SEG_setEach(SEG_S, SEG_E, SEG_T, 0); // "SET"
  do {
    count = BUTTON_read();
    DLY_ms(1);
    SEG_driver();
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
    switch (BUTTON_read()) {
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
          SEG_setEach(SEG_NON ,SEG_E, SEG_NON, 0); // " E "
          while (1) {
            SEG_driver();
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
    SEG_setEach(SEG_NON ,SEG_E, SEG_NON, 0); // " E "
    while (1) {
      SEG_driver();
      DLY_ms(1);
    }
  }
}

// ===================================================================================
// trigger delete mode
// ===================================================================================
void vermode() {
  SEG_number(VERSION, false);
  do {
    count = BUTTON_read();
    DLY_ms(1);
    SEG_driver();
  } while (!BUTTON_IS_SHORT(count));
}
