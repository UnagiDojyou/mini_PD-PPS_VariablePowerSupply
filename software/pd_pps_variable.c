// ===================================================================================
// PD-PPS VariablePowerSupply
// firmware
// Author: Unagi Dojyou
// based on https://github.com/wagiminator
// License: http://creativecommons.org/licenses/by-sa/3.0/
// ===================================================================================
#include <config.h>             // user configurations
#include <system.h>             // system functions
#include <debug_serial.h>       // serial debug functions
#include <gpio.h>               // GPIO functions
#include <usbpd_sink.h>

#define bool _Bool
#define false ((bool)+0)
#define true ((bool)+1)

#define V_ADC PA4
#define I_ADC PA6
#define ONOFF PA5
#define CVCC PA7
#define BUTTON PC17

#define SEG_A1 PB12
#define SEG_A2 PA0
#define SEG_A3 PA1

#define SEG_A0 PC3
#define SEG_B PC19
#define SEG_C PC16
#define SEG_D PA2
#define SEG_E PA3
#define SEG_F PC1
#define SEG_G PB1
#define SEG_OP PC18

#define BUTTON_DOWN 1
#define BUTTON_UP 2
#define BUTTON_CVCC 3
#define BUTTON_OP 4
#define LONGCOUNT 250
#define SHORTCOUNT 2

#define V_COEFF 5809 //5.809
#define I_COEFF 8057 //8.057

#define MAXCOUNT 100
#define LONGPUSH_SPEED 100 //supeed of long push down/up

#define KEEPTIME 100 //Prevents the reset after about 10 seconds

void disable_swd(void) {
  uint32_t temp = AFIO->PCFR1;  // 現在のレジスタ値を読み込む
  temp &= ~AFIO_PCFR1_SWJ_CFG;          // 対象ビットフィールドをクリア
  temp |= 0x04000000;           // SWJ_CFGを100 (バイナリ) に設定
  AFIO->PCFR1 = temp;           // 新しい値をレジスタに書き込む
}

void enable_AFIO(void) {
  uint32_t temp = RCC->APB2PCENR;  // 現在のレジスタ値を読み込む
  temp &= ~RCC_AFIOEN;          // 対象ビットフィールドをクリア
  temp |= 0x00000001;           // AFIOENを1 (バイナリ) に設定
  RCC->APB2PCENR = temp;           // 新しい値をレジスタに書き込む
}

void segregisterset(uint8_t num){
  switch(num){
    case 0:
      GPIOA->BSHR = 0b00000000000000000000000000001111; //set SEG_D,SEG_E,SEG_A2,SEG_A3
      GPIOB->BSHR = 0b00000000000000100001000000000000; //reset SEG_G set SEG_A1
      GPIOC->BSHR = 0b00000000000000000000000000001010; //set SEG_A0,SEG_F
      GPIOC->BSXR = 0b00000000000001000000000000001001; //reset SEG_OP set SEG_B,SEG_C
      break;
    case 1:
      GPIOA->BSHR = 0b00000000000011000000000000000011;
      GPIOB->BSHR = 0b00000000000000100001000000000000;
      GPIOC->BSHR = 0b00000000000010100000000000000000;
      GPIOC->BSXR = 0b00000000000001000000000000001001;
      break;
    case 2:
      GPIOA->BSHR = 0b00000000000000000000000000001111;
      GPIOB->BSHR = 0b00000000000000000001000000000010;
      GPIOC->BSHR = 0b00000000000000100000000000001000;
      GPIOC->BSXR = 0b00000000000001010000000000001000;
      break;
    case 3:
      GPIOA->BSHR = 0b00000000000010000000000000000111;
      GPIOB->BSHR = 0b00000000000000000001000000000010;
      GPIOC->BSHR = 0b00000000000000100000000000001000;
      GPIOC->BSXR = 0b00000000000001000000000000001001;
      break;
    case 4:
      GPIOA->BSHR = 0b00000000000011000000000000000011;
      GPIOB->BSHR = 0b00000000000000000001000000000010;
      GPIOC->BSHR = 0b00000000000010000000000000000010;
      GPIOC->BSXR = 0b00000000000001000000000000001001;
      break;
    case 5:
      GPIOA->BSHR = 0b00000000000010000000000000000111;
      GPIOB->BSHR = 0b00000000000000000001000000000010;
      GPIOC->BSHR = 0b00000000000000000000000000001010;
      GPIOC->BSXR = 0b00000000000011000000000000000001;
      break;
    case 6:
      GPIOA->BSHR = 0b00000000000000000000000000001111;
      GPIOB->BSHR = 0b00000000000000000001000000000010;
      GPIOC->BSHR = 0b00000000000000000000000000001010;
      GPIOC->BSXR = 0b00000000000011000000000000000001;
      break;
    case 7:
      GPIOA->BSHR = 0b00000000000011000000000000000011;
      GPIOB->BSHR = 0b00000000000000100001000000000000;
      GPIOC->BSHR = 0b00000000000000000000000000001010;
      GPIOC->BSXR = 0b00000000000001000000000000001001;
      break;
    case 8:
      GPIOA->BSHR = 0b00000000000000000000000000001111;
      GPIOB->BSHR = 0b00000000000000000001000000000010;
      GPIOC->BSHR = 0b00000000000000000000000000001010;
      GPIOC->BSXR = 0b00000000000001000000000000001001;
      break;
    case 9:
      GPIOA->BSHR = 0b00000000000010000000000000000111;
      GPIOB->BSHR = 0b00000000000000000001000000000010;
      GPIOC->BSHR = 0b00000000000000000000000000001010;
      GPIOC->BSXR = 0b00000000000001000000000000001001;
      break;
    case 10: // no display
      GPIOA->BSHR = 0b00000000000011000000000000000011;
      GPIOB->BSHR = 0b00000000000000100001000000000000;
      GPIOC->BSHR = 0b00000000000010100000000000000000;
      GPIOC->BSXR = 0b00000000000011010000000000000000;
      break;
    case 11: // display "-"
      GPIOA->BSHR = 0b00000000000011000000000000000011;
      GPIOB->BSHR = 0b00000000000000000001000000000010;
      GPIOC->BSHR = 0b00000000000010100000000000000000;
      GPIOC->BSXR = 0b00000000000011010000000000000000;
      break;
    case 12: //F
      GPIOA->BSHR = 0b00000000000001000000000000001011;
      GPIOB->BSHR = 0b00000000000000000001000000000010;
      GPIOC->BSHR = 0b00000000000000000000000000001010;
      GPIOC->BSXR = 0b00000000000011010000000000000000;
      break;
    case 13: //I
      GPIOA->BSHR = 0b00000000000001000000000000001011;
      GPIOB->BSHR = 0b00000000000000100001000000000000;
      GPIOC->BSHR = 0b00000000000010000000000000000010;
      GPIOC->BSXR = 0b00000000000011010000000000000000;
      break;
    case 14: //X
      GPIOA->BSHR = 0b00000000000001000000000000001011;
      GPIOB->BSHR = 0b00000000000000000001000000000010;
      GPIOC->BSHR = 0b00000000000010000000000000000010;
      GPIOC->BSXR = 0b00000000000001000000000000001001;
      break;
    case 15: //P
      GPIOA->BSHR = 0b00000000000001000000000000001011;
      GPIOB->BSHR = 0b00000000000000000001000000000010;
      GPIOC->BSHR = 0b00000000000000000000000000001010;
      GPIOC->BSXR = 0b00000000000001010000000000001000;
      break;
    case 16: //C
      GPIOA->BSHR = 0b00000000000000000000000000001111;
      GPIOB->BSHR = 0b00000000000000100001000000000000;
      GPIOC->BSHR = 0b00000000000000000000000000001010;
      GPIOC->BSXR = 0b00000000000011010000000000000000;
      break;
    case 17: //A
      GPIOA->BSHR = 0b00000000000001000000000000001011;
      GPIOB->BSHR = 0b00000000000000000001000000000010;
      GPIOC->BSHR = 0b00000000000000000000000000001010;
      GPIOC->BSXR = 0b00000000000001000000000000001001;
      break;
    case 18: //L
      GPIOA->BSHR = 0b00000000000000000000000000001111;
      GPIOB->BSHR = 0b00000000000000100001000000000000;
      GPIOC->BSHR = 0b00000000000010000000000000000010;
      GPIOC->BSXR = 0b00000000000011010000000000000000;
      break;
    default:
      break;
  }
}

uint8_t seg_num[3]; //Left,Center,Right.
uint8_t seg_digit = 0; //0:none,1:Left,2:Center,3:Right
//set numbers(0~999000m) to display on 7seg
void setseg(uint32_t num,bool fix100m){
  uint16_t temp = 0;
  if(num < 10000){
    seg_digit = 1;
    temp = num / 10;
  }else if(num < 100000){
    seg_digit = 2;
    temp = num / 100;
  }else{
    seg_digit = 0;
    temp = num / 1000;
  }
  if(!fix100m || num >= 10000){ //use all segs. Ex 10.2, 5.02
    seg_num[0] = temp / 100;
    seg_num[1] = (temp / 10) % 10;
    seg_num[2] = temp % 10;
  }else{ //only use 2 segs. Ex 3.3, 9.9
    seg_digit = 2;
    seg_num[0] = 10; //empty
    seg_num[1] = temp/100;
    seg_num[2] = (temp / 10) % 10;
  }
}

uint8_t seg_driving = SEG_A1;
//7seg driver
void dispseg() {
  switch(seg_driving){
    case SEG_A3: //now Right,next is Left
      segregisterset(seg_num[0]);
      PIN_low(SEG_A1);
      if(seg_digit == 1){
        PIN_high(SEG_OP);
      }
      seg_driving = SEG_A1;
      break;
    case SEG_A1: //now Left,next is Center
      segregisterset(seg_num[1]);
      PIN_low(SEG_A2);
      if(seg_digit == 2){
        PIN_high(SEG_OP);
      }
      seg_driving = SEG_A2;
      break;
    case SEG_A2: //now Center,next is Right
      segregisterset(seg_num[2]);
      PIN_low(SEG_A3);
      if(seg_digit == 3){
        PIN_high(SEG_OP);
      }
      seg_driving = SEG_A3;
      break;
    default:
      seg_driving = SEG_A1;
      break;
  }
}

uint8_t button_pushed = 0; //last pushed button. 0 = none
uint8_t button_pushing = 0;
uint16_t button_count = 0; //how long button is pressed
bool button_sampled = false; //if any button is pushed ,be true
//short push return Normal,long pushing return *10,release after long press retun *10+
uint8_t readbotton(){
  if(PIN_read(BUTTON)){ //HIGH
    if(!button_sampled && seg_driving == SEG_A2){ //when SEG_A1 was low and SEG_A2 is high
      button_pushing = BUTTON_OP;
    }else if(seg_driving == SEG_A3 && button_pushing == 0 && button_sampled){ //always high
      button_pushing = BUTTON_DOWN;
    }
    button_sampled = true;
  }else if(button_sampled){
    switch(seg_driving){
      case SEG_A1:
        //none
        break;
      case SEG_A2:
        button_pushing = BUTTON_CVCC;
        break;
      case SEG_A3:
        if(button_pushing == 0){
          button_pushing = BUTTON_UP;
        }else{ //double press?
          button_sampled = false;
          button_pushing = 0;
        }
        break;
      default:
        break;
    }
  }else{ //released
    button_sampled = false;
    button_pushing = 0;
  }

  //count and reset
  if(seg_driving == SEG_A3){
    if(button_sampled){ //any button was pushed
      if(button_pushing == button_pushed){ //button is pushed
        button_count++;
        if(button_count > LONGCOUNT){
          button_pushing = 0;
          button_sampled = false;
          return button_pushed*10; //long pressing
        }
      }else{ //pushed button is changed or pressed
        button_count = 1;
        button_pushed = button_pushing;
      }
    }else if(button_pushed > 0){ //button is relased
      if(button_count > LONGCOUNT){
        button_count = 0;
        button_pushing = button_pushed; //temp
        button_pushed = 0;
        button_sampled = false;
        return button_pushing*10 + button_pushing; //relase after long press
      }else if(button_count > SHORTCOUNT){
        button_count = 0;
        button_pushing = button_pushed;
        button_pushed = 0;
        button_sampled = false;
        return button_pushing; //short pushed
      }else{ //chattering
        button_pushed = 0;
        button_count = 0;
      }
    }else{ //no button is pushed
      button_pushed = 0;
      button_count = 0;
      return 0;
    }
    //reset
    button_pushing = 0;
    button_sampled = false;
    return 0;
  }
  return 255;
}

uint8_t mode = 0; //0:5V, 1:Fix, 2:PPS, 3:Cal
bool ppsable = false;
bool fixable = false;

void setdefaultmode(){
  if(ppsable && fixable){
    mode = 2; //ppsmode
  }else if(fixable){
    mode = 1; //Fix
  }else{
    mode = 0;
  }
}

void setmode(){
  PIN_input(SEG_A1);
  PIN_input(SEG_A2);
  PIN_input(SEG_A3);
  if(PIN_read(BUTTON)){
    //Down button
    //it must be download mode
    setdefaultmode();
    return;
  }else{
    PIN_output(SEG_A3);
    PIN_high(SEG_A3);
    DLY_ms(1);
    if(PIN_read(BUTTON)){
      //up
      PIN_low(SEG_A3);
      setdefaultmode();
      return;
    }
    PIN_low(SEG_A3);
    PIN_input(SEG_A3);
    PIN_output(SEG_A2);
    PIN_high(SEG_A2);
    DLY_ms(1);
    if(PIN_read(BUTTON)){
      PIN_low(SEG_A2);
      PIN_input(SEG_A2);
      PIN_output(SEG_A1);
      PIN_high(SEG_A1);
      DLY_ms(1);
      if(PIN_read(BUTTON)){
        //CVCC and OP
        PIN_low(SEG_A1);
        mode = 3; //cal
        return;
      }else{
        //cvcc
        PIN_low(SEG_A1);
        setdefaultmode();
        return;
      }
    }
    PIN_low(SEG_A2);
    PIN_input(SEG_A2);
    PIN_output(SEG_A1);
    PIN_high(SEG_A1);
    DLY_ms(1);
    if(PIN_read(BUTTON) && fixable){ //fix mode
      //op
      PIN_low(SEG_A1);
      mode = 1;
      return;
    }
    PIN_low(SEG_A1);
  }
  setdefaultmode();
}

void ppsmode();
void fiveVmode();
void fixmode();
void calmode();

// ===================================================================================
// Main Function
// ===================================================================================
int main(void) {
  enable_AFIO();
  disable_swd();
  // Setup
  PIN_output(ONOFF);
  PIN_input(CVCC); //not display
  PIN_input_PD(BUTTON); //pull down
 
  PIN_output(SEG_A0);
  PIN_output(SEG_B);
  PIN_output(SEG_C);
  PIN_output(SEG_D);
  PIN_output(SEG_E);
  PIN_output(SEG_F);
  PIN_output(SEG_G);
  PIN_output(SEG_OP);
  
  PIN_low(ONOFF);
  seg_num[0] = 11;
  seg_num[1] = 11;
  seg_num[2] = 11;

  // Setup
  ADC_init();                   // init ADC
  ADC_slow();
  //DEBUG_init();                     // init DEBUG (TX: PA2, BAUD: 115200, 8N1)


  if(PD_connect()) {                     // init USB PD
    if(PD_getPPSNum() > 0){
      ppsable = true;
    }
    if(PD_getFixedNum() > 0){
      fixable = true;
    }
  }

  setmode();
  
  PIN_output(SEG_A1);
  PIN_output(SEG_A2);
  PIN_output(SEG_A3);

  // Loop
  switch(mode){
    case 0:
      fiveVmode();
      while(1);
    case 1:
      fixmode();
      while(1);
    case 2:
      ppsmode();
      while(1);
    case 3:
      calmode();
      while(1);
  }
}

void ppsmode(){
  uint16_t count = 0;
  uint16_t countkeep = 0;
  uint16_t mes_Voltage = 0;
  uint16_t mes_Current = 0;
  uint32_t sum_Voltage = 0;
  uint32_t sum_Current = 0;
  uint16_t set_Voltage = 5000;
  uint16_t set_Current = 3000;
  uint16_t min_Voltage = 0;
  uint16_t max_Voltage = 0;
  uint16_t min_Current = 0;
  uint16_t max_Current = 0;
  uint16_t Voltage = 0;
  uint16_t Current = 0;
  uint8_t pdonum = 0;
  bool dispV = true; //V or I
  bool output = false; //OFF
  bool countflag = false;
  bool outvolt = false;

  for(uint8_t i = 1; i <= PD_getPDONum(); i++) {
    if(i <= PD_getFixedNum());
    else if(max_Voltage < PD_getPDOMaxVoltage(i)){ //select more high voltage
      min_Voltage = PD_getPDOMinVoltage(i);
      max_Voltage = PD_getPDOMaxVoltage(i);
      max_Current = PD_getPDOMaxCurrent(i);
      set_Current = max_Current;
      pdonum = i;
    }
  }

  //disp PPS
  seg_num[0] = 15;
  seg_num[1] = 15;
  seg_num[2] = 5;
  do{
    count = readbotton();
    DLY_ms(1);
    dispseg();
  }while(count < BUTTON_DOWN || BUTTON_OP < count);

  //disp maxVoltage
  PIN_output(CVCC);
  PIN_low(CVCC);
  setseg(max_Voltage,true);
  do{
    count = readbotton();
    DLY_ms(1);
    dispseg();
  }while(count < BUTTON_DOWN || BUTTON_OP < count);
  //disp minVoltage
  setseg(min_Voltage,true);
  do{
    count = readbotton();
    DLY_ms(1);
    dispseg();
  }while(count < BUTTON_DOWN || BUTTON_OP < count);
  //disp maxCurrent
  PIN_high(CVCC);
  setseg(max_Current,true);
  do{
    count = readbotton();
    DLY_ms(1);
    dispseg();
  }while(count < BUTTON_DOWN || BUTTON_OP < count);

  count = 0;
  PIN_low(CVCC);

  while(1){
    ADC_input(V_ADC);
    mes_Voltage = (uint32_t)ADC_read() * V_COEFF / 1000;
    sum_Voltage += mes_Voltage;
    if(set_Voltage*0.9 >= mes_Voltage || mes_Voltage >= set_Voltage*1.1){ //stop over voltage
      if(output){
        output = false;
        PIN_low(ONOFF);
        outvolt = true;
      }
    }
    DLY_ms(1);
    ADC_input(I_ADC);
    mes_Current = (uint32_t)ADC_read() * I_COEFF / 1000;
    sum_Current += mes_Current;

    dispseg();
    count ++;
    if(count > MAXCOUNT){
      count = 0;
      countkeep ++;
      countflag = true;
      
      Voltage = sum_Voltage / MAXCOUNT;
      Current = sum_Current / MAXCOUNT;
      sum_Voltage = 0;
      sum_Current = 0;
      if(output){
        if(dispV){
          setseg(Voltage, false);
        }else{
          setseg(Current, false);
        }
      }else{
        if(dispV){
          setseg(set_Voltage ,true);
        }else{
          setseg(set_Current ,true);
        }
      }
      if(countkeep == KEEPTIME){
        PD_setVoltage(set_Voltage+20);
      }else if(countkeep >= (2 * KEEPTIME)){
        PD_setVoltage(set_Voltage);
        countkeep = 0;
      }
    }

    switch(readbotton()){
      case 0: //not pushed
        countflag = false;
        break;
      case 255: //under processing
        break;
      case BUTTON_DOWN:
        if(dispV && set_Voltage > min_Voltage){
          set_Voltage -= 100;
          if(!PD_setVoltage(set_Voltage)){
            outvolt = true;
          }else{
            outvolt =false;
          }
        }
        break;
      case BUTTON_DOWN*10: //long pressing
        if(dispV && countflag && set_Voltage > min_Voltage){
          set_Voltage -= 100;
          countflag = false;
        }
        break;
      case BUTTON_DOWN*10+BUTTON_DOWN: //down botton relased
          set_Voltage += 100;
          if(!PD_setVoltage(set_Voltage)){
            outvolt = true;
          }else{
            outvolt =false;
          }
        break;
      case BUTTON_UP:
        if(dispV && set_Voltage < max_Voltage){
          set_Voltage += 100;
          if(!PD_setVoltage(set_Voltage)){
            outvolt = true;
          }else{
            outvolt =false;
          }
        }
        break;
      case BUTTON_UP*10: //long pressing
        if(dispV && countflag && set_Voltage < max_Voltage){
          set_Voltage += 100;
          countflag = false;
        }
        break;
      case BUTTON_UP*10+BUTTON_UP: //up botton relased
        set_Voltage -= 100;
        if(!PD_setVoltage(set_Voltage)){
            outvolt = true;
          }else{
            outvolt =false;
          }
        break;
      case BUTTON_CVCC:
        dispV =! dispV;
        PIN_toggle(CVCC);
        break;
      case BUTTON_CVCC*10 + BUTTON_CVCC:
        break;
      case BUTTON_OP*10 + BUTTON_OP:
        break;
      case BUTTON_OP:
        if(set_Voltage*0.9 < Voltage && Voltage < set_Voltage*1.1 && !outvolt){
          output =! output;
          PIN_toggle(ONOFF);
        }else{ //Voltage Error
          outvolt = true;
        }
        break;
      default:
        break;
    }
  }
}

void fixmode(){
  uint16_t count = 0;
  uint16_t mes_Voltage = 0;
  uint16_t mes_Current = 0;
  uint32_t sum_Voltage = 0;
  uint32_t sum_Current = 0;
  uint16_t Voltage = 0;
  uint16_t Current = 0;
  uint8_t pdonum = 1;
  bool dispV = true; //V or I
  bool output = false; //ON or OFF
  bool outvolt = false;

  //disp FIX
  seg_num[0] = 12; //F
  seg_num[1] = 13; //I
  seg_num[2] = 14; //X
  if(ppsable){
    do{
      count = readbotton();
      DLY_ms(1);
      dispseg();
    }while(count < BUTTON_DOWN || BUTTON_OP < count);
  }
  do{
    count = readbotton();
    DLY_ms(1);
    dispseg();
  }while(count < BUTTON_DOWN || BUTTON_OP < count);
  
  PIN_output(CVCC);
  PIN_low(CVCC);

  while(1){
    ADC_input(V_ADC);
    mes_Voltage = (uint32_t)ADC_read() * V_COEFF / 1000;
    sum_Voltage += mes_Voltage;
    if(PD_getPDOVoltage(pdonum)*0.9 >= mes_Voltage || mes_Voltage >= PD_getPDOVoltage(pdonum)*1.1){ //stop over voltage
      if(output){
        output = false;
        PIN_low(ONOFF);
        outvolt = true;
      }
    }
    DLY_ms(1);
    ADC_input(I_ADC);
    mes_Current = (uint32_t)ADC_read() * I_COEFF / 1000;
    sum_Current += mes_Current;

    dispseg();
    count ++;
    if(count > MAXCOUNT){
      count = 0;
      
      Voltage = sum_Voltage / MAXCOUNT;
      Current = sum_Current / MAXCOUNT;
      sum_Voltage = 0;
      sum_Current = 0;
      if(output){
        if(dispV){
          setseg(Voltage, false);
        }else{
          setseg(Current, false);
        }
      }else{
        if(dispV){
          setseg(PD_getPDOVoltage(pdonum) ,true);
        }else{
          setseg(PD_getPDOMaxCurrent(pdonum) ,true);
        }
      }
      if(!outvolt){
        //PD_setPDO_noretrun(pdonum, PD_getPDOVoltage(pdonum)); //set voltage with no check
      }
    }

    switch(readbotton()){
      case 0:
        break;
      case 255:
        break;
      case BUTTON_DOWN:
        if(output){
          output = false;
          PIN_low(ONOFF);
        }else if(pdonum > 1){
          pdonum--;
          if(!PD_setVoltage(PD_getPDOVoltage(pdonum))){
            outvolt = true;
          }else{
            outvolt =false;
          }
        }
        break;
      case BUTTON_UP:
        if(output){
          output = false;
          PIN_low(ONOFF);
        }else if(pdonum < PD_getFixedNum()){
          pdonum++;
          if(!PD_setVoltage(PD_getPDOVoltage(pdonum))){
            outvolt = true;
          }else{
            outvolt =false;
          }
        }
        break;
      case BUTTON_CVCC:
        dispV =! dispV;
        PIN_toggle(CVCC);
        break;
      case BUTTON_OP:
        if(PD_getPDOVoltage(pdonum)*0.9 < Voltage && Voltage < PD_getPDOVoltage(pdonum)*1.1 && !outvolt){
          output =! output;
          PIN_toggle(ONOFF);
        }else{ //Voltage Error
          outvolt = true;
        }
        break;
      default:
        break;
    }
  }
}

void fiveVmode(){
  uint16_t count = 0;
  uint16_t mes_Voltage = 0;
  uint16_t mes_Current = 0;
  uint32_t sum_Voltage = 0;
  uint32_t sum_Current = 0;
  uint16_t set_Voltage = 5000;
  uint16_t Voltage = 0;
  uint16_t Current = 0;
  bool dispV = true; //V or I
  bool output = false; //OFF
  bool outvolt = false;

  //disp 5
  seg_num[0] = 10; //" "
  seg_num[1] = 5; //5
  seg_num[2] = 10; //" "
  do{
    count = readbotton();
    DLY_ms(1);
    dispseg();
  }while(count < BUTTON_DOWN || BUTTON_OP < count);
  
  PIN_output(CVCC);
  PIN_low(CVCC);

  while(1){
    ADC_input(V_ADC);
    mes_Voltage = (uint32_t)ADC_read() * V_COEFF / 1000;
    sum_Voltage += mes_Voltage;
    if(set_Voltage*0.9 >= mes_Voltage || mes_Voltage >= set_Voltage*1.1){ //stop over voltage
      if(output){
        output = false;
        PIN_low(ONOFF);
        outvolt = true;
      }
    }
    DLY_ms(1);
    ADC_input(I_ADC);
    mes_Current = (uint32_t)ADC_read() * I_COEFF / 1000;
    sum_Current += mes_Current;

    dispseg();
    count ++;
    if(count > MAXCOUNT){
      count = 0;
      
      Voltage = sum_Voltage / MAXCOUNT;
      Current = sum_Current / MAXCOUNT;
      sum_Voltage = 0;
      sum_Current = 0;
      if(output){
        if(dispV){
          setseg(Voltage, false);
        }else{
          setseg(Current, false);
        }
      }else{
        seg_num[0] = 10; //" "
        seg_num[1] = 5; //5
        seg_num[2] = 10; //" "
        seg_digit = 0;
      }
    }

    switch(readbotton()){
      case BUTTON_OP:
        if(set_Voltage*0.9 < Voltage && Voltage < set_Voltage*1.1 && !outvolt){
          output =! output;
          PIN_toggle(ONOFF);
        }else{ //Voltage Error
          outvolt = true;
        }
        break;
      case BUTTON_CVCC:
        dispV =! dispV;
        PIN_toggle(CVCC);
        break;
    }
  }
}

void calmode(){
  uint16_t count = 0;
  //disp 5
  seg_num[0] = 16; //C
  seg_num[1] = 17; //A
  seg_num[2] = 18; //Ls
  do{
    count = readbotton();
    DLY_ms(1);
    dispseg();
  }while(count < BUTTON_DOWN || BUTTON_OP < count);
}

