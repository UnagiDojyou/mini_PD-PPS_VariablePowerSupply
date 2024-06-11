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

//if DEBUG is defined, calibration is not enable.
//#define DEBUG

#define bool _Bool
#define false ((bool)+0)
#define true ((bool)+1)

#define V_ADC PA4
#define I_ADC PA6
#define ONOFF PA5
#define CVCC PA7 //low:V high:A
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

#define MAXCOUNT 100
#define LONGPUSH_SPEED 100 //supeed of long push down/up

#define KEEPTIME 80 //Prevents the reset after about 10 seconds

#define USE_FLASH_ADD 0x800BF00 //page192
#define V_A_ADD USE_FLASH_ADD
#define V_B_ADD (V_A_ADD + 0x4)
#define I_A_ADD (V_B_ADD + 0x4)
#define TRIGGER_ADD (I_A_ADD + 0x4)
#define USE_FLASH_END TRIGGER_ADD

#define FLASH_END 0x0800F7FF //end of main flash memory

#define CALV1 5000 //calictaion at 5V
#define CALV2 18000 //calictaion at 18V
#define CALA 3000 ////calictaion at 3A
#define R3 0.1 //0.1Ω
#define R8 47000 //47kΩ
#define R9 7500 //7.5kΩ
#define VDD 3000 //3V

#define DISPVOLTAGE 0
#define DISPCURRENT 1
#define DISPWATT 2

#define MAX_TRIGGER_V 21000 //21V
#define MIN_TRIGGER_V 3300 //3.3V

void disable_swd(void) {
  uint32_t temp = AFIO->PCFR1; //read reg
  temp &= ~AFIO_PCFR1_SWJ_CFG; //clear SWG_CFG
  temp |= 0x04000000; //set SWJ_CFG as 100
  AFIO->PCFR1 = temp; //write reg
}

void enable_AFIO(void) {
  uint32_t temp = RCC->APB2PCENR; //read reg
  temp &= ~RCC_AFIOEN; //clear RCC_AFIOEN
  temp |= 0x00000001; //set AFIOEN
  RCC->APB2PCENR = temp; //write reg
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
    case 19: //E
      GPIOA->BSHR = 0b00000000000000000000000000001111;
      GPIOB->BSHR = 0b00000000000000000001000000000010;
      GPIOC->BSHR = 0b00000000000000000000000000001010;
      GPIOC->BSXR = 0b00000000000011010000000000000000;
      break;
    case 20: //T
      GPIOA->BSHR = 0b00000000000000000000000000001111;
      GPIOB->BSHR = 0b00000000000000000001000000000010;
      GPIOC->BSHR = 0b00000000000010000000000000000010;
      GPIOC->BSXR = 0b00000000000011010000000000000000;
      break;
    case 22: //R
      GPIOA->BSHR = 0b00000000000001000000000000001011;
      GPIOB->BSHR = 0b00000000000000000001000000000010;
      GPIOC->BSHR = 0b00000000000010100000000000000000;
      GPIOC->BSXR = 0b00000000000011010000000000000000;
      break;
    case 23: //G
      GPIOA->BSHR = 0b00000000000000000000000000001111;
      GPIOB->BSHR = 0b00000000000000100001000000000000;
      GPIOC->BSHR = 0b00000000000000000000000000001010;
      GPIOC->BSXR = 0b00000000000011000000000000000001;
      break;
    case 24: //D
      GPIOA->BSHR = 0b00000000000000000000000000001111;
      GPIOB->BSHR = 0b00000000000000000001000000000010;
      GPIOC->BSHR = 0b00000000000010100000000000000000;
      GPIOC->BSXR = 0b00000000000001000000000000001001;
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
uint8_t readbutton(){
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

uint8_t mode = 0; //0:5V, 1:Fix, 2:PPS, 3:Cal, 4:trg 5:settrg
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
        mode = 5; //trigger set
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

bool UnlockFlash(){
  FLASH->KEYR = FLASH_KEY1;
  FLASH->KEYR = FLASH_KEY2;
  uint32_t temp = FLASH->CTLR; //read FLASH_CTLR
  return !(temp & FLASH_CTLR_LOCK);
}

bool FastUnlockFlash(){
  FLASH->MODEKEYR = FLASH_KEY1;
  FLASH->MODEKEYR = FLASH_KEY2;
  uint32_t temp = FLASH->CTLR; //read FLASH_CTLR
  return !(temp & FLASH_CTLR_FLOCK);
}

//when EOP is 1 return true,else return false
bool WaitFlashBusy(){
  uint32_t temp;
  do{
    temp = FLASH->STATR; //read FLASH_STATR
  }while(temp & FLASH_STATR_BSY); //while(BSY==1)
  if(!(temp & FLASH_STATR_EOP)){ //check EOP
    FLASH->CTLR = 0; //reset CTLR
    return false;
  }
  FLASH->STATR = temp; //reset EOP
  return true;
}

//Erase Flash Page(256Byte). "address" is Page start address.
bool FastEraseFlash(uint32_t address){
  //1) check locks
  uint32_t temp = FLASH->CTLR; //read FLASH_CTLR
  if(temp & FLASH_CTLR_LOCK){ //check lock
    if(!UnlockFlash()){
      return false;
    }
  }
  //2)
  temp = FLASH->CTLR;
  if(temp & FLASH_CTLR_FLOCK){ //check flock
    if(!FastUnlockFlash()){
      return false;
    }
  }
  //3) check other operation
  WaitFlashBusy();
  //4) set erase mode
  FLASH->CTLR = FLASH_CTLR_FTER; //set FTER bit
  //5) set erase address
  FLASH->ADDR = address;
  //6) start erasec
  temp = FLASH->CTLR;
  temp |= FLASH_CTLR_STRT; //set FTER STRT
  FLASH->CTLR = temp; //write reg
  //7,8) wait erase
  if(!WaitFlashBusy())return false;
  //9) reset CTRL
  FLASH->CTLR = 0;
  return true;
}

//Write Flash only 16Byte. "message" must be uint32_t[4]. "address" is Page start address.
bool FastWriteFlash16(uint32_t address,uint32_t *message){
  //check locks
  uint32_t temp = FLASH->CTLR; //read FLASH_CTLR
  if(temp & FLASH_CTLR_LOCK){ //check lock
    if(!UnlockFlash()){
      return false;
    }
  }
  temp = FLASH->CTLR;
  if(temp & FLASH_CTLR_FLOCK){ //check flock
    if(!FastUnlockFlash()){
      return false;
    }
  }
  //3) check other operation
  WaitFlashBusy();
  //4) set programming mode
  FLASH->CTLR = FLASH_CTLR_FTPG; //set FTPG
  //15)
  for(int i = 0;i < 16;i++){
    //5)
    temp = FLASH->CTLR;
    temp |= FLASH_CTLR_BUFRST; //set BUFRST
    FLASH->CTLR = temp; //write reg
    //6) wait buffa reset
    if(!WaitFlashBusy())return false;
    //10)
    for(int j = 0;j < 4;j++){
      if(i == 0){
        //7) writedata
        *(volatile uint32_t*)(address+(i * 0x10)+(j * 4)) = message[j];
      }else{
        *(volatile uint32_t*)(address+(i * 0x10)+(j * 4)) = 0xFFFFFFFF;
      }
      //8) BUFLOAD
      temp = FLASH->CTLR;
      temp |= FLASH_CTLR_BUFLOAD; //set BUFLOAD
      FLASH->CTLR = temp; //write reg
      //9) wait BUFFLOAD
      WaitFlashBusy();
    }
    //11)
    FLASH->ADDR = address;
    //12)
    temp = FLASH->CTLR;
    temp |= FLASH_CTLR_STRT;
    FLASH->CTLR = temp;
    //13)
    if(!WaitFlashBusy())return false;
  }
  FLASH->CTLR = 0; //reset CTLR
  return true;
}


//lock flash write
void relockFlash(){
  WaitFlashBusy();
  uint32_t temp = 0x00008080;
  FLASH->CTLR = temp;
}

//read flash
uint32_t ReadFlash32(uint32_t address){
  if(FLASH_BASE <= address && address < FLASH_END){
    WaitFlashBusy();
    return (*(volatile const uint32_t*)(address));
  }
  return 0;
}

uint32_t V_a = 0;
int32_t V_b = 0;
uint32_t I_a = 0;
bool readcoeff(){
  V_a = ReadFlash32(V_A_ADD);
  V_b = (int32_t)ReadFlash32(V_B_ADD);
  I_a = ReadFlash32(I_A_ADD);
  return !(V_a == 0xFFFFFFFF || V_b == 0xFFFFFFFF || I_a == 0xFFFFFFFF || V_a == 0 || I_a == 0);
}

uint32_t triggervoltage = 0;
bool readtrigger(){
  triggervoltage = ReadFlash32(TRIGGER_ADD);
  return (MIN_TRIGGER_V <= triggervoltage && triggervoltage <= MAX_TRIGGER_V);
}

bool writeceff(){
  if(V_a == 0 && V_b == 0 && I_a == 0){ //can't read flash
    return false;
  }
  if(!(ReadFlash32(USE_FLASH_END + 0x4) == 0xFFFFFFFF)){ //check using
    return false;
  }
  if(!(ReadFlash32(USE_FLASH_ADD - 0x4) == 0xFFFFFFFF)){ //check using
    return false;
  }
  if(!FastEraseFlash(USE_FLASH_ADD)){
    return false;
  }
  uint32_t message[4] = {V_a, V_b, I_a, triggervoltage};
  if(!FastWriteFlash16(USE_FLASH_ADD,message)){
    return false;
  }
  relockFlash();
  return true;
}

void ppsmode();
void fiveVmode();
void fixmode();
void calmode();
void triggermode();
void triggersetmode();

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

  if(PD_connect()) {                     // init USB PD
    if(PD_getPPSNum() > 0){
      ppsable = true;
    }
    if(PD_getFixedNum() > 0){
      fixable = true;
    }
  }

  setmode();

  if(readtrigger() && mode != 5){
    mode = 4; //tiger mode
  }

  if(!readcoeff()){
    #ifndef DEBUG
    mode = 3;
    #endif
    #ifdef DEBUG
    V_a = (VDD * (R8 + R9) / R9) * 1000 / 4095;
    V_b = 0;
    I_a = 1000 * (VDD / (R3 * 4095));
    #endif
  }
  
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
    case 4:
      triggermode();
      while(1);
    case 5:
      triggersetmode();
      while(1);
  }
}

void ppsmode(){
  uint16_t count = 0;
  uint16_t countkeep = 0;
  int32_t mes_Voltage = 0;
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
  uint8_t dispmode = DISPVOLTAGE;
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
  seg_num[0] = 15; //P
  seg_num[1] = 15; //P
  seg_num[2] = 5; //S
  do{
    count = readbutton();
    DLY_ms(1);
    dispseg();
  }while(count < BUTTON_DOWN || BUTTON_OP < count);

  //disp maxVoltage
  PIN_output(CVCC);
  PIN_low(CVCC);
  setseg(max_Voltage,true);
  do{
    count = readbutton();
    DLY_ms(1);
    dispseg();
  }while(count < BUTTON_DOWN || BUTTON_OP < count);
  //disp minVoltage
  setseg(min_Voltage,true);
  do{
    count = readbutton();
    DLY_ms(1);
    dispseg();
  }while(count < BUTTON_DOWN || BUTTON_OP < count);
  //disp maxCurrent
  PIN_high(CVCC);
  setseg(max_Current,true);
  do{
    count = readbutton();
    DLY_ms(1);
    dispseg();
  }while(count < BUTTON_DOWN || BUTTON_OP < count);

  count = 0;
  PIN_low(CVCC);

  while(1){
    ADC_input(V_ADC);
    mes_Voltage = (ADC_read() * V_a + V_b) / 1000;
    if(mes_Voltage < 0) mes_Voltage = 0;
    sum_Voltage += mes_Voltage;
    #ifndef DEBUG
    if(set_Voltage*0.9 >= mes_Voltage || mes_Voltage >= set_Voltage*1.1){ //stop over voltage
      if(output){
        output = false;
        PIN_low(ONOFF);
        outvolt = true;
      }
    }
    #endif
    DLY_ms(1);
    ADC_input(I_ADC);
    mes_Current = (uint32_t)ADC_read() * I_a / 1000;
    sum_Current += mes_Current;

    dispseg();
    if(dispmode == DISPWATT){
      if(output) PIN_toggle(CVCC);
      else{
        dispmode = DISPVOLTAGE;
        PIN_low(CVCC);
      }
    }
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
        switch(dispmode){
          case DISPVOLTAGE:
            setseg(Voltage, false);
            break;
          case DISPCURRENT:
            setseg(Current, false);
            break;
          case DISPWATT:
            setseg(Voltage * Current / 1000, false);
            break;
        }
      }else{
        switch(dispmode){
          case DISPWATT:
            dispmode = DISPVOLTAGE;
            PIN_low(CVCC);
          case DISPVOLTAGE:
            setseg(set_Voltage ,true);
            break;
          case DISPCURRENT:
            setseg(set_Current ,true);
            break;
        }
      }
      if(countkeep == KEEPTIME){
        PD_setVoltage(set_Voltage+20);
      }else if(countkeep >= (2 * KEEPTIME)){
        PD_setVoltage(set_Voltage);
        countkeep = 0;
      }
    }

    switch(readbutton()){
      case 0: //not pushed
        countflag = false;
        break;
      case 255: //under processing
        break;
      case BUTTON_DOWN:
        if(dispmode == DISPVOLTAGE && set_Voltage > min_Voltage){
          set_Voltage -= 100;
          if(!PD_setVoltage(set_Voltage)){
            outvolt = true;
          }else{
            outvolt =false;
          }
        }
        break;
      case BUTTON_DOWN*10: //long pressing
        if(dispmode == DISPVOLTAGE && countflag && set_Voltage > min_Voltage){
          set_Voltage -= 100;
          countflag = false;
        }
        break;
      case BUTTON_DOWN*10+BUTTON_DOWN: //down button relased
        if(min_Voltage < set_Voltage) set_Voltage += 100;
        if(!PD_setVoltage(set_Voltage)){
          outvolt = true;
        }else{
          outvolt =false;
        }
        break;
      case BUTTON_UP:
        if(dispmode == DISPVOLTAGE && set_Voltage < max_Voltage){
          set_Voltage += 100;
          if(!PD_setVoltage(set_Voltage)){
            outvolt = true;
          }else{
            outvolt =false;
          }
        }
        break;
      case BUTTON_UP*10: //long pressing
        if(dispmode == DISPVOLTAGE && countflag && set_Voltage < max_Voltage){
          set_Voltage += 100;
          countflag = false;
        }
        break;
      case BUTTON_UP*10+BUTTON_UP: //up button relased
        if(set_Voltage < max_Voltage) set_Voltage -= 100;
        if(!PD_setVoltage(set_Voltage)){
            outvolt = true;
          }else{
            outvolt =false;
          }
        break;
      case BUTTON_CVCC:
        switch(dispmode){
          case DISPVOLTAGE:
            dispmode = DISPCURRENT;
            PIN_high(CVCC);
            break;
          case DISPWATT:
          case DISPCURRENT:
            dispmode = DISPVOLTAGE;
            PIN_low(CVCC);
            break;
        }
        break;
      case BUTTON_CVCC*10:
        if(output && dispmode < DISPWATT) dispmode = DISPWATT;
        break;
      case BUTTON_CVCC*10 + BUTTON_CVCC:
        break;
      case BUTTON_OP*10 + BUTTON_OP:
        break;
      case BUTTON_OP:
        #ifdef DEBUG
        output =! output;
        PIN_toggle(ONOFF);
        #endif
        #ifndef DEBUG
        if(set_Voltage*0.9 < Voltage && Voltage < set_Voltage*1.1 && !outvolt){
          output =! output;
          PIN_toggle(ONOFF);
        }else{ //Voltage Error
          outvolt = true;
        }
        #endif
        break;
      default:
        break;
    }
  }
}

void fixmode(){
  uint16_t count = 0;
  int32_t mes_Voltage = 0;
  uint16_t mes_Current = 0;
  uint32_t sum_Voltage = 0;
  uint32_t sum_Current = 0;
  uint16_t Voltage = 0;
  uint16_t Current = 0;
  uint8_t pdonum = 1;
  uint8_t dispmode = DISPVOLTAGE;
  bool output = false; //ON or OFF
  bool outvolt = false;

  //disp FIX
  seg_num[0] = 12; //F
  seg_num[1] = 13; //I
  seg_num[2] = 14; //X
  if(ppsable){
    do{
      count = readbutton();
      DLY_ms(1);
      dispseg();
    }while(count - 10 * (count / 10) == 0 || count == 255); //long press realse and short press
  }
  do{
    count = readbutton();
    DLY_ms(1);
    dispseg();
  }while(count < BUTTON_DOWN || BUTTON_OP < count);
  
  PIN_output(CVCC);
  PIN_low(CVCC);

  while(1){
    ADC_input(V_ADC);
    mes_Voltage = (ADC_read() * V_a + V_b) / 1000;
    if(mes_Voltage < 0) mes_Voltage = 0;
    sum_Voltage += mes_Voltage;
    #ifndef DEBUG
    if(PD_getPDOVoltage(pdonum)*0.9 >= mes_Voltage || mes_Voltage >= PD_getPDOVoltage(pdonum)*1.1){ //stop over voltage
      if(output){
        output = false;
        PIN_low(ONOFF);
        outvolt = true;
      }
    }
    #endif
    DLY_ms(1);
    ADC_input(I_ADC);
    mes_Current = (uint32_t)ADC_read() * I_a / 1000;
    sum_Current += mes_Current;

    dispseg();
    if(dispmode == DISPWATT){
      if(output) PIN_toggle(CVCC);
      else{
        dispmode = DISPVOLTAGE;
        PIN_low(CVCC);
      }
    }
    count ++;
    if(count > MAXCOUNT){
      count = 0;
      
      Voltage = sum_Voltage / MAXCOUNT;
      Current = sum_Current / MAXCOUNT;
      sum_Voltage = 0;
      sum_Current = 0;
      if(output){
        switch(dispmode){
          case DISPVOLTAGE:
            setseg(Voltage, false);
            break;
          case DISPCURRENT:
            setseg(Current, false);
            break;
          case DISPWATT:
            setseg(Voltage * Current / 1000, false);
            break;
        }
      }else{
        switch(dispmode){
          case DISPWATT:
            dispmode = DISPVOLTAGE;
            PIN_low(CVCC);
          case DISPVOLTAGE:
            setseg(PD_getPDOVoltage(pdonum) ,true);
            break;
          case DISPCURRENT:
            setseg(PD_getPDOMaxCurrent(pdonum) ,true);
            break;
        }
      }
      if(!outvolt){
        //PD_setPDO_noretrun(pdonum, PD_getPDOVoltage(pdonum)); //set voltage with no check
      }
    }

    switch(readbutton()){
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
        switch(dispmode){
          case DISPVOLTAGE:
            dispmode = DISPCURRENT;
            PIN_high(CVCC);
            break;
          case DISPWATT:
          case DISPCURRENT:
            dispmode = DISPVOLTAGE;
            PIN_low(CVCC);
            break;
        }
        break;
      case BUTTON_CVCC*10:
        if(output && dispmode < DISPWATT) dispmode = DISPWATT;
        break;
      case BUTTON_OP:
        #ifdef DEBUG
        output =! output;
        PIN_toggle(ONOFF);
        #endif
        #ifndef DEBUG
        if(PD_getPDOVoltage(pdonum)*0.9 < Voltage && Voltage < PD_getPDOVoltage(pdonum)*1.1 && !outvolt){
          output =! output;
          PIN_toggle(ONOFF);
        }else{ //Voltage Error
          outvolt = true;
        }
        #endif
        break;
      default:
        break;
    }
  }
}

void fiveVmode(){
  uint16_t count = 0;
  int32_t mes_Voltage = 0;
  uint16_t mes_Current = 0;
  uint32_t sum_Voltage = 0;
  uint32_t sum_Current = 0;
  uint16_t set_Voltage = 5000;
  uint16_t Voltage = 0;
  uint16_t Current = 0;
  uint8_t dispmode = DISPVOLTAGE;
  bool output = false; //OFF
  bool outvolt = false;
  
  PIN_output(CVCC);
  PIN_low(CVCC);

  seg_num[0] = 10; //" "
  seg_num[1] = 5; //5
  seg_num[2] = 10; //" "
  seg_digit = 0;

  count = 0;

  while(1){
    ADC_input(V_ADC);
    mes_Voltage = (ADC_read() * V_a + V_b) / 1000;
    if(mes_Voltage < 0) mes_Voltage = 0;
    sum_Voltage += mes_Voltage;
    #ifndef DEBUG
    if(set_Voltage*0.9 >= mes_Voltage || mes_Voltage >= set_Voltage*1.1){ //stop over voltage
      if(output){
        output = false;
        PIN_low(ONOFF);
        outvolt = true;
      }
    }
    #endif
    DLY_ms(1);
    ADC_input(I_ADC);
    mes_Current = (uint32_t)ADC_read() * I_a / 1000;
    sum_Current += mes_Current;

    dispseg();
    if(dispmode == DISPWATT){
      if(output) PIN_toggle(CVCC);
      else{
        dispmode = DISPVOLTAGE;
        PIN_low(CVCC);
      }
    }
    count ++;
    if(count > MAXCOUNT){
      count = 0;
      
      Voltage = sum_Voltage / MAXCOUNT;
      Current = sum_Current / MAXCOUNT;
      sum_Voltage = 0;
      sum_Current = 0;
      if(output){
        switch(dispmode){
          case DISPVOLTAGE:
            setseg(Voltage, false);
            break;
          case DISPCURRENT:
            setseg(Current, false);
            break;
          case DISPWATT:
            setseg(Voltage * Current / 1000, false);
            break;
        }
      }else{
        seg_num[0] = 10; //" "
        seg_num[1] = 5; //5
        seg_num[2] = 10; //" "
        seg_digit = 0;
      }
    }

    switch(readbutton()){
      case BUTTON_OP:
        #ifdef DEBUG
        output =! output;
        PIN_toggle(ONOFF);
        #endif
        #ifndef DEBUG
        if(set_Voltage*0.9 < Voltage && Voltage < set_Voltage*1.1 && !outvolt){
          output =! output;
          PIN_toggle(ONOFF);
        }else{ //Voltage Error
          outvolt = true;
        }
        #endif
        break;
      case BUTTON_CVCC:
        switch(dispmode){
          case DISPVOLTAGE:
            dispmode = DISPCURRENT;
            PIN_high(CVCC);
            break;
          case DISPWATT:
          case DISPCURRENT:
            dispmode = DISPVOLTAGE;
            PIN_low(CVCC);
            break;
        }
        break;
      case BUTTON_CVCC*10:
        if(dispmode < DISPWATT) dispmode = DISPWATT;
        break;
    }
  }
}

void calmode(){
  uint16_t count = 0;
  uint32_t sum = 0;
  uint32_t aveV1 = 0;
  uint32_t aveV2 = 0;
  uint32_t aveA = 0;
  //disp 5
  seg_num[0] = 16; //C
  seg_num[1] = 17; //A
  seg_num[2] = 18; //Ls
  do{
    count = readbutton();
    DLY_ms(1);
    dispseg();
  }while(count < BUTTON_DOWN || BUTTON_OP < count);

  //calivration 5.00V
  setseg(CALV1,false);
  PIN_output(CVCC);
  PIN_low(CVCC);
  do{
    count = readbutton();
    DLY_ms(1);
    dispseg();
  }while(count < BUTTON_DOWN || BUTTON_OP < count);
  sum = 0;
  for(int i = 0; i < MAXCOUNT; i++){
    ADC_input(V_ADC);
    DLY_ms(1);
    sum += ADC_read();
  }
  aveV1 = (100 * sum) / MAXCOUNT;

  //calivration 18.00V
  setseg(CALV2,false);
  PIN_low(CVCC);
  do{
    count = readbutton();
    DLY_ms(1);
    dispseg();
  }while(count < BUTTON_DOWN || BUTTON_OP < count);
  sum = 0;
  for(int i = 0; i < MAXCOUNT; i++){
    ADC_input(V_ADC);
    DLY_ms(1);
    sum += ADC_read();
  }
  aveV2 = (100 * sum) / MAXCOUNT;

  //calivration 3.00A
  setseg(CALA,false);
  PIN_high(CVCC);
  PIN_high(ONOFF);
  do{
    count = readbutton();
    DLY_ms(1);
    dispseg();
  }while(count < BUTTON_DOWN || BUTTON_OP < count);
  sum = 0;
  for(int i = 0; i < MAXCOUNT; i++){
    ADC_input(I_ADC);
    DLY_ms(1);
    sum += ADC_read();
  }
  aveA = (100 * sum) / MAXCOUNT;
  PIN_low(ONOFF);

  V_a = 1000 * 100 * (CALV2 - CALV1)/(aveV2 - aveV1);
  V_b = 1000 * CALV2 - ((V_a * aveV2) / 100);
  I_a = 1000 * 100 * CALA / aveA;

  //disp result
  setseg(V_a,false);
  PIN_low(CVCC);
  do{
    count = readbutton();
    DLY_ms(1);
    dispseg();
  }while(count < BUTTON_DOWN || BUTTON_OP < count);
  
  setseg(I_a,false);
  PIN_high(CVCC);
  do{
    count = readbutton();
    DLY_ms(1);
    dispseg();
  }while(count < BUTTON_DOWN || BUTTON_OP < count);

  if(!writeceff()){
    //error
    seg_num[0] = 10; //" "
    seg_num[1] = 19; //E
    seg_num[2] = 10; //" "
    seg_digit = 0;
    while(1){
      dispseg();
      DLY_ms(1);
    }
  }else{
    //sucess
    RST_now();
  }
}

void triggermode(){
  uint16_t count = 0;
  uint16_t countkeep = 0;
  uint8_t pdonum = 0;
  int32_t mes_Voltage = 0;
  uint16_t mes_Current = 0;
  uint32_t sum_Voltage = 0;
  uint32_t sum_Current = 0;
  uint16_t Voltage = 0;
  uint16_t Current = 0;
  uint8_t dispmode = DISPVOLTAGE;
  bool outvolt = false;

  //select pdo
  for(uint8_t i = 1; i <= PD_getPDONum(); i++) {
    if(i <= PD_getFixedNum()){ //fix
      if(PD_getPDOVoltage(i) == triggervoltage){
        pdonum = i;
        mode = 1;
      }
    }
    else if(PD_getPDOMinVoltage(i) <= triggervoltage && triggervoltage <= PD_getPDOMaxVoltage(i)){ //pps
      if(PD_getPDOMaxCurrent(pdonum) < PD_getPDOMaxCurrent(i)){ //select higher current pdo
        pdonum = i;
        mode = 2;
      }
    }
  }

  if(triggervoltage == 5000) mode = 0;

  //Cannot supply triggervoltage
  if(mode == 4){
    while(1){
      dispseg();
      DLY_ms(1);
    }
  }

  if(mode && !PD_setPDO(pdonum, triggervoltage)){
    while(1){
      dispseg();
      DLY_ms(1);
    }
  }

  PIN_output(CVCC);
  PIN_low(CVCC);

  #ifdef DEBUG
  PIN_high(ONOFF);
  #endif

  while(1){
    ADC_input(V_ADC);
    mes_Voltage = (ADC_read() * V_a + V_b) / 1000;
    if(mes_Voltage < 0) mes_Voltage = 0;
    sum_Voltage += mes_Voltage;
    #ifndef DEBUG
    if(triggervoltage*0.9 >= mes_Voltage || mes_Voltage >= triggervoltage*1.1){ //stop over voltage
      PIN_low(ONOFF);
      outvolt = true;
    }else if(!outvolt){
      PIN_high(ONOFF);
    }
    #endif
    DLY_ms(1);
    ADC_input(I_ADC);
    mes_Current = (uint32_t)ADC_read() * I_a / 1000;
    sum_Current += mes_Current;

    dispseg();
    if(dispmode == DISPWATT){
      PIN_toggle(CVCC);
    }
    count ++;
    if(count > MAXCOUNT){
      count = 0;
      countkeep ++;
      
      Voltage = sum_Voltage / MAXCOUNT;
      Current = sum_Current / MAXCOUNT;
      sum_Voltage = 0;
      sum_Current = 0;
      switch(dispmode){
        case DISPVOLTAGE:
          setseg(Voltage, false);
          break;
        case DISPCURRENT:
          setseg(Current, false);
          break;
        case DISPWATT:
          setseg(Voltage * Current / 1000, false);
          break;
      }
      if(mode == 2){
        if(countkeep == KEEPTIME){
          PD_setPDO(pdonum, triggervoltage+20);
        }else if(countkeep >= (2 * KEEPTIME)){
          PD_setPDO(pdonum, triggervoltage);
          countkeep = 0;
        }
      }
    }
    switch(readbutton()){
      case 0: //not pushed
        break;
      case 255: //under processing
        break;
      case BUTTON_CVCC:
        switch(dispmode){
          case DISPVOLTAGE:
            dispmode = DISPCURRENT;
            PIN_high(CVCC);
            break;
          case DISPWATT:
          case DISPCURRENT:
            dispmode = DISPVOLTAGE;
            PIN_low(CVCC);
            break;
        }
        break;
      case BUTTON_CVCC*10:
        if(dispmode < DISPWATT) dispmode = DISPWATT;
        break;
      default:
        break;
    }
  } 
}

void triggersetmode(){
  uint16_t count = 0;
  bool countflag = false;
  //disp FIX
  seg_num[0] = 20; //T
  seg_num[1] = 22; //R
  seg_num[2] = 23; //G
  do{
    count = readbutton();
    DLY_ms(1);
    dispseg();
  }while(count - 10 * (count / 10) == 0 || count == 255); //long press realse or short press
  do{
    count = readbutton();
    DLY_ms(1);
    dispseg();
  }while(count < BUTTON_DOWN || BUTTON_OP < count);

  //when triggervoltage is valid value, reset trigger mode.
  if(MIN_TRIGGER_V <= triggervoltage && triggervoltage <= MAX_TRIGGER_V){
    seg_num[0] = 24; //D
    seg_num[1] = 19; //E
    seg_num[2] = 18; //L
    do{
      count = readbutton();
      DLY_ms(1);
      dispseg();
    }while(count < BUTTON_DOWN || BUTTON_OP < count);
    triggervoltage = 0;
    if(!writeceff()){
      seg_num[0] = 10; //" "
      seg_num[1] = 19; //E
      seg_num[2] = 10; //" "
      seg_digit = 0;
      while(1){
        dispseg();
        DLY_ms(1);
      }
    }else{
      //sucess
      RST_now();
    }
  }
  
  seg_num[0] = 5; //S
  seg_num[1] = 19; //E
  seg_num[2] = 20; //T
  do{
    count = readbutton();
    DLY_ms(1);
    dispseg();
  }while(count < BUTTON_DOWN || BUTTON_OP < count);

  count = 0;
  triggervoltage = 5000; //5V

  PIN_output(CVCC);
  PIN_low(CVCC);

  while(1){
    DLY_ms(1);
    dispseg();
    count ++;
    if(count > MAXCOUNT){
      count = 0;
      countflag = true;
      setseg(triggervoltage, true);
    }

    switch(readbutton()){
      case 0: //not pushed
        countflag = false;
        break;
      case 255: //under processing
        break;
      case BUTTON_DOWN:
        if(triggervoltage > MIN_TRIGGER_V){
          triggervoltage -= 100;
        }
        break;
      case BUTTON_DOWN*10: //long pressing
        if(countflag && triggervoltage > MIN_TRIGGER_V){
          triggervoltage -= 100;
          countflag = false;
        }
        break;
      case BUTTON_DOWN*10+BUTTON_DOWN: //down button relased
        if(MIN_TRIGGER_V < triggervoltage) triggervoltage += 100;
        break;
      case BUTTON_UP:
        if(triggervoltage < MAX_TRIGGER_V){
          triggervoltage += 100;
        }
        break;
      case BUTTON_UP*10: //long pressing
        if(countflag && triggervoltage < MAX_TRIGGER_V){
          triggervoltage += 100;
          countflag = false;
        }
        break;
      case BUTTON_UP*10+BUTTON_UP: //up button relased
        if(triggervoltage < MAX_TRIGGER_V) triggervoltage -= 100;
        break;
      case BUTTON_CVCC:
        break;
      case BUTTON_CVCC*10:
        break;
      case BUTTON_CVCC*10 + BUTTON_CVCC:
        break;
      case BUTTON_OP*10 + BUTTON_OP:
        break;
      case BUTTON_OP:
        if(MIN_TRIGGER_V <= triggervoltage && triggervoltage <= MAX_TRIGGER_V){
          if(!writeceff()){
            seg_num[0] = 10; //" "
            seg_num[1] = 19; //E
            seg_num[2] = 10; //" "
            seg_digit = 0;
            while(1){
              dispseg();
              DLY_ms(1);
            }
          }else{
            //sucess
            RST_now();
          }
        }
        break;
      default:
        break;
    }
  }

}