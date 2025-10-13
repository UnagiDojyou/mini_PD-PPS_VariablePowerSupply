#include "flash_rom.h"

#define FLASH_END 0x0800F7FF // end of main flash memory

bool FLASH_unlock() {
  FLASH->KEYR = FLASH_KEY1;
  FLASH->KEYR = FLASH_KEY2;
  uint32_t temp = FLASH->CTLR; // read FLASH_CTLR
  return !(temp & FLASH_CTLR_LOCK);
}

bool FLASH_unlockFast() {
  FLASH->MODEKEYR = FLASH_KEY1;
  FLASH->MODEKEYR = FLASH_KEY2;
  uint32_t temp = FLASH->CTLR; // read FLASH_CTLR
  return !(temp & FLASH_CTLR_FLOCK);
}

// when EOP is 1 return true,else return false
bool FLASH_waitBusy() {
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
bool FLASH_erasePage(uint32_t address) {
  // 1) check locks
  uint32_t temp = FLASH->CTLR; // read FLASH_CTLR
  if (temp & FLASH_CTLR_LOCK) { // check lock
    if (!FLASH_unlock()) {
      return false;
    }
  }
  // 2)
  temp = FLASH->CTLR;
  if (temp & FLASH_CTLR_FLOCK) { // check flock
    if (!FLASH_unlockFast()) {
      return false;
    }
  }
  // 3) check other operation
  FLASH_waitBusy();
  // 4) set erase mode
  FLASH->CTLR = FLASH_CTLR_FTER; // set FTER bit
  // 5) set erase address
  FLASH->ADDR = address;
  // 6) start erasec
  temp = FLASH->CTLR;
  temp |= FLASH_CTLR_STRT; // set FTER STRT
  FLASH->CTLR = temp; // write reg
  // 7,8) wait erase
  if (!FLASH_waitBusy()) return false;
  // 9) reset CTRL
  FLASH->CTLR = 0;
  return true;
}

// Write Flash only 32Byte. "message" must be uint32_t[8]. "address" is Page start address.
bool FLASH_write32Byte(uint32_t address,uint32_t *message) {
  // check locks
  uint32_t temp = FLASH->CTLR; // read FLASH_CTLR
  if (temp & FLASH_CTLR_LOCK) {  // check lock
    if (!FLASH_unlock()) {
      return false;
    }
  }
  temp = FLASH->CTLR;
  if (temp & FLASH_CTLR_FLOCK) { // check flock
    if (!FLASH_unlockFast()) {
      return false;
    }
  }
  // 3) check other operation
  FLASH_waitBusy();
  // 4) set programming mode
  FLASH->CTLR = FLASH_CTLR_FTPG; // set FTPG
  // 15)
  for (int i = 0;i < 16;i++) {
    //5)
    temp = FLASH->CTLR;
    temp |= FLASH_CTLR_BUFRST; // set BUFRST
    FLASH->CTLR = temp; // write reg
    // 6) wait buffa reset
    if (!FLASH_waitBusy()) return false;
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
      FLASH_waitBusy();
    }
    // 11)
    FLASH->ADDR = address;
    // 12)
    temp = FLASH->CTLR;
    temp |= FLASH_CTLR_STRT;
    FLASH->CTLR = temp;
    // 13)
    if (!FLASH_waitBusy()) return false;
  }
  FLASH->CTLR = 0; // reset CTLR
  return true;
}


// lock flash write
void FLASH_relock() {
  FLASH_waitBusy();
  uint32_t temp = 0x00008080;
  FLASH->CTLR = temp;
}

// read flash
uint32_t FLASH_read4Byte(uint32_t address) {
  if (FLASH_BASE <= address && address < FLASH_END) {
    FLASH_waitBusy();
    return (*(volatile const uint32_t*)(address));
  }
  return 0;
}


