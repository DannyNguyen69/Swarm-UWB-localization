/* nRF52832_xxAA without SoftDevice: whole flash and RAM belong to the application. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x80000
  RAM (rwx) :  ORIGIN = 0x20000000, LENGTH = 0x10000
}

INCLUDE "nrf52_common.ld"
