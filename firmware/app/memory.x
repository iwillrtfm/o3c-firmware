/* App: flash from 0x8000 (after the bootloader) up to the header page at 0x2FF00.
   RAM as the bootloader: the handoff block at 0x20000000 stays out. */
MEMORY
{
    FLASH : ORIGIN = 0x00008000, LENGTH = 0x2FF00 - 0x8000
    RAM : ORIGIN = 0x20000100, LENGTH = 64K - 256
}
