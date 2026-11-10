/* Bootloader: the first 32 KiB of flash. RAM skips the 256-byte handoff block at 0x20000000
   and stays inside 64 KiB, which every RAM/flash split of the CH32V307 has. */
MEMORY
{
    FLASH : ORIGIN = 0x00000000, LENGTH = 32K
    RAM : ORIGIN = 0x20000100, LENGTH = 64K - 256
}
