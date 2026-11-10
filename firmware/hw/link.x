/* O3C images: code from ORIGIN(FLASH), data copied to RAM, stack at the top of RAM.
   memory.x (per image) defines FLASH and RAM. */
INCLUDE memory.x
ENTRY(_start)

SECTIONS
{
    .text ORIGIN(FLASH) :
    {
        KEEP(*(.init));
        . = ALIGN(4);
        *(.trap);
        *(.text .text.*);
    } > FLASH

    .rodata : ALIGN(4)
    {
        *(.srodata .srodata.*);
        *(.rodata .rodata.*);
        . = ALIGN(4);
    } > FLASH

    .data : ALIGN(4)
    {
        _sdata = .;
        PROVIDE(__global_pointer$ = . + 0x800);
        *(.sdata .sdata.* .sdata2 .sdata2.*);
        *(.data .data.*);
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH
    _sidata = LOADADDR(.data);

    .bss (NOLOAD) : ALIGN(4)
    {
        _sbss = .;
        *(.sbss .sbss.* .bss .bss.*);
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    _stack_top = ORIGIN(RAM) + LENGTH(RAM);

    /DISCARD/ : { *(.eh_frame .eh_frame_hdr) }
}
