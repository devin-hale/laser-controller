ENTRY(reset_handler)
/* Entry point at program/board start, same for every MCU */

/* 
Define memory bounds.
ORIGIN = start of memory address
Length = Size, ex 64K == 64 Kilobytes
----
Sourced from stm32-base
*/

MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 64K
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

/* Define references to top of stack, and the interrupt vector table */
PROVIDE(SRAM_END = ORIGIN(SRAM) + LENGTH(SRAM));

SECTIONS
{
	.vector_table :
	{
		KEEP(*(.vector_table))
	} >FLASH

	/* Data that is loaded directly into flash */
    .text : 
	{
		. = ALIGN(4);

		*(.text)
		*(.rodata)
		
		. = ALIGN(4);
		_etext = .;
    } >FLASH

	/* Data that is loaded from flash -> ram on program start */
	.data : 
	{
        . = ALIGN(4);
		_la_data = LOADADDR(.data);

        _sdata = .;
        *(.data*)

        . = ALIGN(4);
        _edata = .;
    } >SRAM AT>FLASH

    .bss : {
        . = ALIGN(4);
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } >SRAM

	.stack (NOLOAD) :
    {
        . = ALIGN(8);
        _estack = .;
        . += 4K;
    } >SRAM
}
