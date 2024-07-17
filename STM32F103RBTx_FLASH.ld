
MEMORY
{
    rom (rx) : ORIGIN = 0x08000000, LENGTH = 64K
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}
PROVIDE(_stack = ORIGIN(ram) + LENGTH(ram));

EXTERN(vector_table);
ENTRY(reset_handler);

SECTIONS
{
    .text : {
        *(.vectors)
        *(.text*)
        . = ALIGN(4);
    } >rom
	.data : {
        . = ALIGN(4);
        _sdata = .;
		_la_data = LOADADDR(.data); /* Load address of .data in FLASH */
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } >ram AT>rom
    .bss : {
        . = ALIGN(4);
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } >ram
	.stack (NOLOAD) :
    {
        . = ALIGN(8);
        _estack = .;
        . += 4K;  /* Allocate 4K for stack */
    } >ram
}
