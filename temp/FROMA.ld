ENTRY(Vector_Init);

MEMORY
{
	TEXT(rwx)	:	ORIGIN = 0x80000000, LENGTH = 256M
	DATA(rwx)	:	ORIGIN = 0x90000000, LENGTH = 256M
}

SECTIONS
{
    .text :
    {
        _text = .;
        __isr_vector_start = .;
        *(.isr_vector)
        __isr_vector_end = .;
        *(.text*)
        *(.rodata*)
        _etext = .;
    } > TEXT

    .data :
    {
        _data = .;
        *(.data*)
        _edata = .;
    } > DATA

    .bss :
    {
			. = ALIGN(4);
        _bss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > DATA

    .stack :
    {
			. = ALIGN(8);
			*(.__stack0);
			. = ALIGN(8);
			*(.__stack1);
		}
}

