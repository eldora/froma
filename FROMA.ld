ENTRY(Vector_Init);

MEMORY
{
	ram (rwx) : ORIGIN = 0x80000000, LENGTH = 768M
}

PROVIDE(__stack0 = 0xa0000000);
PROVIDE(__stack1 = 0xb0000000);

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
    } > ram
    .data :
    {
        _data = .;
        *(vtable)
        *(.data*)
        _edata = .;
    } > ram
    .bss :
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > ram
}

