/* Loaded and entered at 0x40010000, like any RCM payload. */
ENTRY(_start)
SECTIONS {
    . = 0x40010000;
    .text : { KEEP(*(.text.start)) *(.text*) }
    .rodata : { *(.rodata*) }
    .data : { *(.data*) }
    .bss : { *(.bss*) *(COMMON) }
    /DISCARD/ : { *(.comment) *(.note*) *(.ARM.attributes) *(.ARM.exidx*) }
}
