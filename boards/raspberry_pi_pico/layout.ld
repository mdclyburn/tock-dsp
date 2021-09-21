MEMORY
{
  /* uncomment this to boot from RAM */
  /* reset (rx)  : ORIGIN = 0x20000000, LENGTH = 16K
  rom (rx)  : ORIGIN = 0x20000100, LENGTH = 128K
  prog (rx) : ORIGIN = 0x20040000, LENGTH = 1K
  ram (rwx) : ORIGIN = 0x20004000, LENGTH = 240K */

  /* boot from Flash */
  rom (rx)  : ORIGIN = 0x10000000, LENGTH = 128K
  prog (rx) : ORIGIN = 0x10020000, LENGTH = 256K
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 260K
  core1_ram (rwx) : ORIGIN = 0x20041000, LENGTH = 4K
}

MPU_MIN_ALIGN = 8K;
PAGE_SIZE = 4K;

ENTRY(jump_to_bootloader)

/* RP2040 has a bootloader that occupies the first 256 bytes of .text */
SECTIONS {
  .text  : ALIGN(4)
    {
        KEEP (*(.flash_bootloader));
        . = ALIGN (256);
    } > rom
}

INCLUDE ../kernel_layout.ld

SECTIONS {
  .multicore (NOLOAD) :
  {
    . = ALIGN(4);
    KEEP(*(.hardware_sync));
    . = ALIGN(4);
  } >ram

  .core1_support (NOLOAD) :
  {
    . = ALIGN(8);
    _core1_sstack = .;
    KEEP(*(.core1_stack_buffer));
    . = ALIGN(8);
    _core1_estack = .;

    . = ALIGN (256);

    KEEP(*(.core1_vectors));
    KEEP(*(.core1_irqs));
  } >core1_ram
}
