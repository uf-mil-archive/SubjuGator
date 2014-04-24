/* Linker script for STM32F103TB */

/* Define memory regions. */
MEMORY
{
	boot_rom (rx) : ORIGIN = 0x08000000, LENGTH = 64K
	rom (rx) : ORIGIN = 0x08010000, LENGTH = 64K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

/* Include the common ld script. */
INCLUDE libopencm3_stm32f1.ld
