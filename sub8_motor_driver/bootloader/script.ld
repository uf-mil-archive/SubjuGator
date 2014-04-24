/* Linker script for STM32F103TB */

/* Define memory regions. */
MEMORY
{
	rom (rx) : ORIGIN = 0x08000000, LENGTH = 64K
	client_rom (rx) : ORIGIN = 0x08010000, LENGTH = 64K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

PROVIDE(_begin_client_rom = ORIGIN(client_rom));
PROVIDE(_end_client_rom = ORIGIN(client_rom) + LENGTH(client_rom));

/* Include the common ld script. */
INCLUDE libopencm3_stm32f1.ld
