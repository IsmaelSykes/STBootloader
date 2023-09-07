/*
 * memoryMap.h
 *
 * Created: 25/11/2021 06:01:56 p. m.
 *  Author: REDia
 */


#ifndef MEMORYMAP_H_
#define MEMORYMAP_H_

/* Access to variables from linker script */
extern int __boot_rom_start__;
extern int __boot_rom_size__;

extern int __app_rom_start__;
extern int __app_rom_size__;

extern int __loader_rom_start__;
extern int __loader_rom_size__;

extern int __isr_vector_start__;
extern int __shared_data_start__;


extern int __legacybrom_start__;
extern int __legacybrom_size__;

#define PARTITION_SIZE              __appbrom_size__

// #define START_FLASH_ADDR            0x00400000UL

// #define USER_APP_BASE_ADDRESS_A     0x00000000UL

//#define USER_APP_BASE_ADDRESS_B     0x00407200UL



#endif /* MEMORYMAP_H_ */

