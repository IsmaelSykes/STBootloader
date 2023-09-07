/*
 * my_memory.h
 *
 *  Created on: 10 may 2023
 *      Author: Cantilever
 */

#ifndef INC_MY_MEMORY_H_
#define INC_MY_MEMORY_H_

#include <stdint.h>
#include <stdbool.h>


void shared_memory_init(void);

bool shared_mem_is_app_upd_requested(void);
bool shared_mem_is_bl_upd_requested(void);
bool shared_mem_is_bg_fault_set(void);

void shared_mem_set_app_update_requested(bool value);
void shared_mem_set_bl_update_requested(bool value);
void shared_mem_set_bg_fault(bool value);

void shared_mem_increment_boot_counter(void);
void shared_mem_clear_boot_counter(void);
uint8_t shared_mem_get_boot_counter(void);
uint32_t shared_mem_get_update_size(void);
void shared_mem_set_update_completed(void);
void shared_mem_clear_ota_info(void);
char *shared_mem_get_fota_status(void);
void shared_mem_clear_fota_status(void);
void shared_mem_set_update_size(uint32_t size);
void shared_mem_increment_counter(void);
int shared_mem_get_counter(void);

void shared_mem_set_update(void);

void shared_mem_clear_update(void);

int shared_mem_get_update(void);

#endif /* INC_MY_MEMORY_H_ */
