/*
 * my_memory.c
 *
 *  Created on: 10 may 2023
 *      Author: Cantilever
 */
#include <stdio.h>
#include <string.h>
#include "my_memory.h"

const uint32_t MAGIC = 0x1ce;

typedef struct __attribute__((packed)) {
  uint32_t  flags;
  uint32_t  magic;
  uint32_t  update_size;
  char      fota_status[4];
  uint8_t   boot_count;
  uint8_t   update;
}shared_data_t;

shared_data_t shared_data __attribute__((section(".shared_mem")));


/* FLAGS */
enum {
  APP_UPDATE_REQUEST = 1 << 0,
  BL_UPDATE_REQUEST = 1 << 1,

  FLAG_BG_FAULT = 1 << 2,
};

static void prv_set_flag(uint32_t flag, bool value) {
    if (value) {
        shared_data.flags |= flag;
    } else {
        shared_data.flags &= ~flag;
    }
}

static bool prv_get_flag(uint32_t flag) {
    return shared_data.flags & flag;
}

void shared_memory_init(void)
{
  if (shared_data.magic != MAGIC)
  {
    printf("Shared memory uninitialized, setting MAGIC\r\n");
    memset(&shared_data, 0, sizeof (shared_data_t));
    shared_data.magic = MAGIC;
  }
}

bool shared_mem_is_app_upd_requested(void)
{
  return prv_get_flag(APP_UPDATE_REQUEST);
}

bool shared_mem_is_bl_upd_requested(void)
{
  return prv_get_flag(BL_UPDATE_REQUEST);
}

bool shared_mem_is_bg_fault_set(void)
{
  return prv_get_flag(FLAG_BG_FAULT);
}

void shared_mem_increment_boot_counter(void)
{
  shared_data.boot_count++;
}

void shared_mem_clear_boot_counter(void)
{
  shared_data.boot_count = 0;
}

uint8_t shared_mem_get_boot_counter(void)
{
  return shared_data.boot_count;
}

uint32_t shared_mem_get_update_size(void)
{
  return shared_data.update_size;
}

void shared_mem_set_update_completed(void)
{
  memcpy(shared_data.fota_status, "DONE", 4);
}

void shared_mem_clear_ota_info(void)
{
  shared_data.update_size = 0;

}

void shared_mem_set_app_update_requested(bool value)
{
  prv_set_flag(APP_UPDATE_REQUEST, value);
}

void shared_mem_set_bl_update_requested(bool value)
{
  prv_set_flag(BL_UPDATE_REQUEST, value);
}

void shared_mem_set_bg_fault(bool value)
{
  prv_set_flag(FLAG_BG_FAULT, value);
}

char *shared_mem_get_fota_status(void)
{
  return shared_data.fota_status;
}

void shared_mem_clear_fota_status(void)
{
  memset(shared_data.fota_status, '\0', sizeof(shared_data.fota_status));
}

void shared_mem_set_update_size(uint32_t size)
{
  shared_data.update_size = size;
}


void shared_mem_set_update(void)
{
	shared_data.update = 1;
}

void shared_mem_clear_update(void)
{
	shared_data.update = 0;
}

int shared_mem_get_update(void)
{
	return shared_data.update;
}


