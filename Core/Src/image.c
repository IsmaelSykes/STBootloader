/*
 * image.c
 *
 *  Created on: 10 may 2023
 *      Author: Cantilever
 */


#include <stdio.h>
#include "image.h"
#include "memoryMap.h"
#include "crc32.h"

const image_hdr_t *image_get_header(image_slot_t slot)
{
    const image_hdr_t *hdr = NULL;

    switch (slot)
    {
    case IMAGE_SLOT_1:
        hdr = (const image_hdr_t *) &__app_rom_start__;
        break;
    case IMAGE_SLOT_2:
        hdr = (const image_hdr_t *) &__loader_rom_start__;
        break;
    default:
        break;
    }

    if (hdr && hdr->image_magic == IMAGE_MAGIC)
    {
        return hdr;
    }
    else
    {
        return NULL;
    }
}

int image_validate(image_slot_t slot, const image_hdr_t *hdr)
{
    // void *addr = (slot == IMAGE_SLOT_1 ? &__apparom_start__ : &__appbrom_start__);
    void *addr = NULL;

    switch (slot)
    {
    case IMAGE_SLOT_1:
        addr = &__app_rom_start__;
        break;
    case IMAGE_SLOT_2:
        addr = &__loader_rom_start__;
        break;
    default:
        addr = NULL;
        return -1;
    }

    addr += sizeof(image_hdr_t);
    uint32_t len = hdr->data_size;
    uint32_t crc_calc = crc32(addr, len);
    uint32_t crc_image = hdr->crc;

    if (crc_calc == crc_image) {
        printf("CRC OK: %lx vs %lx\r\n", crc_image, crc_calc);
        return 0;
    }
    else {
        printf("CRC mismatch: %lx vs %lx\r\n", crc_image, crc_calc);
        return -1;
    }

}

