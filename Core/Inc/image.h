/*
 * image.h
 *
 *  Created on: May 8, 2023
 *      Author: Cantilever
 */

#ifndef INC_IMAGE_H_
#define INC_IMAGE_H_

#define IMAGE_MAGIC	0x1ce

typedef  struct  __attribute__((packed))  {
uint16_t image_magic;
uint16_t image_hdr_version;
uint32_t crc;
uint32_t data_size;
uint8_t image_type;
uint8_t version_major;
uint8_t version_minor;
uint8_t version_patch;
uint32_t vector_addr;
uint32_t reserved;
char git_sha[8];
}  image_hdr_t;


typedef enum {
    IMAGE_TYPE_APP = 0x2,
    IMAGE_TYPE_UPDATER = 0x3,
} image_type_t;

typedef enum {
    IMAGE_SLOT_1 = 1,
    IMAGE_SLOT_2 = 2,
    IMAGE_SLOT_2_LEGACY = 3,
    IMAGE_NUM_SLOTS,
} image_slot_t;

typedef enum {
    IMAGE_VERSION_1 = 1,
    IMAGE_VERSION_CURRENT = IMAGE_VERSION_1,
} image_version_t;


const image_hdr_t *image_get_header(image_slot_t slot);
int image_validate(image_slot_t slot, const image_hdr_t *hdr);

#endif /* INC_IMAGE_H_ */
