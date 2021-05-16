#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#define RGB565_RED_BITMASK 		0xF8
#define RGB565_GREEN_BITMASK_1  0x07
#define RGB565_GREEN_BITMASK_2  0xE0
#define RGB565_BLUE_BITMASK 	0x1F

uint16_t get_line_position(void);
void process_image_start(void);
uint16_t target_color(void);

#endif /* PROCESS_IMAGE_H */
