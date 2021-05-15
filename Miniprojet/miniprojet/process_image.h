#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

enum color {
	RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, BLACK, WHITE
};

#define TARGET_COLOR RED
#define ARRAY_POS_WIDTH 0
#define ARRAY_POS_LINE 1

enum color getLineColor(uint16_t redWidth, uint16_t greenWidth,
		uint16_t blueWidth);
float get_distance_cm(void);
uint16_t get_line_position(void);
void process_image_start(void);

#endif /* PROCESS_IMAGE_H */
