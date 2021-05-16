#ifndef COLORS_H_
#define COLORS_H_
#include <stdint.h>
//targetable colors
enum color {
	RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, BLACK, WHITE
};

uint8_t target_color(void);
void toggle_color_leds(void);

#endif /* COLORS_H_ */
