#ifndef COLORS_H_
#define COLORS_H_
#include <stdint.h>
//targetable colors
enum color {
	RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, BLACK, WHITE
};

//chooses targeted color from the selector
uint8_t target_color(void);
//lights leds up according to target_color
void toggle_color_leds(void);

#endif /* COLORS_H_ */
