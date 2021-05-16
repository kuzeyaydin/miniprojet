#include <selector.h>
#include <colors.h>
#include <leds.h>

uint8_t target_color(void) {
	if(get_selector()<BLACK) return get_selector();
	else return BLACK;
}

void toggle_color_leds(void) {
	switch(target_color()) {
	case RED:
		set_rgb_led(LED2, RGB_MAX_INTENSITY, 0, 0);
		set_rgb_led(LED4, RGB_MAX_INTENSITY, 0, 0);
		set_rgb_led(LED6, RGB_MAX_INTENSITY, 0, 0);
		set_rgb_led(LED8, RGB_MAX_INTENSITY, 0, 0);
		break;
	case GREEN:
		set_rgb_led(LED2, 0, RGB_MAX_INTENSITY, 0);
		set_rgb_led(LED4, 0, RGB_MAX_INTENSITY, 0);
		set_rgb_led(LED6, 0, RGB_MAX_INTENSITY, 0);
		set_rgb_led(LED8, 0, RGB_MAX_INTENSITY, 0);
		break;
	case BLUE:
		set_rgb_led(LED2, 0, 0, RGB_MAX_INTENSITY);
		set_rgb_led(LED4, 0, 0, RGB_MAX_INTENSITY);
		set_rgb_led(LED6, 0, 0, RGB_MAX_INTENSITY);
		set_rgb_led(LED8, 0, 0, RGB_MAX_INTENSITY);
		break;
	case YELLOW:
		set_rgb_led(LED2, RGB_MAX_INTENSITY, RGB_MAX_INTENSITY, 0);
		set_rgb_led(LED4, RGB_MAX_INTENSITY, RGB_MAX_INTENSITY, 0);
		set_rgb_led(LED6, RGB_MAX_INTENSITY, RGB_MAX_INTENSITY, 0);
		set_rgb_led(LED8, RGB_MAX_INTENSITY, RGB_MAX_INTENSITY, 0);
		break;
	case MAGENTA:
		set_rgb_led(LED2, RGB_MAX_INTENSITY, 0, RGB_MAX_INTENSITY);
		set_rgb_led(LED4, RGB_MAX_INTENSITY, 0, RGB_MAX_INTENSITY);
		set_rgb_led(LED6, RGB_MAX_INTENSITY, 0, RGB_MAX_INTENSITY);
		set_rgb_led(LED8, RGB_MAX_INTENSITY, 0, RGB_MAX_INTENSITY);
		break;
	case CYAN:
		set_rgb_led(LED2, 0, RGB_MAX_INTENSITY, RGB_MAX_INTENSITY);
		set_rgb_led(LED4, 0, RGB_MAX_INTENSITY, RGB_MAX_INTENSITY);
		set_rgb_led(LED6, 0, RGB_MAX_INTENSITY, RGB_MAX_INTENSITY);
		set_rgb_led(LED8, 0, RGB_MAX_INTENSITY, RGB_MAX_INTENSITY);
		break;
	default:
		set_rgb_led(LED2, 0, 0, 0);
		set_rgb_led(LED4, 0, 0, 0);
		set_rgb_led(LED6, 0, 0, 0);
		set_rgb_led(LED8, 0, 0, 0);
	}
}


