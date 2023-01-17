#ifndef _DISPLAY_UTILS_H
#define _DISPLAY_UTILS_H

void display_set_text_color(void);
void display_print_string(char *s);
void display_print_charge_level(int c);
void display_draw_box(int x_start, int y_start, int width, int height, int color, void *framebuffer);

#endif
