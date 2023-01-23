// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 FLIR Systems.
 */
#ifndef _DISPLAY_UTILS_H
#define _DISPLAY_UTILS_H

#include <stdbool.h>

void display_set_text_color(void);
void display_print_string(char *s);
void display_print_charge_level(int c);
void display_draw_box(int x_start, int y_start, int width, int height,
		      int color, void *framebuffer);
void display_update_charge(int level);
void display_timer_reset(void);
void display_on(void);
void display_off(void);
bool display_is_on(void);
void display_check_timer(void);

#endif
