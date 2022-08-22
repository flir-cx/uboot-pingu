// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 Doug Zobel <douglas.zobel@climate.com>
 *
 * Driver for TI lp5562 4 channel LED driver.  There are only 3
 * engines available for the 4 LEDs, so white and blue LEDs share
 * the same engine.  This means that the blink period is shared
 * between them.  Changing the period of blue blink will affect
 * the white period (and vice-versa).  Blue and white On/Off
 * states remain independent (as would PWM brightness if that's
 * ever added to the LED core).
 */

#include <common.h>
#include <dm.h>
#include <errno.h>
#include <led.h>
#include <i2c.h>
#include <asm/gpio.h>
#include <dm/lists.h>
#include "led_lp5562.h"

#define DEFAULT_CURRENT			0x64 /* 10 ma */
#define MIN_BLINK_PERIOD		32   /* ms */
#define MAX_BLINK_PERIOD		2248 /* ms */

/* Register Map */
#define REG_ENABLE			0x00
#define REG_OP_MODE			0x01
#define REG_B_PWM			0x02
#define REG_G_PWM			0x03
#define REG_R_PWM			0x04
#define REG_B_CUR			0x05
#define REG_G_CUR			0x06
#define REG_R_CUR			0x07
#define REG_CONFIG			0x08
#define REG_ENG1_PC			0x09
#define REG_ENG2_PC			0x0A
#define REG_ENG3_PC			0x0B
#define REG_STATUS			0x0C
#define REG_RESET			0x0D
#define REG_W_PWM			0x0E
#define REG_W_CUR			0x0F
#define REG_ENG1_MEM_BEGIN		0x10
#define REG_ENG2_MEM_BEGIN		0x30
#define REG_ENG3_MEM_BEGIN		0x50
#define REG_LED_MAP			0x70

/* LED Register Values */
/* 0x00  ENABLE */
#define REG_ENABLE_CHIP_ENABLE		(0x1 << 6)
#define REG_ENABLE_ENG_EXEC_HOLD	0x0
#define REG_ENABLE_ENG_EXEC_RUN		0x2
#define REG_ENABLE_ENG_EXEC_MASK	0x3

/* 0x01  OP MODE */
#define REG_OP_MODE_DISABLED		0x0
#define REG_OP_MODE_LOAD_SRAM		0x1
#define REG_OP_MODE_RUN			0x2
#define REG_OP_MODE_MASK		0x3

/* 0x02, 0x03, 0x04, 0x0E  PWM */
#define REG_PWM_MIN_VALUE		0
#define REG_PWM_MAX_VALUE		0xFF

/* 0x05, 0x06, 0x07, 0x0F  CURRENT */
#define REG_CUR_MAX_VALUE		0xFF

/* 0x08  CONFIG */
#define REG_CONFIG_EXT_CLK		0x0
#define REG_CONFIG_INT_CLK		0x1
#define REG_CONFIG_AUTO_CLK		0x2

/* 0x70  LED MAP */
#define REG_LED_MAP_ENG_MASK		0x03
#define REG_LED_MAP_W_ENG_SHIFT		6
#define REG_LED_MAP_R_ENG_SHIFT		4
#define REG_LED_MAP_G_ENG_SHIFT		2
#define REG_LED_MAP_B_ENG_SHIFT		0

/* Engine program related */
#define REG_ENGINE_MEM_SIZE		0x20
#define LED_PGRM_RAMP_INCREMENT_SHIFT	0
#define LED_PGRM_RAMP_SIGN_SHIFT	7
#define LED_PGRM_RAMP_STEP_SHIFT	8
#define LED_PGRM_RAMP_PRESCALE_SHIFT	14

struct lp5562_led_priv {
	struct gpio_desc enable_gpio;
	u8 reg_pwm;
	u8 reg_current;
	u8 map_shift;
	u8 enginenum;
};

/* enum values map to LED_MAP (0x70) values */
enum lp5562_led_ctl_mode {
	I2C = 0x0,
#ifdef CONFIG_LED_BLINK
	ENGINE1 = 0x1,
	ENGINE2 = 0x2,
	ENGINE3 = 0x3
#endif
};

/*
 * Update a register value
 *  dev     - I2C udevice (parent of led)
 *  regnum  - register number to update
 *  value   - value to write to register
 *  mask    - mask of bits that should be changed
 */
static int lp5562_led_reg_update(struct udevice *dev, int regnum,
				 u8 value, u8 mask)
{
	u8 data;

	if (mask == 0xFF) {
		data = value;
	} else {
		if (dm_i2c_read(dev, regnum, &data, 1) != 0)
			return -EINVAL;

		data = (data & ~(mask)) | value;
	}

	if (dm_i2c_write(dev, regnum, &data, 1) != 0)
		return -EINVAL;

	/*
	 * The datasheet says to wait up to 488uS between i2c writes, but I
	 * found it had to be over 600us to be reliable.  Using 1ms to be safe.
	 */
	udelay(1000);

	return 0;
}

#ifdef CONFIG_LED_BLINK
/*
 * Program the lp5562 engine
 *  dev     - I2C udevice (parent of led)
 *  program - array of commands
 *  size    - number of commands in program array (1-16)
 *  engine  - engine number (1-3)
 */
static int lp5562_led_program_engine(struct udevice *dev, u16 *program,
				     u8 size, u8 engine)
{
	int ret, cmd;
	u8 engine_reg = REG_ENG1_MEM_BEGIN +
			     ((engine - 1) * REG_ENGINE_MEM_SIZE);
	u8 shift = (3 - engine) * 2;
	u8 *reordered;

	if (size < 1 || size > 16 ||
	    (engine != 1 && engine != 2 && engine != 3))
		return -EINVAL;

	reordered = malloc(sizeof(u16) * size);
	if (!reordered)
		ret = -ENOMEM;

	for (cmd = 0; cmd < size; cmd++) {
		reordered[sizeof(u16) * cmd] = program[cmd] >> 8;
		reordered[(sizeof(u16) * cmd) + 1] = program[cmd] & 0xFF;
	}

	/* set engine mode to 'disabled' */
	ret = lp5562_led_reg_update(dev, REG_OP_MODE,
				    REG_OP_MODE_DISABLED << shift,
				    REG_OP_MODE_MASK << shift);
	if (ret != 0)
		goto done;

	/* set exec mode to 'hold' */
	ret = lp5562_led_reg_update(dev, REG_ENABLE,
				    REG_ENABLE_ENG_EXEC_HOLD << shift,
				    REG_ENABLE_ENG_EXEC_MASK << shift);
	if (ret != 0)
		goto done;

	/* set engine mode to 'load SRAM' */
	ret = lp5562_led_reg_update(dev, REG_OP_MODE,
				    REG_OP_MODE_LOAD_SRAM << shift,
				    REG_OP_MODE_MASK << shift);
	if (ret != 0)
		goto done;

	/* send the re-ordered program sequence */
	ret = dm_i2c_write(dev, engine_reg, (uchar *)reordered,
			   sizeof(u16) * size);
	if (ret != 0)
		goto done;

	/* set engine mode to 'run' */
	ret = lp5562_led_reg_update(dev, REG_OP_MODE,
				    REG_OP_MODE_RUN << shift,
				    REG_OP_MODE_MASK << shift);
	if (ret != 0)
		goto done;

	/* set engine exec to 'run' */
	ret = lp5562_led_reg_update(dev, REG_ENABLE,
				    REG_ENABLE_ENG_EXEC_RUN << shift,
				    REG_ENABLE_ENG_EXEC_MASK << shift);

done:
	free(reordered);
	return ret;
}

/*
 * Get the LED's current control mode (I2C or ENGINE[1-3])
 *  dev       - led udevice (child udevice)
 */
static enum lp5562_led_ctl_mode lp5562_led_get_control_mode(struct udevice *dev)
{
	struct lp5562_led_priv *priv = dev_get_priv(dev);
	u8 data;
	enum lp5562_led_ctl_mode mode = I2C;

	if (dm_i2c_read(dev_get_parent(dev), REG_LED_MAP, &data, 1) == 0)
		mode = (data & (REG_LED_MAP_ENG_MASK << priv->map_shift))
			>> priv->map_shift;

	return mode;
}
#endif

/*
 * Set the LED's control mode to I2C or ENGINE[1-3]
 *  dev       - led udevice (child udevice)
 *  mode      - mode to change to
 */
static int lp5562_led_set_control_mode(struct udevice *dev,
				       enum lp5562_led_ctl_mode mode)
{
	struct lp5562_led_priv *priv = dev_get_priv(dev);

	return (lp5562_led_reg_update(dev_get_parent(dev), REG_LED_MAP,
				      mode << priv->map_shift,
				      REG_LED_MAP_ENG_MASK << priv->map_shift));
}

/*
 * Return the LED's PWM value;  If LED is in BLINK state, then it is
 * under engine control mode which doesn't use this PWM value.
 *  dev       - led udevice (child udevice)
 */
static int lp5562_led_get_pwm(struct udevice *dev)
{
	struct lp5562_led_priv *priv = dev_get_priv(dev);
	u8 data;

	if (dm_i2c_read(dev_get_parent(dev), priv->reg_pwm, &data, 1) != 0)
		return -EINVAL;

	return data;
}

/*
 * Set the LED's PWM value and configure it to use this (I2C mode).
 *  dev       - led udevice (child udevice)
 *  value     - PWM value (0 - 255)
 */
static int lp5562_led_set_pwm(struct udevice *dev, u8 value)
{
	struct lp5562_led_priv *priv = dev_get_priv(dev);

	if (lp5562_led_reg_update(dev_get_parent(dev), priv->reg_pwm,
				  value, 0xff) != 0)
		return -EINVAL;

	/* set LED to I2C register mode */
	return lp5562_led_set_control_mode(dev, I2C);
}

/*
 * Return the led's current state
 *  dev     - led udevice (child udevice)
 *
 */
static enum led_state_t lp5562_led_get_state(struct udevice *dev)
{
	enum led_state_t state = LEDST_ON;

	if (lp5562_led_get_pwm(dev) == REG_PWM_MIN_VALUE)
		state = LEDST_OFF;

#ifdef CONFIG_LED_BLINK
	if (lp5562_led_get_control_mode(dev) != I2C)
		state = LEDST_BLINK;
#endif

	return state;
}

/*
 * Set the led state
 *  dev     - led udevice (child udevice)
 *  state   - State to set the LED to
 */
static int lp5562_led_set_state(struct udevice *dev, enum led_state_t state)
{
#ifdef CONFIG_LED_BLINK
	struct lp5562_led_priv *priv = dev_get_priv(dev);
#endif

	switch (state) {
	case LEDST_OFF:
		return lp5562_led_set_pwm(dev, REG_PWM_MIN_VALUE);
	case LEDST_ON:
		return lp5562_led_set_pwm(dev, REG_PWM_MAX_VALUE);
#ifdef CONFIG_LED_BLINK
	case LEDST_BLINK:
		return lp5562_led_set_control_mode(dev, priv->enginenum);
#endif
	case LEDST_TOGGLE:
		if (lp5562_led_get_state(dev) == LEDST_OFF)
			return lp5562_led_set_state(dev, LEDST_ON);
		else
			return lp5562_led_set_state(dev, LEDST_OFF);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

#ifdef CONFIG_LED_BLINK

static int lp5562_led_pattern_boot(u16 *p1, u16 *p2, u16 *p3)
{
	static const u16 bp1[] = { 0x087F, 0xE004, 0x04FF, 0xE200, 0x0000, };
	static const u16 bp2[] = { 0xE080, 0x087F, 0xE008, 0x04FF, 0x0000, };
	static const u16 bp3[] = { 0xE100, 0x087F, 0xE002, 0x04FF, 0x0000, };

	memcpy(p1, bp1, sizeof(bp1));
	memcpy(p2, bp2, sizeof(bp2));
	memcpy(p3, bp3, sizeof(bp3));

	return sizeof(bp1);
}

static int lp5562_led_pattern_ramp_all(u16 *p1, u16 *p2, u16 *p3, u16 wait_time)
{
	int opc = 0;

	// Engine 1
	p1[opc++] = LED_PGRM_WAIT_TRIGGER3;
	p1[opc++] = LED_PGRM_INCREMENT;
	p1[opc++] = LED_PGRM_INCREMENT;
	p1[opc++] = LED_PGRM_STOP_NO_RESET;

	// Engine 2
	opc = 0;
	p2[opc++] = LED_PGRM_WAIT_TRIGGER3;
	p2[opc++] = LED_PGRM_INCREMENT;
	p2[opc++] = LED_PGRM_INCREMENT;
	p2[opc++] = LED_PGRM_STOP_NO_RESET;

	// Engine 3
	opc = 0;
	p3[opc++] = LED_PGRM_WAIT(wait_time);
	p3[opc++] = LED_PGRM_SEND_TRIGGER12;
	p3[opc++] = LED_PGRM_INCREMENT;
	p3[opc++] = LED_PGRM_INCREMENT;
	p3[opc++] = LED_PGRM_STOP_NO_RESET;

	return opc;
}

static int lp5562_led_pattern_fade_all(u16 *p1, u16 *p2, u16 *p3, u16 wait_time)
{
	int opc = 0;

	// Engine 1
	p1[opc++] = LED_PGRM_WAIT_TRIGGER3;
	p1[opc++] = LED_PGRM_DECREMENT;
	p1[opc++] = LED_PGRM_DECREMENT;
	p1[opc++] = LED_PGRM_STOP_NO_RESET; //STOP no reset

	// Engine 2
	opc = 0;
	p2[opc++] = LED_PGRM_WAIT_TRIGGER3;
	p2[opc++] = LED_PGRM_DECREMENT;
	p2[opc++] = LED_PGRM_DECREMENT;
	p2[opc++] = LED_PGRM_STOP_NO_RESET;

	// Engine 3
	opc = 0;
	p3[opc++] = LED_PGRM_WAIT(wait_time);
	p3[opc++] = LED_PGRM_SEND_TRIGGER12;
	p3[opc++] = LED_PGRM_DECREMENT;
	p3[opc++] = LED_PGRM_DECREMENT;
	p3[opc++] = LED_PGRM_STOP_NO_RESET;

	return opc;
}

static int lp5562_led_pattern_blink(u16 *p1, u16 *p2, u16 *p3, u16 wait_time)
{
	int opc = 0;

	// Engine 1
	p1[opc++] = LED_PGRM_WAIT_TRIGGER3;
	p1[opc++] = LED_PGRM_INCREMENT;
	p1[opc++] = LED_PGRM_INCREMENT;
	p1[opc++] = LED_PGRM_WAIT(wait_time);
	p1[opc++] = LED_PGRM_DECREMENT;
	p1[opc++] = LED_PGRM_DECREMENT;
	p1[opc++] = LED_PGRM_WAIT(wait_time);
	p1[opc++] = LED_PGRM_GOTO_START;

	// Engine 2
	opc = 0;
	p2[opc++] = LED_PGRM_WAIT_TRIGGER3;
	p2[opc++] = LED_PGRM_INCREMENT;
	p2[opc++] = LED_PGRM_INCREMENT;
	p2[opc++] = LED_PGRM_WAIT(wait_time);
	p2[opc++] = LED_PGRM_DECREMENT;
	p2[opc++] = LED_PGRM_DECREMENT;
	p2[opc++] = LED_PGRM_WAIT(wait_time);
	p2[opc++] = LED_PGRM_GOTO_START;

	// Engine 3
	opc = 0;
	p3[opc++] = LED_PGRM_WAIT(wait_time);
	p3[opc++] = LED_PGRM_SEND_TRIGGER12;
	p3[opc++] = LED_PGRM_INCREMENT;
	p3[opc++] = LED_PGRM_INCREMENT;
	p3[opc++] = LED_PGRM_WAIT(wait_time);
	p3[opc++] = LED_PGRM_DECREMENT;
	p3[opc++] = LED_PGRM_DECREMENT;
	p3[opc++] = LED_PGRM_GOTO_START;

	return opc;
}

#define PATTERN_BOOT 0
#define PATTERN_RAMP_ALL 1
#define PATTERN_FADE_ALL 2
#define PATTERN_BLINK_ALL 3


static int lp5562_led_program_pattern(struct udevice *dev, int program, int period_ms)
{
	struct lp5562_led_priv *priv = dev_get_priv(dev);
	struct udevice *parent_dev = dev_get_parent(dev);

	u8 opcode = 0;
	u16 program1[20];
	u16 program2[20];
	u16 program3[20];
	u16 wait_time;

	u16 boot_ramp;
	u16 boot_wait;

	int ret;

	printf("program pattern %d %d\n", program, period_ms);

	if (period_ms < MIN_BLINK_PERIOD)
		period_ms = MIN_BLINK_PERIOD;
	else if (period_ms > MAX_BLINK_PERIOD)
		period_ms = MAX_BLINK_PERIOD;


	wait_time = ((period_ms - 251) / 2);

#define BOOT_RAMP_STEP 0x3
#define WAIT_RAMP_STEP 0xD

	if(period_ms == 1000)
	{
		boot_ramp = BOOT_RAMP_STEP;
		boot_wait = WAIT_RAMP_STEP;
	}
	else
	{
		boot_ramp = 0x1;
		boot_wait = 0xD;
	}

	switch (program)
	{
		case PATTERN_BOOT:
			opcode = lp5562_led_pattern_boot(program1, program2, program3);
			break;
		case PATTERN_RAMP_ALL:
			opcode = lp5562_led_pattern_ramp_all(program1, program2, program3, wait_time);
			break;
		case PATTERN_FADE_ALL:
			opcode = lp5562_led_pattern_fade_all(program1, program2, program3, wait_time);
			break;
		case PATTERN_BLINK_ALL:
			opcode = lp5562_led_pattern_blink(program1, program2, program3, wait_time);
			break;
		default:
			return 0;
			break;
	}

	ret = lp5562_led_program_engine(parent_dev, program3, opcode, 3);
	if(ret)
	{
		printf("failed to program engine 3: %d", ret);
	}
	ret = lp5562_led_program_engine(parent_dev, program2, opcode, 2);
	if(ret)
	{
		printf("failed to program engine 2: %d", ret);
		goto done;
	}
	ret = lp5562_led_program_engine(parent_dev, program1, opcode, 1);
	if(ret)
	{
		printf("failed to program engine 1: %d", ret);
		goto done;
	}
done:
	return ret;
}


/*
 * Set the blink period of an LED; note blue and white share the same
 * engine so changing the period of one affects the other.
 *  dev       - led udevice (child udevice)
 *  period_ms - blink period in ms
 */
static int lp5562_led_set_period(struct udevice *dev, int period_ms)
{
	struct lp5562_led_priv *priv = dev_get_priv(dev);
	u8 opcode = 0;
	u16 program[7];
	u16 wait_time;

	/* Blink is implemented as an engine program.  Simple on/off
	 * for short periods, or fade in/fade out for longer periods:
	 *
	 *  if (period_ms < 500):
	 *    set PWM to 100%
	 *    pause for period / 2
	 *    set PWM to 0%
	 *    pause for period / 2
	 *    goto start
	 *
	 *  else
	 *    raise PWM 0% -> 50% in 62.7 ms
	 *    raise PWM 50% -> 100% in 62.7 ms
	 *    pause for (period - 4 * 62.7) / 2
	 *    lower PWM 100% -> 50% in 62.7 ms
	 *    lower PWM 50% -> 0% in 62.7 ms
	 *    pause for (period - 4 * 62.7) / 2
	 *    goto start
	 */

	if (period_ms < MIN_BLINK_PERIOD)
		period_ms = MIN_BLINK_PERIOD;
	else if (period_ms > MAX_BLINK_PERIOD)
		period_ms = MAX_BLINK_PERIOD;

	if (period_ms < 500) {
		/* Simple on/off blink */
		wait_time = period_ms / 2;

		/* 1st command is full brightness */
		program[opcode++] =
			(1 << LED_PGRM_RAMP_PRESCALE_SHIFT) |
			REG_PWM_MAX_VALUE;

		/* 2nd command is wait (period / 2) using 15.6ms steps */
		program[opcode++] =
			(1 << LED_PGRM_RAMP_PRESCALE_SHIFT) |
			(((wait_time * 10) / 156) << LED_PGRM_RAMP_STEP_SHIFT) |
			(0 << LED_PGRM_RAMP_INCREMENT_SHIFT);

		/* 3rd command is 0% brightness */
		program[opcode++] =
			(1 << LED_PGRM_RAMP_PRESCALE_SHIFT);

		/* 4th command is wait (period / 2) using 15.6ms steps */
		program[opcode++] =
			(1 << LED_PGRM_RAMP_PRESCALE_SHIFT) |
			(((wait_time * 10) / 156) << LED_PGRM_RAMP_STEP_SHIFT) |
			(0 << LED_PGRM_RAMP_INCREMENT_SHIFT);

		/* 7th command: repeat */
		program[opcode++] = LED_PGRM_GOTO_START;
	} else {
		/* fade-in / fade-out blink */
		wait_time = ((period_ms - 251) / 2);

		/* ramp up time is 256 * 0.49ms (125.4ms) done in 2 steps */
		/* 1st command is ramp up 1/2 way */
		program[opcode++] = LED_PGRM_INCREMENT;

		/* 2nd command is ramp up rest of the way */
		program[opcode++] = LED_PGRM_INCREMENT;

		/* 3rd: wait ((period - 2 * ramp_time) / 2) (15.6ms steps) */
		program[opcode++] = LED_PGRM_WAIT(wait_time);

		/* ramp down is same as ramp up with sign bit set */
		/* 4th command is ramp down 1/2 way */
		program[opcode++] = LED_PGRM_DECREMENT;

		/* 5th command is ramp down rest of the way */
		program[opcode++] = LED_PGRM_DECREMENT;

		/* 6th: wait ((period - 2 * ramp_time) / 2) (15.6ms steps) */
		program[opcode++] = LED_PGRM_WAIT(wait_time);

		/* 7th command: repeat */
		program[opcode++] = LED_PGRM_GOTO_START;
	}

	return lp5562_led_program_engine(dev_get_parent(dev), program,
					 opcode, priv->enginenum);
}
#endif

static const struct led_ops lp5562_led_ops = {
	.get_state = lp5562_led_get_state,
	.set_state = lp5562_led_set_state,
#ifdef CONFIG_LED_BLINK
	.set_period = lp5562_led_set_period,
	.set_pattern = lp5562_led_program_pattern,
#endif
};

static int lp5562_led_probe(struct udevice *dev)
{
	struct lp5562_led_priv *priv = dev_get_priv(dev);
	struct led_uc_plat *uc_plat = dev_get_uclass_plat(dev);
	u32 current;
	int ret = 0;

	/* Top-level LED node */
	if (!uc_plat->label) {
		/* Enable gpio if needed */
		if (gpio_request_by_name(dev, "gpios", 0,
					 &priv->enable_gpio, GPIOD_IS_OUT) == 0)
			dm_gpio_set_value(&priv->enable_gpio, 1);

		/* Enable the chip */
		if (lp5562_led_reg_update(dev, REG_ENABLE,
					  REG_ENABLE_CHIP_ENABLE, 0xff) != 0)
			return -EINVAL;

		if (!dev_read_bool(dev, "ti,external_clock")) {
			if (lp5562_led_reg_update(dev, REG_CONFIG,
						  REG_CONFIG_INT_CLK, 0xff)
			    != 0)
				return -EINVAL;
		}
	} else {
		/* Child LED nodes */
		priv->reg_pwm = dev_read_addr(dev);
		switch (priv->reg_pwm) {
		case (REG_B_PWM):
			priv->reg_current = REG_B_CUR;
			priv->map_shift = REG_LED_MAP_B_ENG_SHIFT;
			priv->enginenum = 1; /* shared with white */
			break;
		case (REG_G_PWM):
			priv->reg_current = REG_G_CUR;
			priv->map_shift = REG_LED_MAP_G_ENG_SHIFT;
			priv->enginenum = 2;
			break;
		case (REG_R_PWM):
			priv->reg_current = REG_R_CUR;
			priv->map_shift = REG_LED_MAP_R_ENG_SHIFT;
			priv->enginenum = 3;
			break;
		case (REG_W_PWM):
			priv->reg_current = REG_W_CUR;
			priv->map_shift = REG_LED_MAP_W_ENG_SHIFT;
			priv->enginenum = 1; /* shared with blue */
			break;
		default:
			return -EINVAL;
		}

		current = dev_read_u32_default(dev, "ti,led_current",
					       DEFAULT_CURRENT);
		if (current > REG_CUR_MAX_VALUE) {
			debug("%s: node %s has invalid current limit: %d\n",
			      __func__, uc_plat->label, current);
			return -EINVAL;
		}

		if (lp5562_led_reg_update(dev_get_parent(dev),
					  priv->reg_current,
					  current, 0xff) != 0)
			return -EINVAL;
	}

	return ret;
}

static int lp5562_led_bind(struct udevice *parent)
{
	ofnode node;

	dev_for_each_subnode(node, parent) {
		struct led_uc_plat *uc_plat;
		struct udevice *dev;
		const char *label;
		int ret;

		label = ofnode_read_string(node, "label");
		if (!label) {
			debug("%s: node %s has no label\n", __func__,
			      ofnode_get_name(node));
			return -EINVAL;
		}

		ret = device_bind_driver_to_node(parent, "lp5562-led",
						 ofnode_get_name(node),
						 node, &dev);
		if (ret)
			return ret;

		uc_plat = dev_get_uclass_plat(dev);
		uc_plat->label = label;
	}

	return 0;
}

static const struct udevice_id lp5562_led_ids[] = {
	{ .compatible = "ti,lp5562-leds" },
	{ /* sentinel */ }
};

U_BOOT_DRIVER(lp5562_led) = {
	.name = "lp5562-led",
	.id = UCLASS_LED,
	.of_match = lp5562_led_ids,
	.bind = lp5562_led_bind,
	.probe = lp5562_led_probe,
	.priv_auto = sizeof(struct lp5562_led_priv),
	.ops = &lp5562_led_ops,
};
