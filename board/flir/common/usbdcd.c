#include <linux/delay.h>
#include "usbdcd.h"

#define usbdcd_clear_interrupt_field() writel(readl(USB_DCD_CONTROL) | USB_DCD_CONTROL_IACK, USB_DCD_CONTROL)
#define usbdcd_control_if(reg) ((reg) & USB_DCD_CONTROL_IF)
#define usbdcd_get_sequence_result(reg) (((reg) & USB_DCD_STATUS_SEQ_RES) >> USB_DCD_STATUS_SEQ_RES_SHIFT)
#define usbdcd_get_sequence_status(reg) (((reg) & USB_DCD_STATUS_SEQ_STAT) >> USB_DCD_STATUS_SEQ_STAT_SHIFT)
#define usbdcd_get_tunitcon() readl(USB_DCD_TIMER0) & USB_DCD_TUNITCON
#define usbdcd_set_signal_override_vdp() writel((readl(USB_DCD_SIGNAL_OVERRIDE) | USB_DCD_SIGNAL_OVERRIDE_PS_VDP_SRC), USB_DCD_SIGNAL_OVERRIDE)
#define usbdcd_software_reset() writel(readl(USB_DCD_CONTROL) | USB_DCD_CONTROL_SR, USB_DCD_CONTROL)
#define usbdcd_status_active(reg) ((reg) & USB_DCD_STATUS_ACTIVE)
#define usbdcd_status_err(reg) ((reg) & USB_DCD_STATUS_ERR)
#define usbdcd_status_to(reg) ((reg) & USB_DCD_STATUS_TO)
#define usbdcd_tdcd_dbnc() (readl(USB_DCD_TIMER1) & USB_DCD_TDCD_DBNC) >> USB_DCD_TDCD_DBNC_SHIFT

#define usbdcd_print(str, ...) do {			\
		printf("USB DCD [%d]: " str, usbdcd_get_tunitcon(), ##__VA_ARGS__); \
	} while (0)

static int usbdcd_start_charger_detection_sequence(void);

int usbdcd_get_charge_current_mA(int *charge_current_mA)
{
	u32 status_reg;

	if (!get_usb_cable_state())
		return 0;

	usbdcd_start_charger_detection_sequence();

	status_reg = usbdcd_data_pin_contact_detect();
	if (usbdcd_status_to(status_reg))
		return usbdcd_handle_timeout(charge_current_mA);

	status_reg = usbdcd_charger_port();
	if (usbdcd_status_to(status_reg))
		return usbdcd_handle_timeout(charge_current_mA);
	else if (usbdcd_status_err(status_reg))
		return usbdcd_handle_error(charge_current_mA);

	status_reg = usbdcd_charger_type();
	if (usbdcd_status_to(status_reg))
		return usbdcd_handle_timeout(charge_current_mA);

	switch (usbdcd_get_sequence_result(status_reg)) {
	case SEQ_DCP:
		usbdcd_print("Dedicated Charger Port detected\n");

		/* V_DP_SRC needs to be enabled if DCP according to BC
		 * 1.2
		 */
		usbdcd_set_signal_override_vdp();
		*charge_current_mA = 1500;
		break;
	case SEQ_CDP_DCP:
		usbdcd_print("Charging Downstream Port detected\n");

		/* Unknown why we need to set D+ high in this case. If
		 * not, the upstream port will after a moment stop
		 * supplying current.
		 */
		usbdcd_set_signal_override_vdp();
		*charge_current_mA = 1500;
		break;
	case SEQ_SDP:
		usbdcd_print("Standard Downstream Port detected\n");

		/* Set D+ to HIGH. According to BC 1.2 we are not allowed to
		 * draw more than 2.5mA if we have not reached configured (or
		 * enumerated) and the upstream port is an SDP. We will never
		 * reach configured if camera is off but charging. To be able
		 * to draw more current we need to fulfill the requirements
		 * for Dead Battery Provision and one of these requirements is
		 * to set D+ to high.
		 */
		usbdcd_set_signal_override_vdp();
		*charge_current_mA = 500;
		break;
	default:
		usbdcd_print("Unknown charging port detected, sequence result: x%x\n",
			     usbdcd_get_sequence_result(status_reg));
		usbdcd_set_signal_override_vdp();
		*charge_current_mA = 500;
		break;
	}

	usbdcd_software_reset();
	return 0;
}

static int usbdcd_start_charger_detection_sequence(void)
{
	/* Set data pin contact debounce timer. */
	usbdcd_set_tdcd_dbnc(50);

	/* Disable pulldown resistors on USB_DP and USB_DM. IF the
	 * pulldown resistors are enabled the data pins are held low
	 * which interferes with the Data Pin Contact Detection.
	 */
	writel(readl(USBPHY_ANACTRL) & ~USBPHY_ANACTRL_DEV_PULLDOWN, USBPHY_ANACTRL);

	usbdcd_software_reset();

	/* Disable interrupts. */
	writel((readl(USB_DCD_CONTROL) & ~USB_DCD_CONTROL_IE),
	       USB_DCD_CONTROL);

	usbdcd_clear_interrupt_field();

	/* Set Usb Battery Charging revision. */
	writel((readl(USB_DCD_CONTROL) | USB_DCD_CONTROL_BC12),
	       USB_DCD_CONTROL);

	/* Start charger detection sequence */
	writel((readl(USB_DCD_CONTROL) | USB_DCD_CONTROL_START),
	       USB_DCD_CONTROL);

	return 0;
}

void usbdcd_set_tdcd_dbnc(u32 val)
{
	u32 curr;

	curr = readl(USB_DCD_TIMER1) & ~USB_DCD_TDCD_DBNC;
	val = (val << USB_DCD_TDCD_DBNC_SHIFT) & USB_DCD_TDCD_DBNC;
	writel(curr | val, USB_DCD_TIMER1);
}

int usbdcd_handle_timeout(int *charge_current_mA)
{
	usbdcd_print("Failed to detect data pin connection: Timeout reached\n");
	usbdcd_software_reset();
	usbdcd_set_signal_override_vdp();
	*charge_current_mA = 500;

	return USB_DCD_TIMEOUT;
}

int usbdcd_handle_error(int *charge_current_mA)
{
	printf("Failed to complete charger detection sequence: Error occurred\n");
	usbdcd_software_reset();
	usbdcd_set_signal_override_vdp();
	*charge_current_mA = 500;

	return USB_DCD_ERROR;
}

/*
 * DETECT CHARGER TYPE
 *
 * Possible reasons for exiting while loop:
 * - Charger reaches sequence completion (not active)
 * - USBDCD module determines the type of the charger (interrupt).
 * - Timeout
 *
 * Notice that no errors are generated by the module at this
 * stage.
 */
u32 usbdcd_charger_type(void)
{
	u32 status_reg, control_reg;

	status_reg = readl(USB_DCD_STATUS);
	control_reg = readl(USB_DCD_CONTROL);

	while (!usbdcd_control_if(control_reg) &&
	       !usbdcd_status_to(status_reg) &&
	       usbdcd_status_active(status_reg)) {
		status_reg = readl(USB_DCD_STATUS);
		control_reg = readl(USB_DCD_CONTROL);
		mdelay(1);
	}
	usbdcd_clear_interrupt_field();

	return status_reg;
}

/*
 * CHARGING PORT DETECTION
 *
 * Possible reasons for exiting loop:
 * - Charger reaches sequence completion (not active)
 * - USBDCD module determines that port is charger or not (interrupt).
 * - Timeout
 * - Error
 */
u32 usbdcd_charger_port(void)
{
	u32 status_reg, control_reg;

	status_reg = readl(USB_DCD_STATUS);
	control_reg = readl(USB_DCD_CONTROL);

	while (!usbdcd_control_if(control_reg) &&
	       !usbdcd_status_to(status_reg) &&
	       !usbdcd_status_err(status_reg) &&
	       usbdcd_status_active(status_reg)) {
		status_reg = readl(USB_DCD_STATUS);
		control_reg = readl(USB_DCD_CONTROL);
		mdelay(1);
	}
	usbdcd_clear_interrupt_field();

	return status_reg;
}

u32 usbdcd_data_pin_contact_detect(void)
{
	u32 status_reg;
	status_reg = readl(USB_DCD_STATUS);

	while (!usbdcd_status_to(status_reg) &&
	       usbdcd_get_sequence_status(status_reg) == STAT_DISABLED) {
		status_reg = readl(USB_DCD_STATUS);
		mdelay(1);
	}

	return status_reg;
}
