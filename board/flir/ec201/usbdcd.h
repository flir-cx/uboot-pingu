#include <asm/io.h>
#include <usb.h>

#include "../common/pf1550.h"

#define USBPHY_ANACTRL                               (USB_PHY0_BASE_ADDR + 0x100)
#define USBPHY_ANACTRL_DEV_PULLDOWN                  (1 << 10)

#define USBPHY_USB1_CHRG_DET_STAT                    (USB_PHY0_BASE_ADDR + 0xF0)
#define USBPHY_USB1_CHRG_DET_STAT_PLUG_CONTACT_MASK  (1 << 0)
#define USBPHY_USB1_CHRG_DET_STAT_CHRG_DETECTED_MASK (1 << 1)
#define USBPHY_USB1_CHRG_DET_STAT_DM_STATE_MASK      (1 << 2)
#define USBPHY_USB1_CHRG_DET_STAT_DP_STATE_MASK      (1 << 3)
#define USBPHY_USB1_CHRG_DET_STAT_SECDET_DCP_MASK    (1 << 4)

#define USB_DCD                                      (USB_PHY0_BASE_ADDR + 0x800)
#define USB_DCD_CONTROL                              (USB_DCD + 0x0)
#define USB_DCD_CONTROL_SR                           (1<<25)
#define USB_DCD_CONTROL_START                        (1<<24)
#define USB_DCD_CONTROL_BC12                         (1<<17)
#define USB_DCD_CONTROL_IE                           (1<<16)
#define USB_DCD_CONTROL_IACK                         (1)
#define USB_DCD_CONTROL_IF                           (1<<8)
#define USB_DCD_STATUS                               (USB_DCD + 0x8)
#define USB_DCD_STATUS_ACTIVE                        (1<<22)
#define USB_DCD_STATUS_TO                            (1<<21)
#define USB_DCD_STATUS_ERR                           (1<<20)
#define USB_DCD_STATUS_SEQ_STAT                      (0x000C0000)
#define USB_DCD_STATUS_SEQ_RES                       (0x00030000)
#define USB_DCD_STATUS_SEQ_STAT_SHIFT                (18)
#define USB_DCD_STATUS_SEQ_RES_SHIFT                 (16)
#define USB_DCD_SIGNAL_OVERRIDE                      (USB_DCD + 0xC)
#define USB_DCD_SIGNAL_OVERRIDE_PS_VDP_SRC           (0x2)
#define USB_DCD_TIMER0                               (USB_DCD + 0x10)
#define USB_DCD_TIMER1                               (USB_DCD + 0x14)
#define USB_DCD_TDCD_DBNC                            (0x3FF0000)
#define USB_DCD_TDCD_DBNC_SHIFT                      (16)
#define USB_DCD_TUNITCON                             (0xFFF)

enum {
	SEQ_NO_RESULT=0,
	SEQ_SDP=1,		/* Standard Downstream Port */
	SEQ_CDP_DCP=2,		/* Charging Downstream Port or Dedicated Charging Port */
	SEQ_DCP=3,		/* Dedicated Charging Port */
};

enum {
	STAT_DISABLED=0,
	STAT_DATA_PIN=1,
	STAT_CHARGING=2,
	STAT_CHARGER_TYPE=3,
};

enum {
	USB_DCD_ERROR=-2,
	USB_DCD_TIMEOUT=-1,
};

int usbdcd_get_charge_current_mA(int *charge_current_mA);
void usbdcd_set_tdcd_dbnc(u32 val);
u32 usbdcd_charger_port(void);
u32 usbdcd_data_pin_contact_detect(void);
u32 usbdcd_charger_type(void);
int usbdcd_handle_error(int *charge_current_mA);
int usbdcd_handle_timeout(int *charge_current_mA);
