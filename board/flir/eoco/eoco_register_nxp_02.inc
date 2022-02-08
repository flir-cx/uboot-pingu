//=============================================================================
//init script for i.MX6QP LPDDR2
//=============================================================================
// Revision History
// v1.5	Fixed Mode Only
//=============================================================================

//wait = on	// uncomment if using debugger
//=============================================================================
// Disable	WDOG
//=============================================================================
//setmem /16	0x020bc000 =	0x30	// uncomment if using debugger

//=============================================================================
// Enable all clocks (they are disabled by ROM code)
//=============================================================================
setmem /32	0x020c4068 =	0xffffffff
setmem /32	0x020c406c =	0xffffffff
setmem /32	0x020c4070 =	0xffffffff
setmem /32	0x020c4074 =	0xffffffff
setmem /32	0x020c4078 =	0xffffffff
setmem /32	0x020c407c =	0xffffffff
setmem /32	0x020c4080 =	0xffffffff

setmem /32	0x020c4018 =	0x00060324	//DDR clk to 400MHz
//=============================================================================
// IOMUX
//=============================================================================
//DDR IO TYPE:
setmem /32	0x020e0798 =	0x00080000	// IOMUXC_SW_PAD_CTL_GRP_DDR_TYPE
setmem /32	0x020e0758 =	0x00000000	// IOMUXC_SW_PAD_CTL_GRP_DDRPKE

//CLOCK:
setmem /32	0x020e0588 =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDCLK_0
setmem /32	0x020e0594 =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDCLK_1

//ADDRESS:
setmem /32	0x020e056c =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_CAS
setmem /32	0x020e0578 =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_RAS
setmem /32	0x020e074c =	0x00000030	// IOMUXC_SW_PAD_CTL_GRP_ADDDS

//CONTROL:
setmem /32	0x020e057c =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_RESET

setmem /32	0x020e058c =	0x00000000	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDBA2 - DSE can be configured using Group Control Register: IOMUXC_SW_PAD_CTL_GRP_CTLDS
setmem /32	0x020e059c =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDODT0
setmem /32	0x020e05a0 =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDODT1
setmem /32	0x020e078c =	0x00000030	// IOMUXC_SW_PAD_CTL_GRP_CTLDS

//DATA STROBE:
setmem /32	0x020e0750 =	0x00020000	// IOMUXC_SW_PAD_CTL_GRP_DDRMODE_CTL

setmem /32	0x020e05a8 =	0x00003030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS0
setmem /32	0x020e05b0 =	0x00003030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS1
setmem /32	0x020e0524 =	0x00003030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS2
setmem /32	0x020e051c =	0x00003030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS3
setmem /32	0x020e0518 =	0x00003030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS4
setmem /32	0x020e050c =	0x00003030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS5
setmem /32	0x020e05b8 =	0x00003030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS6
setmem /32	0x020e05c0 =	0x00003030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS7

//DATA:
setmem /32	0x020e0774 =	0x00020000	// IOMUXC_SW_PAD_CTL_GRP_DDRMODE

setmem /32	0x020e0784 =	0x00000030	// IOMUXC_SW_PAD_CTL_GRP_B0DS
setmem /32	0x020e0788 =	0x00000030	// IOMUXC_SW_PAD_CTL_GRP_B1DS
setmem /32	0x020e0794 =	0x00000030	// IOMUXC_SW_PAD_CTL_GRP_B2DS
setmem /32	0x020e079c =	0x00000030	// IOMUXC_SW_PAD_CTL_GRP_B3DS
setmem /32	0x020e07a0 =	0x00000030	// IOMUXC_SW_PAD_CTL_GRP_B4DS
setmem /32	0x020e07a4 =	0x00000030	// IOMUXC_SW_PAD_CTL_GRP_B5DS
setmem /32	0x020e07a8 =	0x00000030	// IOMUXC_SW_PAD_CTL_GRP_B6DS
setmem /32	0x020e0748 =	0x00000030	// IOMUXC_SW_PAD_CTL_GRP_B7DS

setmem /32	0x020e05ac =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM0
setmem /32	0x020e05b4 =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM1
setmem /32	0x020e0528 =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM2
setmem /32	0x020e0520 =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM3
setmem /32	0x020e0514 =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM4
setmem /32	0x020e0510 =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM5
setmem /32	0x020e05bc =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM6
setmem /32	0x020e05c4 =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM7

//=============================================================================
// DDR Controller Registers
//=============================================================================
// Manufacturer:	Nanya
// Device Part Number:	NT6TL128M32BA
// Clock Freq.: 	400MHz
// Density per CS in Gb: 	4
// Chip Selects used:	2
// Number of Banks:	8
// Row address:    	14
// Column address: 	10
// Data bus width	32
//=============================================================================

// MMDC0_MDSCR, set the Configuration request bit during MMDC set up
setmem /32	0x021b001c =	0x00008000	// Chan 0
setmem /32	0x021b401c =	0x00008000	// Chan 1

setmem /32	0x021b085c = 	0x1B4700C7	//LPDDR2 ZQ params
setmem /32	0x021b485c = 	0x1B4700C7	//LPDDR2 ZQ params

// values for CA bus delay were optimized for the validation board, where the setting of 0x00400000 is the default value
setmem /32	0x021b0890 =	0x00480C58	// MMDC0_MPPDCMPR2: ZQ_PU_OFFSET and ZQ_PD_OFFSET optimal settings
setmem /32	0x021b4890 =	0x00480C58	// MMDC0_MPPDCMPR2: ZQ_PU_OFFSET and ZQ_PD_OFFSET optimal settings
setmem /32	0x021b0800 =	0xA1390003	// DDR_PHY_P0_MPZQHWCTRL, enable both one-time & periodic HW ZQ calibration.
setmem /32	0x021b4800 =	0xA1380003	// DDR_PHY_P1_MPZQHWCTRL, enable periodic HW ZQ calibration.

setmem /32	0x021b080c = 	0x00000000
setmem /32	0x021b0810 = 	0x00000000
setmem /32	0x021b480c = 	0x00000000
setmem /32	0x021b4810 = 	0x00000000

//######################################################
//calibration values based on calibration compare of 0x00ffff00:
//Note, these calibration values are based on NXP's board
//May need to run calibration on target board to fine tune these
//######################################################

//Read calibration
setmem /32	0x021b0848 =	0x44383E3C	// MPRDDLCTL PHY0
setmem /32	0x021b4848 =	0x3C3C3846	// MPRDDLCTL PHY1

//Write calibration
setmem /32	0x021b0850 =	0x3C3A3E3E	// MPWRDLCTL PHY0
setmem /32	0x021b4850 =	0x38283E34	// MPWRDLCTL PHY1

//read data bit delay: (3 is the reccommended default value, although out of reset value is 0):
setmem /32	0x021b081c =	0x33333333	// DDR_PHY_P0_MPREDQBY0DL3
setmem /32	0x021b0820 =	0x33333333	// DDR_PHY_P0_MPREDQBY1DL3
setmem /32	0x021b0824 =	0x33333333	// DDR_PHY_P0_MPREDQBY2DL3
setmem /32	0x021b0828 =	0x33333333	// DDR_PHY_P0_MPREDQBY3DL3
setmem /32	0x021b481c =	0x33333333	// DDR_PHY_P1_MPREDQBY0DL3
setmem /32	0x021b4820 =	0x33333333	// DDR_PHY_P1_MPREDQBY1DL3
setmem /32	0x021b4824 =	0x33333333	// DDR_PHY_P1_MPREDQBY2DL3
setmem /32	0x021b4828 =	0x33333333	// DDR_PHY_P1_MPREDQBY3DL3

setmem /32	0x021b08c0 =	0x24912249	// fine tune SDCLK duty cyc to low - Engineering Bulletin EB828
setmem /32	0x021b48c0 =	0x24914289

// Complete calibration by forced measurement:
setmem /32	0x021b08b8 =	0x00000800 	// DDR_PHY_P0_MPMUR0, frc_msr
setmem /32	0x021b48b8 =	0x00000800 	// DDR_PHY_P0_MPMUR0, frc_msr

//MMDC init Ch0:
setmem /32	0x021b0018 =	0x00001602	// Perform software reset of MMDC prior to initializing its registers
setmem /32	0x021b0004 =	0x00020024	// MMDC0_MDPDC
setmem /32	0x021b0008 =	0x00000000	// MMDC0_MDOTC
setmem /32	0x021b000c =	0x53574135	// MMDC0_MDCFG0
setmem /32	0x021b0010 =	0x00100A83	// MMDC0_MDCFG1
setmem /32	0x021b0014 =	0x00000093	// MMDC0_MDCFG2
setmem /32	0x021b0018 =	0x000016CC	// MMDC0_MDMISC
setmem /32	0x021b002c =	0x0F9F26D2	// MMDC0_MDRWD; recommend to maintain the default values
setmem /32	0x021b0030 =	0x009F0E10	// MMDC0_MDOR
setmem /32	0x021b0038 =	0x001A0889	// MMDC0_MDCFG3LP
setmem /32	0x021b0040 =	0x0000004F	// CS0_END
setmem /32	0x021b0400 =	0x514201F0	// adopt bypass only if NOC schedular enabled Ch0
setmem /32	0x021b0000 =	0xC3110000	// MMDC0_MDCTL

//MMDC init Ch1:
setmem /32	0x021b4018 =	0x00001602	// Perform software reset of MMDC prior to initializing its registers
setmem /32	0x021b4004 =	0x00020024	// MMDC1_MDPDC
setmem /32	0x021b4008 =	0x00000000	// MMDC1_MDOTC
setmem /32	0x021b400c =	0x53574135	// MMDC1_MDCFG0
setmem /32	0x021b4010 =	0x00100A83	// MMDC1_MDCFG1
setmem /32	0x021b4014 =	0x00000093	// MMDC1_MDCFG2
setmem /32	0x021b4018 =	0x000016CC	// MMDC1_MDMISC
setmem /32	0x021b402c =	0x0F9F26D2	// MMDC1_MDRWD; recommend to maintain the default values
setmem /32	0x021b4030 =	0x009F0E10	// MMDC1_MDOR
setmem /32	0x021b4038 =	0x001A0889	// MMDC1_MDCFG3LP
setmem /32	0x021b4040 =	0x00000017	// CS0_END
setmem /32	0x021b4000 =	0xC3110000	// MMDC1_MDCTL


//########################################################
//## NOC DDR configuration
setmem /32	0x00bb0008 =	0x0000000E	// NoC DDRCONF
setmem /32	0x00bb000c =	0x9CC9B49A	// NoC DDRTIMING according to MMDC0_MDCFG0/1/2
setmem /32	0x00bb0038 =	0x00000544	// NoC Activate
setmem /32	0x00bb0014 =	0x00000040 	// NoC Read latency
setmem /32	0x00bb0028 =	0x00000020 	// NoC Aging control for IPU1/PRE0/PRE3
setmem /32	0x00bb002c =	0x00000020 	// NoC Aging control for IPU2/PRE1/PRE2
setmem /32	0x00BB0010 =	0x00000000	// DDRMODE, contains extended MSB of BwRatio field, which is zero for 64- and 32-bit wide DDR memories.
//######################################################

// Mode register writes, CH0, CS0
setmem /32	0x021b001c =	0x00008010	// PRE-CHARGE ALL
setmem /32	0x021b001c =	0x003F8030	// MRW: BA=0 CS=0 MR_ADDR=63 MR_OP=0 (Reset)
setmem /32	0x021b001c =	0xFF0A8030	// MRW: BA=0 CS=0 MR_ADDR=10 MR_OP=0xff (IO calibration, calibration code)
setmem /32	0x021b001c =	0xC2018030	// MRW: BA=0 CS=0 MR_ADDR=1  MR_OP=see Register Configuration
setmem /32	0x021b001c =	0x06028030	// MRW: BA=0 CS=0 MR_ADDR=2  MR_OP=see Register Configuration
setmem /32	0x021b001c =	0x01038030	// MRW: BA=0 CS=0 MR_ADDR=3  MR_OP=see Register Configuration

// Mode register writes, CH0, CS1
setmem /32	0x021b001c =	0x00008018	// PRE-CHARGE ALL
setmem /32	0x021b001c =	0x003F8038	// MRW: BA=0 CS=1 MR_ADDR=63 MR_OP=0 (Reset)
setmem /32	0x021b001c =	0xFF0A8038	// MRW: BA=0 CS=1 MR_ADDR=10 MR_OP=0xff (IO calibration, calibration code)
setmem /32	0x021b001c =	0xC2018038	// MRW: BA=0 CS=1 MR_ADDR=1  MR_OP=see Register Configuration
setmem /32	0x021b001c =	0x06028038	// MRW: BA=0 CS=1 MR_ADDR=2  MR_OP=see Register Configuration
setmem /32	0x021b001c =	0x01038038	// MRW: BA=0 CS=1 MR_ADDR=3  MR_OP=see Register Configuration

// Mode register writes, CH1, CS0
setmem /32	0x021b401c =	0x00008010	// PRE-CHARGE ALL
setmem /32	0x021b401c =	0x003F8030	// MRW: BA=0 CS=0 MR_ADDR=63 MR_OP=0 (Reset)
setmem /32	0x021b401c =	0xFF0A8030	// MRW: BA=0 CS=0 MR_ADDR=10 MR_OP=0xff (IO calibration, calibration code)
setmem /32	0x021b401c =	0xC2018030	// MRW: BA=0 CS=0 MR_ADDR=1  MR_OP=see Register Configuration
setmem /32	0x021b401c =	0x06028030	// MRW: BA=0 CS=0 MR_ADDR=2  MR_OP=see Register Configuration
setmem /32	0x021b401c =	0x01038030	// MRW: BA=0 CS=0 MR_ADDR=3  MR_OP=see Register Configuration

// Mode register writes, CH1, CS1
setmem /32	0x021b401c =	0x00008018	// PRE-CHARGE ALL
setmem /32	0x021b401c =	0x003F8038	// MRW: BA=0 CS=1 MR_ADDR=63 MR_OP=0 (Reset)
setmem /32	0x021b401c =	0xFF0A8038	// MRW: BA=0 CS=1 MR_ADDR=10 MR_OP=0xff (IO calibration, calibration code)
setmem /32	0x021b401c =	0xC2018038	// MRW: BA=0 CS=1 MR_ADDR=1  MR_OP=see Register Configuration
setmem /32	0x021b401c =	0x06028038	// MRW: BA=0 CS=1 MR_ADDR=2  MR_OP=see Register Configuration
setmem /32	0x021b401c =	0x01038038	// MRW: BA=0 CS=1 MR_ADDR=3  MR_OP=see Register Configuration

//######################################################
//final DDR setup, before operation start:
// It's necessary to disable DQS gating after programming the LPDDR2 mode registers
// in addition to resetting the read FIFO pointers
setmem /32	0x021b083c =	0xA0000000	// Reset read FIFO (perform twice to ensure reset) and PHY0 dqs gating dis
setmem /32	0x021b083c =	0xA0000000	// Reset read FIFO (perform twice to ensure reset) and PHY0 dqs gating dis
setmem /32	0x021b483c =	0xA0000000	// Reset read FIFO (perform twice to ensure reset) and PHY1 dqs gating dis
setmem /32	0x021b483c =	0xA0000000	// Reset read FIFO (perform twice to ensure reset) and PHY1 dqs gating dis

setmem /32	0x021b08b8 =	0x00000800	// DDR_PHY_P0_MPMUR0, frc_msr to update PHY with register changes
setmem /32	0x021b48b8 =	0x00000800	// DDR_PHY_P1_MPMUR0, frc_msr to update PHY with register changes

setmem /32	0x021b0020 =	0x00001800	// MMDC0_MDREF
setmem /32	0x021b4020 =	0x00001800	// MMDC1_MDREF

setmem /32	0x021b0818 =	0x00000000	// DDR_PHY_P0_MPODTCTRL
setmem /32	0x021b4818 =	0x00000000	// DDR_PHY_P1_MPODTCTRL

setmem /32	0x021b0004 =	0x00025564	// MMDC0_MDPDC with PWDT bits set
setmem /32	0x021b4004 =	0x00025564	// MMDC1_MDPDC with PWDT bits set

setmem /32	0x021b0404 = 	0x00011006	// MMDC0_MAPSR ADOPT power down enabled, MMDC will enter automatically to self-refresh while the number of idle cycle reached.
setmem /32	0x021b4404 = 	0x00011006	// MMDC1_MAPSR ADOPT power down enabled, MMDC will enter automatically to self-refresh while the number of idle cycle reached.

setmem /32	0x021b001c =	0x00000000	// MMDC0_MDSCR, clear this register (especially the configuration bit as initialization is complete)
setmem /32	0x021b401c =	0x00000000	// MMDC1_MDSCR, clear this register (especially the configuration bit as initialization is complete)
