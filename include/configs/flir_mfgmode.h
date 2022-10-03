/*
 * File used to define what manufacturing mode the u-boot should be generated for.
 *
 * This file is patched by the yocto recipes for the different u-boot variants.
 * The valid values are
 * 0 for normal boot from EMMC
 * 1 fusing and setting up partitions from usb boot
 * 2 run recboot without fusing and partitioning
 *
 */

#define CONFIG_FLIR_MFG 0
