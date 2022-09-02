# openwrt-packages
Local OpenWrt packages

## realtek-poe-ti

This package is a daemon to manage TI TPS23861 PSE controllers on Realtek-based managed switches. It was written for a TP-Link TL-SG2452P that makes use of 12x TPS23861 chips.

There is an in-kernel hwmon driver for this IC, but it only supports Auto-mode and is not capable for initializing the chip. An initialization PR was turned down by the maintainers.

The daemon makes sure that the mapping of LAN ports to PoE ports is correct, initializes and monitors the PSE controllers and will be responsible for managing the power budget.