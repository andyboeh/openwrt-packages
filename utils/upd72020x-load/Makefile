include $(TOPDIR)/rules.mk

PKG_NAME:=upd72020x-load
PKG_RELEASE:=1

PKG_RELEASE=$(PKG_SOURCE_VERSION)

PKG_SOURCE_PROTO:=git
PKG_SOURCE_URL:=https://github.com/andyboeh/upd72020x-load
PKG_SOURCE_DATE:=2020-01-25

PKG_LICENSE:=GPL-2.0
PKG_LICENSE_FILES:=COPYING
PKG_MIRROR_HASH=:=skip
PKG_SOURCE_VERSION=ba9330b201f88e5d15e54cff15ec933f45a86ca2

PKG_MAINTAINER:=Andreas Boehler <dev@aboehler.at>

include $(INCLUDE_DIR)/package.mk

define Package/upd72020x-load
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=Renesas uPD72020x firmware loader
  URL:=https://github.com/andyboeh/upd72020x-load.git
endef

define Package/upd72020x-load/description
	upload firmware to the RAM of Renesas uPD7202720x devices
endef

define Build/Configure
endef

define Build/Compile
	$(MAKE) -C $(PKG_BUILD_DIR) \
		CC="$(TARGET_CC)" \
		CFLAGS="$(TARGET_CFLAGS) -Wall" \
		LDFLAGS="$(TARGET_LDFLAGS)"
endef

define Package/upd72020x-load/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/upd72020x-load $(1)/usr/bin/
endef

$(eval $(call BuildPackage,upd72020x-load))
