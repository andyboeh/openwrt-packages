include $(TOPDIR)/rules.mk

PKG_NAME:=qtools
PKG_RELEASE:=1
PKG_VERSION=0.0.1

PKG_LICENSE:=GPL-3.0

PKG_SOURCE_PROTO:=git
PKG_SOURCE_URL:=https://github.com/forth32/qtools.git
PKG_SOURCE_DATE:=2018-06-25
PKG_MIRROR_HASH=:=skip
PKG_SOURCE_VERSION=da1d2d0acda2b512253408fbc15a8cce466eb4a5

PKG_MAINTAINER:=Andreas Boehler <dev@aboehler.at>

include $(INCLUDE_DIR)/package.mk

define Package/qtools
	SECTION:=utils
	CATEGORY:=Utilities
	DEPENDS:=+libreadline
	TITLE:=Unofficial Qualcomm Modem Utilities (qtools)
endef

define Package/qtools/description
	Tools to modify the flash memory and other specialties of Qualcomm modems
endef

define Build/Compile
	$(MAKE) -C $(PKG_BUILD_DIR) \
		CC="$(TARGET_CC)" \
		CFLAGS="$(TARGET_CFLAGS) -Wall" \
		LDFLAGS="$(TARGET_LDFLAGS)"
endef

define Package/qtools/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/qcommand $(1)/usr/bin/

#	$(INSTALL_DIR) $(1)/etc/config
#	$(INSTALL_CONF) ./files/hmlangw.config $(1)/etc/config/hmlangw

#	$(INSTALL_DIR) $(1)/etc/init.d
#	$(INSTALL_BIN) ./files/hmlangw.init $(1)/etc/init.d/hmlangw
endef

$(eval $(call BuildPackage,qtools))
