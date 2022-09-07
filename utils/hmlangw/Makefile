include $(TOPDIR)/rules.mk

PKG_NAME:=hmlangw
PKG_RELEASE:=1
PKG_VERSION=0.0.3

PKG_LICENSE:=GPL-2.0

PKG_SOURCE_PROTO:=git
PKG_SOURCE_URL:=https://github.com/andyboeh/hmlangw.git
PKG_SOURCE_DATE:=2022-09-07
PKG_MIRROR_HASH=:=skip
PKG_SOURCE_VERSION=a719b6c9226790fed61b7ac1bc748c7a8297c841

PKG_MAINTAINER:=Andreas Boehler <dev@aboehler.at>

include $(INCLUDE_DIR)/package.mk

define Package/hmlangw
	SECTION:=utils
	CATEGORY:=Utilities
	DEPENDS:=+libpthread
	TITLE:=Homematic LAN gateway simulation
endef

define Package/hmlangw/description
	Simulate a Homematic LAN gateway using a HM-MOD-UART connected to your 
	OpenWrt device
endef

define Package/hmlangw/conffiles
/etc/config/hmlangw
endef

define Build/Compile
	$(MAKE) -C $(PKG_BUILD_DIR) \
		CC="$(TARGET_CC)" \
		CFLAGS="$(TARGET_CFLAGS) -Wall" \
		LDFLAGS="$(TARGET_LDFLAGS)"
endef

define Package/hmlangw/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/hmlangw $(1)/usr/bin/

	$(INSTALL_DIR) $(1)/etc/config
	$(INSTALL_CONF) ./files/hmlangw.config $(1)/etc/config/hmlangw

	$(INSTALL_DIR) $(1)/etc/init.d
	$(INSTALL_BIN) ./files/hmlangw.init $(1)/etc/init.d/hmlangw
endef

$(eval $(call BuildPackage,hmlangw))
