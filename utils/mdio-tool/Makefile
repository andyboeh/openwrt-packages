include $(TOPDIR)/rules.mk

PKG_NAME:=mdio-tool
PKG_RELEASE:=1

PKG_RELEASE=$(PKG_SOURCE_VERSION)

PKG_SOURCE_PROTO:=git
PKG_SOURCE_URL:=https://github.com/Menion2k/mdio-tool
PKG_SOURCE_DATE:=2019-12-30

PKG_LICENSE:=GPL-2.0
PKG_LICENSE_FILES:=COPYING
PKG_MIRROR_HASH=:=skip
PKG_SOURCE_VERSION=ab16e7f374367c7313de9448db8420ee686f924b

PKG_MAINTAINER:=Andreas Boehler <dev@aboehler.at>

include $(INCLUDE_DIR)/package.mk

define Package/mdio-tool
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=MDIO Tool
  URL:=https://github.com/Menion2k/mdio-tool.git
endef

define Package/mdio-tool/description
	read and write MII registers from ethernet physicals under linux
endef

define Build/Configure
endef

define Build/Compile
	$(MAKE) -C $(PKG_BUILD_DIR) \
		CC="$(TARGET_CC)" \
		CFLAGS="$(TARGET_CFLAGS) -Wall" \
		LDFLAGS="$(TARGET_LDFLAGS)"
endef

define Package/mdio-tool/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/mdio-tool $(1)/usr/bin/
endef

$(eval $(call BuildPackage,mdio-tool))
