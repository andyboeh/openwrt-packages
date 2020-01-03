include $(TOPDIR)/rules.mk

PKG_NAME:=wasp_downloader
PKG_RELEASE:=1

PKG_RELEASE=$(PKG_SOURCE_VERSION)

PKG_SOURCE_PROTO:=git
PKG_SOURCE_URL:=https://github.com/andyboeh/wasp_downloader.git
PKG_SOURCE_DATE:=2020-01-02

PKG_LICENSE:=GPL-2.0
PKG_LICENSE_FILES:=COPYING
PKG_MIRROR_HASH=:=skip
PKG_SOURCE_VERSION=d9ae756e542cb1569da8c15a3e79180ead160416

PKG_MAINTAINER:=Andreas Boehler <dev@aboehler.at>

include $(INCLUDE_DIR)/package.mk

define Package/wasp_downloader
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=AVM WASP firmware downloader
  URL:=https://github.com/andyboeh/wasp_downloader.git
endef

define Package/wasp_downloader/description
   Download configuration files to AVM WASP devices
endef

define Build/Configure
endef

define Build/Compile
	$(MAKE) -C $(PKG_BUILD_DIR) \
		CC="$(TARGET_CC)" \
		CFLAGS="$(TARGET_CFLAGS) -Wall" \
		LDFLAGS="$(TARGET_LDFLAGS)"
endef

define Package/wasp_downloader/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/wasp_downloader $(1)/usr/bin/
	[ -d ./files ] && $(CP) ./files/* $(1)/
endef

$(eval $(call BuildPackage,wasp_downloader))
