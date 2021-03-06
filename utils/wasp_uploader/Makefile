include $(TOPDIR)/rules.mk

PKG_NAME:=wasp_uploader
PKG_RELEASE:=1

PKG_RELEASE=$(PKG_SOURCE_VERSION)

PKG_SOURCE_PROTO:=git
PKG_SOURCE_URL:=https://github.com/andyboeh/wasp_uploader.git
PKG_SOURCE_DATE:=2019-12-16

PKG_LICENSE:=GPL-2.0
PKG_LICENSE_FILES:=COPYING
PKG_MIRROR_HASH=:=skip
PKG_SOURCE_VERSION=af15586ad1df6abbfb952064f689ed5b1f63c63b

PKG_MAINTAINER:=Andreas Boehler <dev@aboehler.at>

include $(INCLUDE_DIR)/package.mk

define Package/wasp_uploader
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=AVM WASP firmware uploader
  URL:=https://github.com/andyboeh/wasp_uploader.git
endef

define Package/wasp_uploader/description
   Upload firmware to AVM WASP devices
endef

define Build/Configure
endef

define Build/Compile
	$(MAKE) -C $(PKG_BUILD_DIR) \
		CC="$(TARGET_CC)" \
		CFLAGS="$(TARGET_CFLAGS) -Wall" \
		LDFLAGS="$(TARGET_LDFLAGS)"
endef

define Package/wasp_uploader/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/wasp_uploader_stage1 $(1)/usr/bin/
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/wasp_uploader_stage2 $(1)/usr/bin/
	$(INSTALL_DIR) $(1)/opt/wasp
	$(CP) $(PKG_BUILD_DIR)/files/upload-wasp.sh $(1)/opt/wasp/
endef

$(eval $(call BuildPackage,wasp_uploader))
