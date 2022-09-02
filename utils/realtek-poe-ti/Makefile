include $(TOPDIR)/rules.mk

PKG_NAME:=realtek-poe-ti
PKG_RELEASE:=1
CMAKE_INSTALL:=1

include $(INCLUDE_DIR)/package.mk
include $(INCLUDE_DIR)/cmake.mk

define Package/realtek-poe-ti
  TITLE:=Utility to manage PoE ports with TSP23861 chipsets
  SECTION:=utils
  CATEGORY:=Utilities
  DEPENDS:=@TARGET_realtek +libubox +libubus +libuci +libi2c
endef

define Package/realtek-poe-ti/description
  Utility to manage PoE ports with TSP23861 chipsets
endef

define Package/realtek-poe-ti/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/realtek-poe-ti $(1)/usr/bin/realtek-poe-ti
	$(CP) ./files/* $(1)
endef

$(eval $(call BuildPackage,realtek-poe-ti))
