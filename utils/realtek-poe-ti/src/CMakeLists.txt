cmake_minimum_required(VERSION 2.6)

PROJECT(realtek-poe-ti C)
ADD_DEFINITIONS(-Wall -Werror --std=gnu99 -Wmissing-declarations)

SET(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")

FIND_LIBRARY(ubus NAMES ubus)
FIND_LIBRARY(uci NAMES uci)
FIND_LIBRARY(ubox NAMES ubox)
FIND_LIBRARY(i2c NAMES i2c)
FIND_PATH(uci_include_dir NAMES uci.h)
FIND_PATH(ubus_include_dir NAMES libubus.h)
FIND_PATH(ubox_include_dir NAMES libubox/usock.h)
FIND_PATH(i2c_include_dir NAMES i2c/smbus.h)
INCLUDE_DIRECTORIES(${ubox_include_dir} ${ubus_include_dir} ${uci_include_dir} ${i2c_include_dir})



ADD_EXECUTABLE(realtek-poe-ti realtek-poe-ti.c tps23861.c)
TARGET_LINK_LIBRARIES(realtek-poe-ti ${ubox} ${ubus} ${uci} ${i2c})

INSTALL(TARGETS realtek-poe-ti RUNTIME DESTINATION bin)
