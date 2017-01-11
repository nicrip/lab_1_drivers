#--------------------------------------------------------
# The CMakeLists.txt for:             mcc_usb_1608fs_plus
# Author(s):                                 Nick Rypkema
#--------------------------------------------------------

cmake_minimum_required(VERSION 2.6)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUNIX -D_LINUX=1 -DCMDLINE=1")

# mcc_usb_1608fs_plus library
SET(SRC
  usb-1608FS-Plus.c
  pmd.c
)

ADD_LIBRARY(mcc_usb_1608fs_plus SHARED ${SRC})

# test mcc_usb_1608fs_plus library
SET(SRC
  test-usb1608FS-Plus.c
)

ADD_EXECUTABLE(test-usb1608FS-Plus ${SRC})

TARGET_LINK_LIBRARIES(test-usb1608FS-Plus
   mcc_usb_1608fs_plus m 
   pthread 
   -lc 
   -lm 
   -lhidapi-libusb 
   -lusb-1.0)

SET_TARGET_PROPERTIES(mcc_usb_1608fs_plus PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION "${PROJECT_SOVERSION}")

