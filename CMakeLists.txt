
cmake_minimum_required(VERSION 3.5)

if(DEFINED ENV{HOMEKIT_PATH})
  set(HOMEKIT_PATH $ENV{HOMEKIT_PATH})
else()
  set(HOMEKIT_PATH ${CMAKE_CURRENT_LIST_DIR}/components/esp-homekit-sdk)
endif(DEFINED ENV{HOMEKIT_PATH})

if (EXISTS ${HOMEKIT_PATH})
  message(WARNING "HOMEKIT_PATH=" ${HOMEKIT_PATH})
else()
  message(ERROR "Homekit path is not valid: HOMEKIT_PATH=" ${HOMEKIT_PATH})
endif()

set(EXTRA_COMPONENT_DIRS
    ${HOMEKIT_PATH}/components
    ${HOMEKIT_PATH}/components/homekit
    ${HOMEKIT_PATH}/examples/common/app_hap_setup_payload
    ${HOMEKIT_PATH}/examples/common/qrcode
    )

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(garagedoor)
